#!/usr/bin/env python3

from __future__ import annotations

import argparse
import os
import select
import signal
import subprocess
import sys
import termios
import time
import tty
from dataclasses import dataclass

import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosbag2_interfaces.srv import IsPaused, Pause, Resume, Seek, TogglePaused
from rosgraph_msgs.msg import Clock


BACKSPACE_KEYS = {"\x7f", "\x08"}


@dataclass
class PendingAction:
    kind: str
    future: object
    resume_after_seek: bool = False
    seek_target_sec: float | None = None
    seek_target_ros_sec: float | None = None


class PlaybackKeyboardController(Node):
    def __init__(
        self,
        start_offset_sec: float,
        rewind_seconds: float,
        auto_start: bool,
        player_pid: int,
        player_node: str,
        rviz_config: str | None,
        rviz_pid_file: str | None,
        rviz_launcher: str | None,
        refresh_rviz_on_rewind: bool,
    ) -> None:
        super().__init__("playback_keyboard_controller")
        self.start_offset_sec = float(start_offset_sec)
        self.rewind_seconds = float(rewind_seconds)
        self.auto_start = auto_start
        self.player_pid = int(player_pid)
        self.player_node = player_node.rstrip("/")
        self.rviz_config = rviz_config
        self.rviz_pid_file = rviz_pid_file
        self.rviz_launcher = rviz_launcher
        self.refresh_rviz_on_rewind = refresh_rviz_on_rewind

        self.first_clock_sec: float | None = None
        self.current_clock_sec: float | None = None
        self.current_bag_time_sec = self.start_offset_sec
        self.last_printed_whole_sec: int | None = None
        self.pending_action: PendingAction | None = None
        self.auto_start_issued = False

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Clock, "/clock", self.on_clock, qos)

        self.pause_client = self.create_client(Pause, f"{self.player_node}/pause")
        self.resume_client = self.create_client(Resume, f"{self.player_node}/resume")
        self.toggle_client = self.create_client(TogglePaused, f"{self.player_node}/toggle_paused")
        self.is_paused_client = self.create_client(IsPaused, f"{self.player_node}/is_paused")
        self.seek_client = self.create_client(Seek, f"{self.player_node}/seek")

        self.create_timer(0.05, self.on_timer)

    def on_clock(self, msg: Clock) -> None:
        current_clock_sec = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.first_clock_sec is None:
            self.first_clock_sec = current_clock_sec

        self.current_clock_sec = current_clock_sec
        self.current_bag_time_sec = self.start_offset_sec + (current_clock_sec - self.first_clock_sec)
        whole_sec = int(self.current_bag_time_sec)

        if whole_sec != self.last_printed_whole_sec:
            print(f"[CLOCK] 当前回放到 bag 第 {self.current_bag_time_sec:6.2f} s", flush=True)
            self.last_printed_whole_sec = whole_sec

    def player_services_ready(self) -> bool:
        return all(
            client.service_is_ready()
            for client in (
                self.pause_client,
                self.resume_client,
                self.toggle_client,
                self.is_paused_client,
                self.seek_client,
            )
        )

    def player_alive(self) -> bool:
        try:
            os.kill(self.player_pid, 0)
            return True
        except OSError:
            return False

    def on_timer(self) -> None:
        if not self.player_alive():
            print("[INFO] 回放进程已退出，键盘控制结束。", flush=True)
            raise SystemExit(0)

        if self.pending_action is not None and self.pending_action.future.done():
            self.handle_action_done()

        if self.pending_action is None and self.auto_start and not self.auto_start_issued and self.player_services_ready():
            self.auto_start_issued = True
            self.request_resume()
            return

        if self.pending_action is None:
            self.handle_keyboard()

    def handle_action_done(self) -> None:
        assert self.pending_action is not None
        action = self.pending_action
        self.pending_action = None

        try:
            response = action.future.result()
        except Exception as exc:  # noqa: BLE001
            print(f"[WARN] {action.kind} 调用失败: {exc}", flush=True)
            return

        if action.kind == "toggle":
            print("[INFO] 已切换暂停状态", flush=True)
            return

        if action.kind == "pause":
            print("[INFO] 已暂停", flush=True)
            return

        if action.kind == "resume":
            print("[INFO] 已继续播放", flush=True)
            return

        if action.kind == "pause_then_seek":
            self.request_seek(resume_after_seek=True)
            return

        if action.kind == "seek":
            success = bool(getattr(response, "success", False))
            if success:
                print(
                    f"[INFO] 已回退到 bag 第 {action.seek_target_sec:6.2f} s",
                    flush=True,
                )
                if self.refresh_rviz_on_rewind:
                    self.refresh_rviz()
                if action.resume_after_seek:
                    self.request_resume()
            else:
                print("[WARN] seek 失败，可能目标时间超出 bag 范围。", flush=True)
            return

        if action.kind == "rewind_prepare":
            paused = bool(getattr(response, "paused", False))
            if not paused:
                self.request_pause_and_then_seek()
            else:
                self.request_seek(resume_after_seek=False)

    def handle_keyboard(self) -> None:
        while True:
            ready, _, _ = select.select([sys.stdin], [], [], 0.0)
            if not ready:
                return

            key = sys.stdin.read(1)
            if key == " ":
                self.request_toggle()
            elif key in BACKSPACE_KEYS:
                self.request_rewind()
            elif key in {"r", "R"}:
                self.refresh_rviz()
            elif key in {"q", "Q"}:
                print("[INFO] 收到退出指令 q，结束键盘控制。", flush=True)
                raise SystemExit(0)

            if self.pending_action is not None:
                return

    def request_toggle(self) -> None:
        if not self.toggle_client.service_is_ready():
            print("[WARN] toggle_paused 服务还没就绪。", flush=True)
            return
        future = self.toggle_client.call_async(TogglePaused.Request())
        self.pending_action = PendingAction(kind="toggle", future=future)

    def request_pause(self) -> None:
        if not self.pause_client.service_is_ready():
            print("[WARN] pause 服务还没就绪。", flush=True)
            return
        future = self.pause_client.call_async(Pause.Request())
        self.pending_action = PendingAction(kind="pause", future=future)

    def request_resume(self) -> None:
        if not self.resume_client.service_is_ready():
            print("[WARN] resume 服务还没就绪。", flush=True)
            return
        future = self.resume_client.call_async(Resume.Request())
        self.pending_action = PendingAction(kind="resume", future=future)

    def request_rewind(self) -> None:
        if self.current_clock_sec is None or self.first_clock_sec is None:
            print("[WARN] 还没收到 /clock，暂时不能回退。", flush=True)
            return
        if not self.is_paused_client.service_is_ready():
            print("[WARN] is_paused 服务还没就绪。", flush=True)
            return
        future = self.is_paused_client.call_async(IsPaused.Request())
        self.pending_action = PendingAction(kind="rewind_prepare", future=future)

    def request_pause_and_then_seek(self) -> None:
        if not self.pause_client.service_is_ready():
            print("[WARN] pause 服务还没就绪。", flush=True)
            return
        future = self.pause_client.call_async(Pause.Request())
        self.pending_action = PendingAction(kind="pause_then_seek", future=future)

    def request_seek(self, resume_after_seek: bool) -> None:
        if self.current_clock_sec is None or self.first_clock_sec is None:
            print("[WARN] 还没收到 /clock，暂时不能 seek。", flush=True)
            return
        if not self.seek_client.service_is_ready():
            print("[WARN] seek 服务还没就绪。", flush=True)
            return

        bag_start_ros_sec = self.first_clock_sec - self.start_offset_sec
        target_ros_sec = max(bag_start_ros_sec, self.current_clock_sec - self.rewind_seconds)
        target_bag_sec = self.start_offset_sec + (target_ros_sec - self.first_clock_sec)

        request = Seek.Request()
        request.time = to_builtin_time(target_ros_sec)
        future = self.seek_client.call_async(request)
        self.pending_action = PendingAction(
            kind="seek",
            future=future,
            resume_after_seek=resume_after_seek,
            seek_target_sec=target_bag_sec,
            seek_target_ros_sec=target_ros_sec,
        )

    def refresh_rviz(self) -> bool:
        if not self.rviz_config or not self.rviz_pid_file or not self.rviz_launcher:
            print("[WARN] 没有提供 RViz 刷新参数，跳过刷新。", flush=True)
            return False

        old_pid = self.read_pid_file()
        if old_pid is not None:
            self.terminate_process(old_pid, "RViz")

        try:
            proc = subprocess.Popen(
                ["/bin/bash", self.rviz_launcher, self.rviz_config],
                stdin=subprocess.DEVNULL,
                start_new_session=True,
            )
        except Exception as exc:  # noqa: BLE001
            print(f"[WARN] 重新启动 RViz 失败: {exc}", flush=True)
            return False

        try:
            with open(self.rviz_pid_file, "w", encoding="utf-8") as handle:
                handle.write(f"{proc.pid}\n")
        except OSError as exc:
            print(f"[WARN] 写入 RViz pid 文件失败: {exc}", flush=True)

        print("[INFO] 已刷新 RViz，清掉旧的 TF/Marker 缓存。", flush=True)
        time.sleep(1.0)
        return True

    def read_pid_file(self) -> int | None:
        if not self.rviz_pid_file or not os.path.exists(self.rviz_pid_file):
            return None
        try:
            with open(self.rviz_pid_file, "r", encoding="utf-8") as handle:
                raw = handle.read().strip()
        except OSError:
            return None

        if not raw:
            return None

        try:
            return int(raw)
        except ValueError:
            return None

    def terminate_process(self, pid: int, label: str) -> None:
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            return

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if not self.process_alive(pid):
                return
            time.sleep(0.1)

        try:
            os.kill(pid, signal.SIGKILL)
        except OSError:
            return
        print(f"[WARN] {label} 退出较慢，已强制结束旧进程。", flush=True)

    @staticmethod
    def process_alive(pid: int) -> bool:
        try:
            os.kill(pid, 0)
            return True
        except OSError:
            return False


def to_builtin_time(seconds: float) -> Time:
    msg = Time()
    whole = int(seconds)
    frac = seconds - whole
    if frac < 0:
        whole -= 1
        frac += 1.0
    msg.sec = whole
    msg.nanosec = int(frac * 1e9)
    return msg


class RawStdin:
    def __enter__(self) -> "RawStdin":
        self.fd = sys.stdin.fileno()
        self.old_attrs = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_attrs)


def install_signal_handlers() -> None:
    def _handler(signum, frame) -> None:  # noqa: ARG001
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _handler)
    signal.signal(signal.SIGTERM, _handler)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Keyboard controller for ros2 bag playback: Space pause/resume, Backspace rewind N seconds."
    )
    parser.add_argument("--start-offset", type=float, default=0.0)
    parser.add_argument("--rewind-seconds", type=float, default=5.0)
    parser.add_argument("--auto-start", action="store_true")
    parser.add_argument("--player-pid", type=int, required=True)
    parser.add_argument("--player-node", default="/rosbag2_player")
    parser.add_argument("--rviz-config")
    parser.add_argument("--rviz-pid-file")
    parser.add_argument("--rviz-launcher")
    parser.add_argument("--refresh-rviz-on-rewind", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    install_signal_handlers()

    print(
        "[INFO] 键盘控制已启用: SPACE 暂停/继续, Backspace 回退 "
        f"{args.rewind_seconds:.2f}s, r 刷新 RViz, q 退出控制",
        flush=True,
    )
    if not args.auto_start:
        print("[INFO] 播放器会以暂停态启动，请按一次 SPACE 开始播放。", flush=True)

    rclpy.init()
    node = PlaybackKeyboardController(
        start_offset_sec=args.start_offset,
        rewind_seconds=args.rewind_seconds,
        auto_start=args.auto_start,
        player_pid=args.player_pid,
        player_node=args.player_node,
        rviz_config=args.rviz_config,
        rviz_pid_file=args.rviz_pid_file,
        rviz_launcher=args.rviz_launcher,
        refresh_rviz_on_rewind=args.refresh_rviz_on_rewind,
    )

    try:
        with RawStdin():
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit as exc:
        return int(exc.code or 0)
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

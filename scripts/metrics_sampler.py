#!/usr/bin/env python3
"""
Lightweight system metrics sampler (CPU %, MEM %, NET state) that writes a key=value
status file compatible with DomainControllerMonitor. No external deps.
"""
from __future__ import annotations
import argparse
import time
from pathlib import Path


def clamp(val: float, lo: float = 0.0, hi: float = 100.0) -> float:
    return max(lo, min(hi, val))


class ProcSampler:
    def __init__(self) -> None:
        self._prev_total = None
        self._prev_idle = None

    def cpu_percent(self) -> float:
        try:
            with open("/proc/stat", "r", encoding="utf-8") as f:
                line = f.readline()
            parts = line.strip().split()
            if len(parts) < 5 or parts[0] != "cpu":
                return 0.0
            user, nice, system, idle = (float(parts[i]) for i in range(1, 5))
            iowait = float(parts[5]) if len(parts) > 5 else 0.0
            irq = float(parts[6]) if len(parts) > 6 else 0.0
            softirq = float(parts[7]) if len(parts) > 7 else 0.0
            steal = float(parts[8]) if len(parts) > 8 else 0.0
            idle_all = idle + iowait
            total = user + nice + system + idle_all + irq + softirq + steal
            if self._prev_total is None or self._prev_idle is None:
                self._prev_total = total
                self._prev_idle = idle_all
                return 0.0
            delta_total = total - self._prev_total
            delta_idle = idle_all - self._prev_idle
            self._prev_total = total
            self._prev_idle = idle_all
            if delta_total <= 0.0:
                return 0.0
            return clamp((1.0 - delta_idle / delta_total) * 100.0)
        except Exception:
            return 0.0

    def mem_percent(self) -> float:
        try:
            mem_total = 0.0
            mem_available = 0.0
            with open("/proc/meminfo", "r", encoding="utf-8") as f:
                for line in f:
                    if line.startswith("MemTotal:"):
                        mem_total = float(line.split()[1])
                    elif line.startswith("MemAvailable:"):
                        mem_available = float(line.split()[1])
                    if mem_total and mem_available:
                        break
            if mem_total <= 0.0:
                return 0.0
            used = mem_total - mem_available
            return clamp((used / mem_total) * 100.0)
        except Exception:
            return 0.0

    def network_status(self) -> int:
        """
        0=offline, 1=connecting (iface exists), 2=online (default route + up)
        """
        try:
            default_iface = None
            with open("/proc/net/route", "r", encoding="utf-8") as f:
                header = f.readline()
                for line in f:
                    parts = line.split()[:3]
                    if len(parts) < 3:
                        continue
                    iface, destination, _gateway = parts
                    if destination == "00000000":
                        default_iface = iface
                        break
            def operstate(iface: str) -> str:
                try:
                    with open(f"/sys/class/net/{iface}/operstate", "r", encoding="utf-8") as f:
                        return f.read().strip().lower()
                except Exception:
                    return "down"

            if default_iface:
                state = operstate(default_iface)
                if state == "up":
                    return 2
                if state in ("dormant", "unknown", "testing"):
                    return 1
                return 0

            # fallback: any non-lo iface that is up → connecting
            try:
                for path in Path("/sys/class/net").iterdir():
                    if path.name == "lo":
                        continue
                    if operstate(path.name) == "up":
                        return 1
            except Exception:
                pass
            return 0
        except Exception:
            return 0


def write_status(path: Path, cpu: float, mem: float, net: int) -> None:
    content = f"cpu={cpu:.2f}\nmem={mem:.2f}\nnetwork={net}\n"
    path.write_text(content, encoding="utf-8")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="/proc-based metrics sampler")
    p.add_argument("--output", "-o", default="/tmp/domain_controller_status.txt",
                   help="status file path (key=value lines)")
    p.add_argument("--interval", "-i", type=float, default=1.0,
                   help="sampling interval seconds")
    p.add_argument("--once", action="store_true", help="sample once and exit")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    sampler = ProcSampler()
    out_path = Path(args.output)

    # First read primes CPU deltas; take two samples if running once
    cpu = sampler.cpu_percent()
    mem = sampler.mem_percent()
    net = sampler.network_status()
    if args.once:
        time.sleep(args.interval)
        cpu = sampler.cpu_percent()
        mem = sampler.mem_percent()
        net = sampler.network_status()
        write_status(out_path, cpu, mem, net)
        print(f"wrote {out_path} cpu={cpu:.2f} mem={mem:.2f} net={net}")
        return

    while True:
        cpu = sampler.cpu_percent()
        mem = sampler.mem_percent()
        net = sampler.network_status()
        write_status(out_path, cpu, mem, net)
        print(f"cpu={cpu:.2f} mem={mem:.2f} net={net}")
        time.sleep(max(args.interval, 0.1))


if __name__ == "__main__":
    main()

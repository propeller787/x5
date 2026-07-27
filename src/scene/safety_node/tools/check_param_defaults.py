#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""方案 9.2 参数默认值漂移检查。

对比 config/safety_node_params.yaml 的生产值与 C++ 加载器
（safety_params.cpp / safety_node.cpp declareAndLoadParams）中的 fallback 默认值，
输出漂移清单。缺参数文件启动时行为不应与生产意图相反——代码默认值应与 yaml
一致或取更保守侧。

用法（包根目录下）:
    python tools/check_param_defaults.py [--strict]
--strict 时存在漂移返回退出码 1（供 CI 门禁）。

限制：只解析 get_d/get_i/get_b/get_s 与 declare_parameter<T> 两种写法；
按扁平参数键（a.b.c）比对标量，列表/嵌套结构跳过。
"""
import argparse
import re
import sys
from pathlib import Path

PKG_ROOT = Path(__file__).resolve().parent.parent
YAML_PATH = PKG_ROOT / "config" / "safety_node_params.yaml"
CPP_SOURCES = [
    PKG_ROOT / "src" / "safety_params.cpp",
    PKG_ROOT / "src" / "safety_node.cpp",
]

# get_x("key", default) —— default 可能是数字/true/false/字符串字面量
GETTER_RE = re.compile(
    r'get_[dibs]\(\s*"(?P<key>[^"]+)"\s*,\s*(?P<def>[^)]+?)\s*\)')
# declare_parameter<T>("key", default)
DECLARE_RE = re.compile(
    r'declare_parameter<[^>]+>\(\s*"(?P<key>[^"]+)"\s*,\s*(?P<def>[^)]+?)\s*\)')


def normalize(value: str):
    value = value.strip().rstrip("fF")
    if value in ("true", "True"):
        return True
    if value in ("false", "False"):
        return False
    if value.startswith('"') and value.endswith('"'):
        return value.strip('"')
    try:
        num = float(value)
        return num
    except ValueError:
        return None  # 表达式默认值（如嵌套 get_d 回退），跳过


def parse_cpp_defaults():
    defaults = {}
    for path in CPP_SOURCES:
        text = path.read_text(encoding="utf-8")
        for regex in (GETTER_RE, DECLARE_RE):
            for match in regex.finditer(text):
                key = match.group("key")
                value = normalize(match.group("def"))
                if value is None:
                    continue
                # 同键多处出现取首次（loadFromNode 先 declare 者生效）
                defaults.setdefault(key, value)
    return defaults


def parse_yaml_flat():
    """极简 YAML 扁平化：按缩进层级拼接键名，只取标量行。"""
    flat = {}
    stack = []  # (indent, key)
    for raw in YAML_PATH.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        indent = len(line) - len(line.lstrip())
        stripped = line.strip()
        if stripped in ("/**:", "ros__parameters:"):
            continue
        if ":" not in stripped:
            continue
        key, _, value = stripped.partition(":")
        key = key.strip().strip('"')
        value = value.strip()
        while stack and stack[-1][0] >= indent:
            stack.pop()
        if not value or value == "{}" or value.startswith("{"):
            # 嵌套节（或行内字典——降级矩阵规则行，跳过标量比对）
            if not value:
                stack.append((indent, key))
            continue
        full_key = ".".join([k for _, k in stack] + [key])
        parsed = normalize(value)
        if parsed is not None:
            flat[full_key] = parsed
    return flat


def values_equal(a, b):
    if isinstance(a, bool) or isinstance(b, bool):
        return a == b
    if isinstance(a, float) and isinstance(b, float):
        return abs(a - b) <= 1e-9 * max(1.0, abs(a), abs(b))
    return a == b


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--strict", action="store_true", help="漂移即退出码 1")
    args = parser.parse_args()

    cpp = parse_cpp_defaults()
    yaml_values = parse_yaml_flat()

    drifted = []
    for key, yaml_value in sorted(yaml_values.items()):
        if key not in cpp:
            continue  # yaml 独有键（如 mode.default 由字符串枚举处理）不比对
        if not values_equal(cpp[key], yaml_value):
            drifted.append((key, cpp[key], yaml_value))

    missing_in_yaml = sorted(k for k in cpp if k not in yaml_values)

    if drifted:
        print("== 默认值漂移（代码 fallback ≠ 生产 yaml）==")
        for key, code_value, yaml_value in drifted:
            print(f"  {key}: code={code_value!r}  yaml={yaml_value!r}")
    if missing_in_yaml:
        print("== 代码可加载但 yaml 未显式给值（按代码默认运行）==")
        for key in missing_in_yaml:
            print(f"  {key} (code default={cpp[key]!r})")
    if not drifted and not missing_in_yaml:
        print("默认值与生产 yaml 一致，无漂移。")

    if args.strict and drifted:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())

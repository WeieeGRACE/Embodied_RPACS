#!/usr/bin/env python3
r"""容错版调试日志解码器（主工程 tools/decode_debug_log.ps1 的补充）。

为什么不直接用主工程的脚本：那个脚本用 `^D\s+hex16` 匹配，**带时间戳的抓包
（`[20:36:32.334] D 0000...A060`，read_serial_log.ps1 -Timestamped 的产物）一律匹配不上**，
会静默输出 snapshots=0。本脚本先剥掉 `[hh:mm:ss.fff]` 前缀再解析。

原始行格式（src/uart/system_event_logger.v）：
    S <source> <code>     启动/状态事件（如 "S I 00"）
    E <source> <code>     错误/故障事件
    D <16 hex 位>         debug_state 快照（变化 + 1 Hz 心跳）

debug_state 位域（system_motion.v）：
    [63:56] start_diag   [55:48] route_reason  [47:40] menu_command
    [39:32] ack          [31:24] aux_type      [23:16] aux_code
    [15:13] tof_reused   [12:8]  route_state
    [7] voice_enable [6] modules_ready [5] tof_all_valid [4] fault
    [3] busy [2] waiting [1] auto_run [0] avoid_mode

用法::

    python3 decode_debug_log_tolerant.py <log> [--changes-only] [--json out.json]
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from collections import Counter

TS_RE = re.compile(r"^\[\s*(\d{1,2}:\d{2}:\d{2}(?:\.\d+)?)\s*\]\s*")
D_RE = re.compile(r"^D\s+([0-9A-Fa-f]{16})\s*$")
S_RE = re.compile(r"^S\s+(\S+)\s+(\S+)\s*$")
E_RE = re.compile(r"^E\s+(\S+)\s+(\S+)\s*$")

# route_state 是 5 bit，取值含义随固件版本变化（主工程解码脚本也只打数字），
# 这里保持数字，便于与 tools/decode_debug_log.ps1 的输出逐字段对照。
FLAGS = [(7, "voice_enable"), (6, "modules_ready"), (5, "tof_all_valid"),
         (4, "fault"), (3, "busy"), (2, "waiting"), (1, "auto_run"), (0, "avoid_mode")]


def parse_state(v: int) -> dict:
    return {
        "start_diag": (v >> 56) & 0xFF,
        "route_reason": (v >> 48) & 0xFF,
        "menu_command": (v >> 40) & 0xFF,
        "ack": (v >> 32) & 0xFF,
        "aux_type": (v >> 24) & 0xFF,
        "aux_code": (v >> 16) & 0xFF,
        "tof_reused": (v >> 13) & 0x07,
        "route_state": (v >> 8) & 0x1F,
        "flags": {name: bool((v >> bit) & 1) for bit, name in FLAGS},
    }


def decode_line(raw: str):
    ts = None
    m = TS_RE.match(raw)
    if m:
        ts = m.group(1)
        raw = raw[m.end():]
    line = raw.strip()
    if not line:
        return None
    m = D_RE.match(line)
    if m:
        return ("D", ts, parse_state(int(m.group(1), 16)))
    m = S_RE.match(line)
    if m:
        return ("S", ts, {"source": m.group(1), "code": m.group(2)})
    m = E_RE.match(line)
    if m:
        return ("E", ts, {"source": m.group(1), "code": m.group(2)})
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description="tolerant debug-log decoder")
    ap.add_argument("log")
    ap.add_argument("--changes-only", action="store_true")
    ap.add_argument("--json", default=None)
    ap.add_argument("--limit", type=int, default=0, help="只处理前 N 行（0=全部）")
    args = ap.parse_args()

    snapshots, starts, errors = [], [], []
    unparsed = 0
    for i, raw in enumerate(open(args.log, "r", encoding="utf-8", errors="replace")):
        if args.limit and i >= args.limit:
            break
        item = decode_line(raw.rstrip("\n"))
        if item is None:
            if raw.strip():
                unparsed += 1
            continue
        kind, ts, data = item
        if kind == "D":
            snapshots.append((ts, data))
        elif kind == "S":
            starts.append((ts, data))
        else:
            errors.append((ts, data))

    print("== decode summary: %s ==" % args.log)
    print("D(snapshots)=%d  S(start)=%d  E(errors)=%d  unparsed=%d"
          % (len(snapshots), len(starts), len(errors), unparsed))

    if starts:
        print("\n启动/状态事件:")
        for ts, d in starts[:20]:
            print("  %s S %s %s" % (ts or "", d["source"], d["code"]))
    if errors:
        print("\n错误事件:")
        for ts, d in errors[:20]:
            print("  %s E %s %s" % (ts or "", d["source"], d["code"]))

    if not snapshots:
        print("\n没有 D 快照 —— 检查抓包是否为二进制/乱码，或位流是否输出了调试日志")
        return 1

    hist = Counter(s["route_state"] for _, s in snapshots)
    print("\nroute_state 分布:", dict(hist))
    flag_hist = Counter()
    for _, s in snapshots:
        for k, v in s["flags"].items():
            if v:
                flag_hist[k] += 1
    print("置位计数:", dict(flag_hist))

    first_ts, last_ts = snapshots[0][0], snapshots[-1][0]
    print("首帧时间 %s / 末帧时间 %s（共 %d 帧）" % (first_ts, last_ts, len(snapshots)))

    print("\n变化记录（同值心跳已折叠）:")
    prev = None
    shown = 0
    for ts, s in snapshots:
        key = (s["route_state"], s["route_reason"], s["menu_command"], s["ack"],
               s["aux_type"], s["aux_code"], s["tof_reused"], tuple(s["flags"].items()))
        if prev is not None and key == prev:
            continue
        prev = key
        if shown < 40 or not args.changes_only:
            flags = ",".join(k for k, v in s["flags"].items() if v) or "-"
            print("  %s state=%-2d reason=0x%02X menu=0x%02X ack=0x%02X aux=%02X/%02X "
                  "reused=%d diag=%d [%s]"
                  % (ts or "-", s["route_state"], s["route_reason"], s["menu_command"],
                     s["ack"], s["aux_type"], s["aux_code"], s["tof_reused"],
                     s["start_diag"], flags))
        shown += 1
    print("  ... 共 %d 次变化" % shown)

    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump({"log": args.log, "snapshots": len(snapshots),
                       "starts": starts, "errors": errors,
                       "route_state_histogram": dict(hist),
                       "flag_counts": dict(flag_hist),
                       "first_ts": first_ts, "last_ts": last_ts},
                      f, indent=2, ensure_ascii=False)
        print("-> %s" % args.json)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""把串口助手（Fluent Serial Assistant 等）抓到的十六进制日志解码成可读帧。

典型用途：用户用图形串口工具抓了一段 hex，想快速看出里面有哪些帧、有没有 CRC 错、
遥测速率多少、故障位是什么。

支持这些输入形态（混在一起也行）::

    0A 55 81 17 ...           纯 hex，空格/换行/逗号/0x 前缀都可以
    RX AA 55 81 17 ...        行首带 RX/TX 标签（自动忽略标签）
    12:34:56.789 AA 55 ...    行首带时间戳（会用于计算帧间隔）
    # 注释行                 以 # 开头的行忽略

用法::

    python decode_hex.py capture.txt
    python decode_hex.py --json out.json capture.txt
    type capture.txt | python decode_hex.py
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import protocol as P  # noqa: E402

HEX_TOKEN = re.compile(r"^(?:0[xX])?([0-9a-fA-F]{2})$")
TS_LINE = re.compile(r"^\s*(\d{1,2}:\d{2}:\d{2}(?:[.,]\d{1,6})?)\s+(.*)$")


def parse_text(text: str):
    """返回 (bytes, 每字节时间戳列表或 None, 行号列表)。"""
    data = bytearray()
    stamps = []
    has_ts = False
    for lineno, raw in enumerate(text.splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith("#") or line.startswith("//"):
            continue
        ts = None
        m = TS_LINE.match(line)
        if m:
            ts = m.group(1)
            line = m.group(2)
        # drop common direction labels
        line = re.sub(r"^(\[?\s*(RX|TX|接收|发送)\s*\]?[:：]?)\s*", "", line,
                      flags=re.IGNORECASE)
        for token in re.split(r"[\s,;]+", line):
            if not token:
                continue
            hm = HEX_TOKEN.match(token)
            if not hm:
                continue          # skip unknown tokens instead of failing
            data.append(int(hm.group(1), 16))
            stamps.append(ts)
            has_ts = has_ts or (ts is not None)
    return bytes(data), (stamps if has_ts else None)


def ts_seconds(ts: str):
    """把 hh:mm:ss.mmm 转成秒（相对第一帧的偏移由调用方算）。"""
    if not ts:
        return None
    ts = ts.replace(",", ".")
    parts = ts.split(":")
    try:
        h, m, s = int(parts[0]), int(parts[1]), float(parts[2])
        return h * 3600 + m * 60 + s
    except (ValueError, IndexError):
        return None


def main() -> int:
    ap = argparse.ArgumentParser(description="decode hex serial capture into link v1 frames")
    ap.add_argument("file", nargs="?", help="capture file (default: stdin)")
    ap.add_argument("--json", default=None, help="write decoded frames to JSON")
    ap.add_argument("--quiet", action="store_true", help="only print the summary")
    args = ap.parse_args()

    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:  # noqa: BLE001
        pass

    if args.file:
        text = open(args.file, "r", encoding="utf-8", errors="replace").read()
        name = args.file
    else:
        text = sys.stdin.read()
        name = "<stdin>"

    raw, stamps = parse_text(text)
    print("输入: %s  解析出 %d 字节" % (name, len(raw)))
    if not raw:
        return 1

    parser = P.FrameParser()
    frames = parser.feed(raw)

    # 时间戳：帧起点对应的字节下标 -> 该帧时间
    starts = []
    pos = 0
    # FrameParser 不返回偏移，这里用 raw 里找帧头的方式近似
    idx = 0
    while True:
        idx = raw.find(b"\xaa\x55", idx)
        if idx < 0:
            break
        starts.append(idx)
        idx += 2

    def stamp_of(frame_index: int):
        if not stamps or frame_index >= len(starts):
            return None
        return ts_seconds(stamps[starts[frame_index]]) if stamps[starts[frame_index]] else None

    print("解析出 %d 帧" % len(frames))
    print("统计:", parser.stats.as_dict())

    telem = [f for f in frames if f.msg_type == P.MSG_TELEM]
    acks = [f for f in frames if f.msg_type == P.MSG_ACK]
    down = [f for f in frames if f.msg_type in (P.MSG_SET_VEL, P.MSG_STOP,
                                                P.MSG_HEARTBEAT, P.MSG_SET_LIMIT,
                                                P.MSG_CLEAR_FAULT)]
    print("分类: TELEM=%d ACK=%d 下行命令=%d" % (len(telem), len(acks), len(down)))

    if len(telem) >= 2:
        t0 = stamp_of(frames.index(telem[0]))
        t1 = stamp_of(frames.index(telem[-1]))
        if t0 is not None and t1 is not None and t1 > t0:
            print("遥测速率: %.2f Hz（按时间戳）" % ((len(telem) - 1) / (t1 - t0)))
        else:
            print("遥测速率: 无时间戳，无法计算（可用 --json 导出后自行分析）")

    if telem:
        last = P.decode_telem(telem[-1].payload)
        print("末帧遥测:", P.describe(telem[-1]))

    if not args.quiet:
        print("\n--- 帧列表 ---")
        for i, f in enumerate(frames[:200]):
            print("%5d  %s" % (i, P.describe(f)))
        if len(frames) > 200:
            print("... 共 %d 帧，只显示前 200" % len(frames))

    if args.json:
        out = {
            "input": name,
            "bytes": len(raw),
            "stats": parser.stats.as_dict(),
            "frames": [
                {"index": i, "type": f.msg_type, "name": f.name,
                 "len": len(f.payload), "payload_hex": f.payload.hex(" "),
                 "ts": stamp_of(i),
                 "text": P.describe(f)}
                for i, f in enumerate(frames)
            ],
        }
        with open(args.json, "w", encoding="utf-8") as fh:
            json.dump(out, fh, indent=2, ensure_ascii=False)
        print("JSON ->", args.json)

    return 0 if parser.stats.good == len(frames) else 2


if __name__ == "__main__":
    raise SystemExit(main())

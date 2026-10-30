#!/usr/bin/env python3
"""任务四 段记录串口采集（支持 PC 的 COM 口与 Linux 的 tty）。

FPGA 只在段结束/故障时发一条 54 字节记录，所以采集要"长时间挂着、来一条打一条"。

用法::

    # 自动找板子的串口（CH9102 对应 V8 板载 USB 串口；无线 WCH-Link 一般也是同款）
    python3 t4_capture.py --seconds 90 --out floor.log

    # 指定端口
    python3 t4_capture.py --port COM24 --seconds 90 --out floor.log
    python3 t4_capture.py --port /dev/ttyACM0 --seconds 90 --out floor.log

    # 也可以后台跑（上板测试时推荐）：
    python3 t4_capture.py --seconds 600 --out floor.log --quiet &

采集同时会把解析出的记录实时打印（带时间戳），并写 JSONL：
    floor.log  = 原始字节（判读用）
    floor.jsonl= 解析后的记录
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve()
sys.path.insert(0, str(HERE.parent))

import t4_log as L  # noqa: E402

BAUD = 115200

CANDIDATES = [
    # Linux：CH9102 / CH340 / FTDI / CDC-ACM
    "/dev/serial/by-id/usb-1a86_USB_Single_Serial_*",
    "/dev/serial/by-id/usb-*CH9102*",
    "/dev/serial/by-id/usb-*CH340*",
    "/dev/serial/by-id/usb-*FTDI*",
    "/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1",
    # Windows
    "COM24", "COM14", "COM15",
]


def find_port() -> str | None:
    import glob
    for pat in CANDIDATES:
        if any(ch in pat for ch in "*?["):
            hits = sorted(glob.glob(pat))
            if hits:
                return hits[0]
        else:
            try:
                import serial
                # 只探测存在性：Windows 上 COMx 存在即可，Linux 上检查设备文件
                if pat.startswith("/dev/") and not Path(pat).exists():
                    continue
                return pat
            except ImportError:
                if pat.startswith("/dev/") and Path(pat).exists():
                    return pat
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description="capture task4 segment UART log")
    ap.add_argument("--port", default=None, help="串口名；默认自动探测")
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--seconds", type=float, default=90.0)
    ap.add_argument("--out", default="floor.log")
    ap.add_argument("--jsonl", default=None, help="默认 <out>.jsonl")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    try:
        import serial
    except ImportError:
        print("需要 pyserial：pip install pyserial（Pi 的相机 venv 里已有）", file=sys.stderr)
        return 1

    port = args.port or find_port()
    if not port:
        print("找不到串口，请用 --port 指定（Windows 例：COM24；Pi 例：/dev/ttyACM0）",
              file=sys.stderr)
        return 1

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    jsonl = Path(args.jsonl) if args.jsonl else out.with_suffix(".jsonl")

    parser = L.SegmentLogParser()
    t0 = time.time()
    n_raw = 0
    n_rec = 0
    print("[capture] %s @%d，%0.0f s -> %s" % (port, args.baud, args.seconds, out),
          file=sys.stderr)
    with serial.Serial(port, args.baud, timeout=0.1) as sp, \
            open(out, "wb") as fraw, open(jsonl, "w", encoding="utf-8") as fj:
        while time.time() - t0 < args.seconds:
            data = sp.read(256)
            if not data:
                continue
            n_raw += len(data)
            fraw.write(data)
            fraw.flush()
            recs = parser.feed(data, ts=time.time())
            for r in recs:
                n_rec += 1
                fj.write(json.dumps(r.to_dict(), ensure_ascii=False) + "\n")
                fj.flush()
                print("[%s] %s" % (time.strftime("%H:%M:%S"), r), file=sys.stderr)

    print("[capture] 原始 %d 字节，记录 %d 条 -> %s / %s"
          % (n_raw, n_rec, out, jsonl), file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

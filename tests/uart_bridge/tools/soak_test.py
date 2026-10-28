#!/usr/bin/env python3
"""链路长稳测试（对应 docs/PROTOCOL_v1.md 第 8 节 T15）。

用法::

    python soak_test.py --port COM14 --duration 600 --out records/soak.json

做三件事：
  1. 按 10 Hz 恒发心跳，维持链路在线；
  2. 统计整个过程的帧数、CRC/同步/尾字节/长度错误（误码率）；
  3. 每 10 s 打一个窗口，记录遥测速率、故障位、心跳年龄，便于发现间歇性问题。

误码率定义：错误帧 / 总帧。目标 < 1e-6（115200、几十秒内应为 0）。
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import protocol as P          # noqa: E402
import transport as T         # noqa: E402
from uart_link import Link    # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="link soak test")
    ap.add_argument("--port", default="COM14")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--transport", default="uart", choices=["uart", "udp", "file"])
    ap.add_argument("--duration", type=float, default=600.0, help="seconds")
    ap.add_argument("--bucket", type=float, default=10.0, help="report period")
    ap.add_argument("--heartbeat-hz", type=float, default=10.0)
    ap.add_argument("--out", default=None)
    ap.add_argument("--window", type=int, default=50, help="report every N windows")
    args = ap.parse_args()

    if args.transport == "uart":
        tp = T.UartTransport(port=args.port, baud=args.baud)
    elif args.transport == "udp":
        tp = T.UdpTransport(remote=("192.168.1.50", 50000), local=("0.0.0.0", 50001))
    else:
        raise SystemExit("soak test needs a live transport")

    link = Link(tp, heartbeat_hz=args.heartbeat_hz)
    link.start()
    t0 = time.time()
    buckets = []
    prev_frames = 0
    prev_errs = 0
    prev_bytes = 0
    print("soak %s  duration=%.0fs  heartbeat=%.1fHz" % (tp.name, args.duration,
                                                        args.heartbeat_hz))
    try:
        # 先建立链路（发心跳 + clear fault 以便看到正常故障位）
        time.sleep(0.5)
        link.heartbeat(1)
        time.sleep(0.2)
        link.clear_fault()
        time.sleep(0.3)
        next_report = t0 + args.bucket
        while time.time() - t0 < args.duration:
            time.sleep(0.1)
            now = time.time()
            if now < next_report:
                continue
            next_report += args.bucket
            st = link.parser.stats
            total = st.good + st.crc_err + st.tail_err + st.len_err + st.sync_err + st.timeout
            errs = st.crc_err + st.tail_err + st.len_err
            tele = link.telemetry[-1] if link.telemetry else None
            rec = {
                "t_s": round(now - t0, 1),
                "frames_total": total,
                "d_frames": total - prev_frames,
                "d_errs": errs - prev_errs,
                "d_bytes": link.rx_bytes - prev_bytes,
                "telem_rate_hz": round(link.telem_rate(1.0), 2),
                "fault": tele.fault if tele else -1,
                "hb_age_ms": tele.hb_age_ms if tele else -1,
                "link_seq": tele.link_seq if tele else -1,
                "cmd_seq": tele.cmd_seq if tele else -1,
            }
            prev_frames, prev_errs, prev_bytes = total, errs, link.rx_bytes
            buckets.append(rec)
            if len(buckets) % args.window == 0 or len(buckets) <= 3:
                print("[%6.0fs] frames=%-6d (+%d) errs=%-3d (+%d) rate=%.1fHz "
                      "fault=0x%02X hb_age=%dms"
                      % (rec["t_s"], rec["frames_total"], rec["d_frames"],
                         errs, rec["d_errs"], rec["telem_rate_hz"],
                         rec["fault"], rec["hb_age_ms"]))
    except KeyboardInterrupt:
        print("interrupted")
    finally:
        link.stop()

    st = link.parser.stats
    total = max(1, st.good + st.crc_err + st.tail_err + st.len_err)
    errs = st.crc_err + st.tail_err + st.len_err
    rates = [b["telem_rate_hz"] for b in buckets if b["telem_rate_hz"] > 0]
    summary = {
        "port": args.port, "duration_s": round(time.time() - t0, 1),
        "parser_stats": st.as_dict(),
        "frames_total": total,
        "error_frames": errs,
        "error_rate": errs / total,
        "telem_rate_mean": round(sum(rates) / len(rates), 2) if rates else 0.0,
        "telem_rate_min": round(min(rates), 2) if rates else 0.0,
        "telem_rate_max": round(max(rates), 2) if rates else 0.0,
        "buckets": buckets,
        "ok": errs == 0,
    }
    print("\n== soak summary ==")
    print(json.dumps({k: v for k, v in summary.items() if k != "buckets"},
                     indent=2, ensure_ascii=False))
    if args.out:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2, ensure_ascii=False)
        print("report ->", args.out)
    return 0 if summary["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

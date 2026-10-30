#!/usr/bin/env python3
"""架空车轮运动长稳：多组工况循环，检查长时间运行下 ACK/遥测/命令链路是否稳定。

  90 s 循环：前进 150 → 后退 150 → 原地转 → 停 2 s
  用法：python3 bench_motion_soak.py --duration 180 --json soak.json
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from collections import Counter
from pathlib import Path

HERE = Path(__file__).resolve()


def _find_uart_tools() -> Path:
    env = os.environ.get("UART_BRIDGE_TOOLS")
    if env:
        return Path(env)
    for c in [HERE.parents[2] / "uart_bridge" / "tools" if len(HERE.parents) > 2 else None,
              HERE.parent / "uart_tools", Path.home() / "uart_tools",
              Path.cwd() / "uart_tools"]:
        if c and (c / "protocol.py").is_file():
            return c
    return Path.home() / "uart_tools"


sys.path.insert(0, str(_find_uart_tools()))
import protocol as P          # noqa: E402
import transport as T         # noqa: E402
from uart_link import Link    # noqa: E402

CMD_HZ = 20.0


def cpu_times():
    with open("/proc/stat", "r", encoding="utf-8") as f:
        v = [int(x) for x in f.readline().split()[1:]]
    idle = v[3] + (v[4] if len(v) > 4 else 0)
    return sum(v), idle


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=180.0)
    ap.add_argument("--json", default=None)
    ap.add_argument("--host", default="192.168.0.2")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--local-port", type=int, default=5001)
    args = ap.parse_args()

    tp = T.UdpTransport(remote=(args.host, args.port), local=("0.0.0.0", args.local_port))
    link = Link(tp, heartbeat_hz=20.0)
    acks: list[dict] = []
    telem: list[dict] = []

    def on_frame(fr):
        now = time.time()
        if fr.msg_type == P.MSG_ACK:
            try:
                a = P.decode_ack(fr.payload)
            except ValueError:
                return
            acks.append({"t": now, "status": a.status_name, "type": a.acked_type})
        elif fr.msg_type == P.MSG_TELEM:
            try:
                t = P.decode_telem(fr.payload)
            except ValueError:
                return
            telem.append({"t": now, "fault": t.fault})

    link.on_frame = on_frame
    link.start()
    time.sleep(1.0)
    link.clear_fault()
    link.wait_ack(1.0, P.MSG_CLEAR_FAULT)

    seq = 0

    def send(l, r):
        nonlocal seq
        seq = (seq % 200) + 1
        link.set_vel(l, r, seq)

    phases = [("forward_150", 150, 150, 12.0), ("backward_150", -150, -150, 12.0),
              ("rotate", -120, 120, 10.0), ("pause", 0, 0, 4.0)]
    t0 = time.time()
    c0 = cpu_times()
    cpus: list[float] = []
    cycles = 0
    next_cpu = t0 + 5.0
    try:
        while time.time() - t0 < args.duration:
            for name, l, r, secs in phases:
                if time.time() - t0 >= args.duration:
                    break
                print("[%s] %-12s L=%+4d R=%+4d %.0fs" %
                      (time.strftime("%H:%M:%S"), name, l, r, secs))
                end = time.time() + secs
                n = int(0.4 * CMD_HZ)
                k = 1
                # 简单斜坡
                while k <= n and time.time() < end:
                    send(int(l * k / n), int(r * k / n))
                    time.sleep(1.0 / CMD_HZ)
                    k += 1
                while time.time() < end:
                    send(l, r)
                    time.sleep(1.0 / CMD_HZ)
                for k in range(n - 1, -1, -1):
                    send(int(l * k / n), int(r * k / n))
                    time.sleep(1.0 / CMD_HZ)
            cycles += 1
            if time.time() >= next_cpu:
                c1 = cpu_times()
                if c1[0] > c0[0]:
                    cpus.append(round((1.0 - (c1[1] - c0[1]) / (c1[0] - c0[0])) * 100, 1))
                c0 = c1
                next_cpu = time.time() + 10.0
        link.stop_cmd()
        time.sleep(0.15)
        link.clear_fault()
    finally:
        link.stop()

    dur = time.time() - t0
    st = Counter(a["status"] for a in acks)
    fh = Counter(t["fault"] for t in telem)
    ts = [t["t"] for t in telem]
    gaps = sorted([(b - a) * 1000.0 for a, b in zip(ts, ts[1:])])
    res = {
        "duration_s": round(dur, 1), "cycles": cycles,
        "acks": dict(st), "n_acks": len(acks),
        "ack_ok_ratio": round(st.get("OK", 0) / max(1, len(acks)), 4),
        "telemetry": {"frames": len(telem),
                      "rate_hz": round(len(telem) / max(1e-9, dur), 2),
                      "gap_max_ms": round(gaps[-1], 1) if gaps else None,
                      "fault_hist": {("0x%02X" % k): v for k, v in fh.items()}},
        "cpu_percent": {"mean": round(sum(cpus) / len(cpus), 1) if cpus else None,
                        "max": max(cpus) if cpus else None},
    }
    # 排除起步建链阶段（前 2 s）的故障样本再判定
    warm = Counter(x["fault"] for x in telem if x["t"] >= t0 + 2.0)
    bad = [k for k in warm if k & ~(P.FAULT_CMD_TIMEOUT | P.FAULT_STOP_LATCHED)]
    res["ok"] = bool(res["ack_ok_ratio"] >= 0.999 and not bad and
                     49.0 <= res["telemetry"]["rate_hz"] <= 52.0)
    print(json.dumps(res, indent=2, ensure_ascii=False))
    print("\nMOTION SOAK %s" % ("PASS" if res["ok"] else "FAIL"))
    if args.json:
        Path(args.json).parent.mkdir(parents=True, exist_ok=True)
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(res, f, indent=2, ensure_ascii=False)
        print("-> %s" % args.json)
    return 0 if res["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

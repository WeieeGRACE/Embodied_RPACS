#!/usr/bin/env python3
"""链路 v2（以太网/UDP）长稳测试。

持续 50 Hz 遥测 + 10 Hz 心跳 + 周期命令（SET_VEL 递增 / 每 60 s STOP+CLEAR），
统计速率、帧间隔、解析错误、ACK 状态、故障位分布与 Pi CPU。

用法::

    python3 eth_soak.py --duration 600 --json soak.json --csv soak.csv
    python3 eth_soak.py --duration 60 --json quick.json
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
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


UART_TOOLS = _find_uart_tools()
if str(UART_TOOLS) not in sys.path:
    sys.path.insert(0, str(UART_TOOLS))

import protocol as P          # noqa: E402
import transport as T         # noqa: E402
from uart_link import Link    # noqa: E402


def cpu_times() -> tuple[int, int]:
    with open("/proc/stat", "r", encoding="utf-8") as f:
        parts = f.readline().split()[1:]
    vals = [int(x) for x in parts]
    idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
    return sum(vals), idle


def main() -> int:
    ap = argparse.ArgumentParser(description="link v2 UDP soak")
    ap.add_argument("--host", default="192.168.0.2")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--local-port", type=int, default=5001)
    ap.add_argument("--duration", type=float, default=600.0)
    ap.add_argument("--json", default=None)
    ap.add_argument("--csv", default=None)
    ap.add_argument("--telem-period-ms", type=float, default=20.0)
    args = ap.parse_args()

    tp = T.UdpTransport(remote=(args.host, args.port),
                        local=("0.0.0.0", args.local_port))
    link = Link(tp, heartbeat_hz=10.0)
    link.start()

    # 记录所有遥测的时间戳与关键字段
    samples: list[dict] = []
    stop_evt = __import__("threading").Event()

    def collector() -> None:
        # Link.telemetry 是 maxlen=2000 的 deque：必须 drain，否则会重复/丢帧
        while not stop_evt.is_set():
            time.sleep(0.005)
            while link.telemetry:
                f = link.telemetry.popleft()
                samples.append({"t": time.time(), "cmd_seq": f.cmd_seq,
                                "fault": f.fault, "hb_age": f.hb_age_ms,
                                "enc": (f.enc_l, f.enc_r)})

    import threading
    th = threading.Thread(target=collector, daemon=True)
    th.start()

    # ---- 起步：等建链（心跳 300 ms 超时），然后清掉 LINK_LOST 锁存
    time.sleep(1.0)
    link.clear_fault()
    link.wait_ack(1.0, P.MSG_CLEAR_FAULT)
    time.sleep(0.1)

    t_start = time.time()
    cpu0 = cpu_times()
    cpu_samples: list[float] = []
    acks: dict[int, dict] = {}
    cmd_seq = 0
    n_acks_seen = 0
    next_cmd = t_start + 2.0
    next_cycle = t_start + 60.0
    csv_f = open(args.csv, "w", encoding="utf-8") if args.csv else None
    if csv_f:
        csv_f.write("t_s,frames,cum_rate_hz,rate_win,fault,hb_age,cmd_seq,sent_acks\n")

    last_win_t = t_start
    last_win_n = 0
    last_stats = link.parser.stats.as_dict()

    auto_clear_next = t_start + 5.0
    try:
        while time.time() - t_start < args.duration:
            now = time.time()
            # 周期命令：SET_VEL（递增 seq），值在限幅内
            if now >= next_cmd:
                cmd_seq = (cmd_seq % 200) + 1
                v = 80 + (cmd_seq % 20) * 5          # 80..175 mm/s，远低于限幅 300
                link.set_vel(v, v, cmd_seq)
                next_cmd = now + 0.2
            # 链路锁存了 LINK_LOST 时自动清除（真实上位机应当这么做）
            if now >= auto_clear_next:
                # 无条件周期性清锁存：真实上位机也应该这么做（link 未通时会被拒绝，无害）
                link.clear_fault()
                auto_clear_next = now + 5.0
            # 每 60 s：STOP + CLEAR_FAULT（验证故障位往返）
            if now >= next_cycle:
                link.stop_cmd()
                time.sleep(0.15)
                link.clear_fault()
                next_cycle = now + 60.0
            # ACK 统计（只处理新增的，避免重复计数）
            new_acks = list(link.acks)[n_acks_seen:]
            n_acks_seen = len(link.acks)
            for a in new_acks:
                d = acks.setdefault(a.acked_type, {"ok": 0, "other": {}, "n": 0})
                d["n"] += 1
                if a.status == P.ACK_OK:
                    d["ok"] += 1
                else:
                    d["other"][a.status_name] = d["other"].get(a.status_name, 0) + 1
            # CPU 采样（每 5 s）
            if int(now - t_start) % 5 == 0:
                c1 = cpu_times()
                if c1[0] > cpu0[0]:
                    busy = 1.0 - (c1[1] - cpu0[1]) / (c1[0] - cpu0[0])
                    cpu_samples.append(round(busy * 100.0, 1))
            # 统计窗口（5 s）
            if now - last_win_t >= 5.0:
                n = len(samples)
                win_rate = (n - last_win_n) / (now - last_win_t)
                cum_rate = (n / (now - t_start)) if now > t_start else 0.0
                last = samples[-1] if samples else None
                if csv_f and last:
                    csv_f.write("%.1f,%d,%.2f,%.2f,0x%02X,%d,%d,%d\n"
                                % (now - t_start, n, cum_rate, win_rate,
                                   last["fault"], last["hb_age"],
                                   last["cmd_seq"], sum(d["n"] for d in acks.values())))
                last_win_t, last_win_n = now, n
            time.sleep(0.01)
    finally:
        stop_evt.set()
        th.join(timeout=2.0)
        link.stop()
        if csv_f:
            csv_f.close()

    # ---------------- 汇总
    # 只统计测量窗口内的样本（排除起步建链/CLEAR 的预热帧）
    samples = [s for s in samples if s["t"] >= t_start]
    total = len(samples)
    dur = max(1e-9, time.time() - t_start)
    ts = [s["t"] for s in samples]
    gaps = [(b - a) * 1000.0 for a, b in zip(ts, ts[1:])]
    gaps.sort()
    expected = dur * (1000.0 / args.telem_period_ms)
    cur_stats = link.parser.stats.as_dict()
    errs = {k: cur_stats[k] for k in ("crc_err", "tail_err", "len_err", "sync_err")}
    faults: dict[str, int] = {}
    hb_ages = [s["hb_age"] for s in samples]
    cmd_seqs = [s["cmd_seq"] for s in samples]

    res = {
        "duration_s": round(dur, 1),
        "frames": total,
        "expected_frames": int(expected),
        "loss_percent": round(max(0.0, (expected - total) / expected * 100.0), 3),
        "rate_hz": round(total / dur, 3),
        "gap_ms": {"max": round(gaps[-1], 1) if gaps else None,
                   "p99": round(gaps[int(len(gaps) * 0.99)] if gaps else 0.0, 1),
                   "p50": round(gaps[len(gaps) // 2] if gaps else 0.0, 1)},
        "parse_errors": errs,
        "hb_age_ms": {"min": min(hb_ages) if hb_ages else None,
                      "max": max(hb_ages) if hb_ages else None},
        "cmd_seq_seen": {"first": cmd_seqs[0] if cmd_seqs else None,
                         "last": cmd_seqs[-1] if cmd_seqs else None,
                         "distinct": len(set(cmd_seqs))},
        "fault_histogram": {("0x%02X" % k): v for k, v in
                            sorted({f: [s["fault"] for s in samples].count(f)
                                    for f in set(s["fault"] for s in samples)}.items())},
        "acks": {str(k): {"n": v["n"], "ok": v["ok"], "other": v["other"]}
                 for k, v in acks.items()},
        "cpu_percent": {"mean": round(sum(cpu_samples) / len(cpu_samples), 1)
                        if cpu_samples else None,
                        "max": max(cpu_samples) if cpu_samples else None},
        "parser_total": cur_stats,
    }
    # 判定
    healthy = sum(1 for s in samples if s["fault"] == 0)
    res["healthy_fraction"] = round(healthy / total, 4) if total else 0.0
    ok = (49.0 <= res["rate_hz"] <= 51.5
          and res["loss_percent"] <= 0.5
          and (errs["crc_err"] + errs["tail_err"] + errs["len_err"]) == 0
          and errs["sync_err"] <= 2
          and (res["gap_ms"]["max"] or 0) <= 150.0
          and res["healthy_fraction"] >= 0.90)
    res["ok"] = bool(ok)

    print(json.dumps(res, indent=2, ensure_ascii=False))
    print("\nSOAK %s (%.0f s, %d frames, %.2f Hz)"
          % ("PASS" if ok else "FAIL", dur, total, res["rate_hz"]))
    if args.json:
        Path(args.json).parent.mkdir(parents=True, exist_ok=True)
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(res, f, indent=2, ensure_ascii=False)
        print("-> %s" % args.json)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())

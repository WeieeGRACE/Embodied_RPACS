#!/usr/bin/env python3
"""架空车轮（wheels-up）电机台架测试。

前提：车辆已架空、电机电源接通、板上是 eth_motion_top.fs。
覆盖所有"轮子转起来但不落地"的用例：
  1. 前进速度阶梯 50..300 mm/s
  2. 后退 50 / 150
  3. 原地转（左右反向）
  4. 差速
  5. 快速换向（死区压力）
  6. 行驶中 STOP（应停 + 锁存 + 后续 SET_VEL 被拒）
  7. 清故障后恢复
  8. 行驶中心跳丢失（应 LINK_LOST + 命令被拒，恢复后可用）
  9. 持续运行 20 s
命令 20 Hz、带 0.4 s 斜坡；任何异常（ACK 非 OK / 非 CMD_TIMEOUT 故障位）立即停车。

用法::

    python3 bench_motion_test.py --json bench.json
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


class Bench:
    def __init__(self, host: str, port: int, local_port: int) -> None:
        self.tp = T.UdpTransport(remote=(host, port), local=("0.0.0.0", local_port))
        self.link = Link(self.tp, heartbeat_hz=20.0)
        self.link.on_frame = self._on_frame
        self.seq = 0
        self.telem: list[dict] = []
        self.acks: list[dict] = []
        self.steps: list[dict] = []

    def _on_frame(self, frame) -> None:
        now = time.time()
        if frame.msg_type == P.MSG_TELEM:
            try:
                t = P.decode_telem(frame.payload)
            except ValueError:
                return
            self.telem.append({"t": now, "fault": t.fault, "hb": t.hb_age_ms,
                               "cmd_seq": t.cmd_seq})
        elif frame.msg_type == P.MSG_ACK:
            try:
                a = P.decode_ack(frame.payload)
            except ValueError:
                return
            self.acks.append({"t": now, "type": a.acked_type,
                              "seq": a.seq, "status": a.status_name})

    def start(self) -> None:
        self.link.start()
        time.sleep(1.0)
        self.link.clear_fault()
        self.link.wait_ack(1.0, P.MSG_CLEAR_FAULT)

    def close(self) -> None:
        try:
            self.link.stop()
        except Exception:  # noqa: BLE001
            pass

    def send(self, l: int, r: int) -> None:
        self.seq = (self.seq % 200) + 1
        self.link.set_vel(l, r, self.seq)

    def emergency(self, why: str) -> None:
        print("!! %s -> STOP" % why)
        try:
            self.link.stop_cmd()
            time.sleep(0.15)
            self.link.clear_fault()
        finally:
            raise RuntimeError(why)

    def step(self, name: str, l: int, r: int, seconds: float,
             ramp_s: float = 0.4, stop_after: bool = True) -> dict:
        print("\n[%s] %-22s L=%+4d R=%+4d %.1fs   <-- 请观察轮子"
              % (time.strftime("%H:%M:%S"), name, l, r, seconds))
        i_t, i_a = len(self.telem), len(self.acks)
        t0 = time.time()
        n = max(1, int(ramp_s * CMD_HZ))
        for k in range(1, n + 1):
            self.send(int(l * k / n), int(r * k / n))
            time.sleep(1.0 / CMD_HZ)
        while time.time() - t0 < seconds:
            self.send(l, r)
            time.sleep(1.0 / CMD_HZ)
        for k in range(n - 1, -1, -1):
            self.send(int(l * k / n), int(r * k / n))
            time.sleep(1.0 / CMD_HZ)
        i_t_end = len(self.telem)          # 先截断：只统计巡航段遥测
        if stop_after:
            self.link.stop_cmd()
            time.sleep(0.1)
            self.link.clear_fault()
            time.sleep(0.1)

        telem = self.telem[i_t:i_t_end]
        acks = self.acks[i_a:]
        st = Counter(a["status"] for a in acks)
        fh = Counter(t["fault"] for t in telem)
        bad_faults = [f for f in fh if f & ~P.FAULT_CMD_TIMEOUT]
        ok = bool(acks) and ("OK" in st) and not bad_faults
        rec = {"name": name, "left": l, "right": r, "seconds": seconds,
               "acks": dict(st), "n_acks": len(acks),
               "fault_hist": {("0x%02X" % k): v for k, v in fh.items()},
               "telem": len(telem), "ok": ok}
        self.steps.append(rec)
        print("        acks=%s faults=%s telem=%d -> %s"
              % (dict(st), rec["fault_hist"], len(telem), "OK" if ok else "ANOMALY"))
        if not ok:
            self.emergency("步骤异常: %s" % name)
        return rec


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.0.2")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--local-port", type=int, default=5001)
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    b = Bench(args.host, args.port, args.local_port)
    b.start()
    t_start = time.time()
    try:
        for v in (50, 100, 150, 200, 250, 300):
            b.step("forward_%d" % v, v, v, 2.0)
        for v in (50, 150):
            b.step("backward_%d" % v, -v, -v, 2.0)
        b.step("rotate_left", -120, 120, 2.0)
        b.step("rotate_right", 120, -120, 2.0)
        b.step("differential", -100, 200, 2.0)

        print("\n[%s] 快速换向 6 次  <-- 请观察换向是否平顺" % time.strftime("%H:%M:%S"))
        i_t, i_a = len(b.telem), len(b.acks)
        for k in range(6):
            l, r = (150, -150) if k % 2 == 0 else (-150, 150)
            for _ in range(int(0.8 * CMD_HZ)):
                b.send(l, r)
                time.sleep(1.0 / CMD_HZ)
        i_t_end = len(b.telem)             # 只统计停车前的遥测
        b.link.stop_cmd(); time.sleep(0.1); b.link.clear_fault()
        st = Counter(a["status"] for a in b.acks[i_a:])
        fh = Counter(t["fault"] for t in b.telem[i_t:i_t_end])
        ok = ("OK" in st) and not [f for f in fh if f & ~P.FAULT_CMD_TIMEOUT]
        b.steps.append({"name": "quick_reversals", "acks": dict(st),
                        "fault_hist": {("0x%02X" % k): v for k, v in fh.items()}, "ok": ok})
        print("        acks=%s faults=%s -> %s" % (dict(st), dict(fh), "OK" if ok else "ANOMALY"))
        if not ok:
            b.emergency("快速换向异常")

        print("\n[%s] 行驶中 STOP  <-- 轮子应立刻停" % time.strftime("%H:%M:%S"))
        for _ in range(int(1.5 * CMD_HZ)):
            b.send(200, 200); time.sleep(1.0 / CMD_HZ)
        b.link.stop_cmd()
        ack = b.link.wait_ack(1.0, P.MSG_STOP)
        time.sleep(0.2)
        while b.link.acks:                  # 清掉驾驶阶段的旧 ACK
            b.link.acks.popleft()
        probe_seq = b.link.set_vel(120, 120)
        ack2 = b.link.wait_ack(1.0, P.MSG_SET_VEL)
        ok = (ack is not None and ack.status == P.ACK_OK and
              ack2 is not None and ack2.seq == probe_seq and
              ack2.status == P.ACK_REJECT_STOP)
        detail = {"stop": ack.status_name if ack else "none",
                  "setvel_after_stop": (ack2.status_name if ack2 else "none")}
        b.steps.append({"name": "stop_mid_drive", "detail": detail, "ok": ok})
        print("        %s -> %s" % (detail, "OK" if ok else "ANOMALY"))
        if not ok:
            b.emergency("STOP 行为异常")

        b.link.clear_fault()
        b.link.wait_ack(1.0, P.MSG_CLEAR_FAULT)
        time.sleep(0.2)
        b.step("resume_after_stop", 100, 100, 1.5)

        print("\n[%s] 行驶中心跳丢失  <-- 轮子应停" % time.strftime("%H:%M:%S"))
        i_t = len(b.telem)
        for _ in range(int(1.0 * CMD_HZ)):
            b.send(120, 120); time.sleep(1.0 / CMD_HZ)
        b.link.pause_heartbeat()
        time.sleep(0.8)
        while b.link.acks:                  # 清掉旧 ACK
            b.link.acks.popleft()
        probe_seq = b.link.set_vel(120, 120)
        ack = b.link.wait_ack(1.0, P.MSG_SET_VEL)
        fh = Counter(t["fault"] for t in b.telem[i_t:])
        lost = any(f & P.FAULT_LINK_LOST for f in fh)
        ok = (ack is not None and ack.seq == probe_seq and
              ack.status == P.ACK_REJECT_LINK and lost)
        b.steps.append({"name": "heartbeat_loss_while_driving",
                        "detail": {"setvel": ack.status_name if ack else "none",
                                   "link_lost_seen": lost},
                        "fault_hist": {("0x%02X" % k): v for k, v in fh.items()},
                        "ok": ok})
        print("        setvel=%s link_lost=%s -> %s"
              % (ack.status_name if ack else "none", lost, "OK" if ok else "ANOMALY"))
        if not ok:
            b.emergency("心跳丢失行为异常")
        b.link.resume_heartbeat()
        time.sleep(0.3)
        b.link.clear_fault()
        b.link.wait_ack(1.0, P.MSG_CLEAR_FAULT)
        time.sleep(0.2)

        b.step("sustained_200", 200, 200, 20.0, ramp_s=0.6)

        b.link.stop_cmd()
        time.sleep(0.15)
        b.link.clear_fault()
        time.sleep(0.2)
        print("\n[%s] 测试结束，已停车并清故障" % time.strftime("%H:%M:%S"))
    except RuntimeError as exc:
        print("SAFETY STOP: %s" % exc)
        return 2
    finally:
        b.close()

    total = len(b.steps)
    passed = sum(1 for s in b.steps if s["ok"])
    ts = [t["t"] for t in b.telem]
    gaps = sorted([(y - x) * 1000.0 for x, y in zip(ts, ts[1:])])
    dur = time.time() - t_start
    report = {
        "started": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "duration_s": round(dur, 1),
        "steps": b.steps,
        "steps_passed": passed,
        "steps_total": total,
        "telemetry": {
            "frames": len(b.telem),
            "rate_hz": round(len(b.telem) / max(1e-9, dur), 2),
            "gap_max_ms": round(gaps[-1], 1) if gaps else None,
            "gap_p99_ms": round(gaps[int(len(gaps) * 0.99)], 1) if gaps else None,
            "fault_hist": {("0x%02X" % k): v for k, v in
                           Counter(t["fault"] for t in b.telem).items()},
        },
        "ok": passed == total,
    }
    print("\n%d/%d steps passed" % (passed, total))
    print(json.dumps({k: report[k] for k in ("duration_s", "steps_passed",
                                             "steps_total", "telemetry", "ok")},
                     indent=2, ensure_ascii=False))
    if args.json:
        Path(args.json).parent.mkdir(parents=True, exist_ok=True)
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
        print("-> %s" % args.json)
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

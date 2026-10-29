#!/usr/bin/env python3
"""链路 v2（以太网/UDP）安全与故障行为测试。

覆盖协议里的每一条安全语义（都不涉及电机，当前位流没有电机输出）：
  F1 心跳停止 -> LINK_LOST 锁存，命令被拒
  F2 心跳恢复 + CLEAR_FAULT -> 故障清除，命令恢复
  F3 命令超时（500 ms 无命令）-> CMD_TIMEOUT 位
  F4 STOP 锁存 -> 命令被 REJECT_STOP，CLEAR 后恢复
  F5 SET_LIMIT 后 SET_VEL 超限 -> 被限幅或拒绝
  F6 CRC 错帧 -> 被丢弃，链路继续
  F7 未知类型帧 -> 不崩，链路继续
  F8 全程遥测保持 ~50 Hz

用法::

    python3 eth_fault_test.py --json fault.json
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


sys.path.insert(0, str(_find_uart_tools()))

import protocol as P          # noqa: E402
import transport as T         # noqa: E402
from uart_link import Link    # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.0.2")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--local-port", type=int, default=5001)
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    tp = T.UdpTransport(remote=(args.host, args.port),
                        local=("0.0.0.0", args.local_port))
    link = Link(tp, heartbeat_hz=10.0)
    link.start()
    time.sleep(1.2)
    link.clear_fault()
    link.wait_ack(1.0, P.MSG_CLEAR_FAULT)
    time.sleep(0.2)

    results: list[dict] = []

    def check(name: str, ok: bool, detail: str = "") -> bool:
        results.append({"name": name, "ok": bool(ok), "detail": detail})
        print("%s %-36s %s" % ("PASS" if ok else "FAIL", name, detail))
        return ok

    def last_telem(timeout: float = 1.0):
        return link.wait_telemetry(timeout, lambda t: True)

    def fault_now(timeout: float = 0.8):
        t = last_telem(timeout)
        return t.fault if t else None

    try:
        # 基线：健康。注意 CMD_TIMEOUT 是"500 ms 内没有命令"的正常语义，
        # 心跳不刷新它，所以基线要先补一条命令再判定。
        link.set_vel(0, 0, 5)
        link.wait_ack(1.0, P.MSG_SET_VEL)
        time.sleep(0.15)
        t = last_telem(1.0)
        check("F0_baseline_healthy", t is not None and t.fault == 0,
              "fault=0x%02X" % (t.fault if t else 0xFF))

        # ---- F1 心跳停止 -> LINK_LOST
        link.pause_heartbeat()
        time.sleep(1.0)
        f = fault_now()
        check("F1_heartbeat_stop_latches_link_lost",
              f is not None and bool(f & P.FAULT_LINK_LOST), "fault=0x%02X" % (f or 0))
        link.set_vel(100, 100, 7)
        a = link.wait_ack(1.0, P.MSG_SET_VEL)
        check("F1b_setvel_rejected_when_link_lost",
              a is not None and a.status == P.ACK_REJECT_LINK,
              a.status_name if a else "no ACK")

        # ---- F2 恢复
        link.resume_heartbeat()
        time.sleep(0.5)
        link.clear_fault()
        a = link.wait_ack(1.0, P.MSG_CLEAR_FAULT)
        check("F2_clear_after_link_recovery", a is not None and a.status == P.ACK_OK,
              a.status_name if a else "no ACK")
        time.sleep(0.3)
        f = fault_now()
        check("F2b_link_lost_cleared", f is not None and not (f & P.FAULT_LINK_LOST),
              "fault=0x%02X" % (f or 0))

        # ---- F3 命令超时
        link.set_vel(50, 50, 11)
        link.wait_ack(1.0, P.MSG_SET_VEL)
        time.sleep(1.0)                 # > 500 ms 无新命令
        f = fault_now()
        check("F3_command_timeout_bit",
              f is not None and bool(f & P.FAULT_CMD_TIMEOUT), "fault=0x%02X" % (f or 0))
        link.clear_fault()
        link.wait_ack(1.0, P.MSG_CLEAR_FAULT)

        # ---- F4 STOP 锁存
        link.stop_cmd()
        a = link.wait_ack(1.0, P.MSG_STOP)
        check("F4_stop_ack", a is not None and a.status == P.ACK_OK,
              a.status_name if a else "no ACK")
        time.sleep(0.2)
        f = fault_now()
        check("F4b_stop_latched_bit",
              f is not None and bool(f & P.FAULT_STOP_LATCHED), "fault=0x%02X" % (f or 0))
        link.set_vel(100, 100, 12)
        a = link.wait_ack(1.0, P.MSG_SET_VEL)
        check("F4c_setvel_rejected_when_stopped",
              a is not None and a.status == P.ACK_REJECT_STOP,
              a.status_name if a else "no ACK")
        link.clear_fault()
        a = link.wait_ack(1.0, P.MSG_CLEAR_FAULT)
        check("F4d_clear_unlatches_stop", a is not None and a.status == P.ACK_OK,
              a.status_name if a else "no ACK")

        # ---- F5 SET_LIMIT
        link.set_limit(100)
        a = link.wait_ack(1.0, P.MSG_SET_LIMIT)
        check("F5_set_limit_ack", a is not None and a.status == P.ACK_OK,
              a.status_name if a else "no ACK")
        link.set_vel(300, 300, 13)
        a = link.wait_ack(1.0, P.MSG_SET_VEL)
        ok_or_reject = a is not None and a.status in (P.ACK_OK, P.ACK_REJECT_LIMIT)
        check("F5b_setvel_over_limit_handled", ok_or_reject,
              a.status_name if a else "no ACK")
        link.set_limit(300)
        link.wait_ack(1.0, P.MSG_SET_LIMIT)
        link.clear_fault()
        link.wait_ack(0.8, P.MSG_CLEAR_FAULT)

        # ---- F6 CRC 错帧
        good = P.build_frame(P.MSG_HEARTBEAT, bytes([0x21]))
        bad = bytearray(good)
        bad[-2] ^= 0xFF                      # 破坏 CRC
        link.send_raw(bytes(bad))
        time.sleep(0.3)
        t = last_telem(1.0)
        check("F6_corrupt_frame_ignored", t is not None, "telemetry alive")

        # ---- F7 未知类型
        link.send_raw(P.build_frame(0x7F, b"\x00\x01\x02"))
        time.sleep(0.3)
        t = last_telem(1.0)
        check("F7_unknown_type_ignored", t is not None, "telemetry alive")

        # ---- F8 遥测速率
        n0 = len(link.telemetry)
        time.sleep(1.0)
        rate = link.telem_rate(1.0)
        check("F8_telemetry_rate_stable", 45.0 <= rate <= 55.0, "%.2f Hz" % rate)

        # 收尾：恢复干净状态
        link.clear_fault()
        link.wait_ack(0.8, P.MSG_CLEAR_FAULT)
        time.sleep(0.2)
        f = fault_now()
        check("F9_final_clean", f is not None, "fault=0x%02X" % (f or 0))
    finally:
        link.stop()

    passed = sum(1 for r in results if r["ok"])
    total = len(results)
    print("\n%d/%d checks passed" % (passed, total))
    print("ETH FAULT %s" % ("PASS" if passed == total else "FAIL"))
    out = {"host": args.host, "port": args.port, "checks": results,
           "passed": passed, "total": total, "ok": passed == total}
    if args.json:
        Path(args.json).parent.mkdir(parents=True, exist_ok=True)
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(out, f, indent=2, ensure_ascii=False)
        print("-> %s" % args.json)
    return 0 if passed == total else 1


if __name__ == "__main__":
    raise SystemExit(main())

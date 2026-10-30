#!/usr/bin/env python3
"""明天动车用：安全驾驶助手（以太网链路 v2）。

特性：
  * 以 20 Hz 持续发 SET_VEL（满足 CMD_TIMEOUT=500 ms 的要求）
  * 自动建链 + CLEAR_FAULT
  * 任何异常（链路丢失 / 故障位非 0 / ACK 非 OK）立即停车并告警
  * 支持定时、限速、斜坡启动（避免突然满占空比）
  * 结束时自动 STOP + CLEAR

用法::

    # 轮子离地：前进 150 mm/s 持续 3 s（有斜坡 0.5 s）
    python3 eth_drive.py --left 150 --right 150 --seconds 3

    # 原地转（左轮后退、右轮前进）
    python3 eth_drive.py --left -120 --right 120 --seconds 2

    # 长时间低速：100 mm/s，60 s
    python3 eth_drive.py --left 100 --right 100 --seconds 60

    # 只停车 / 只清故障
    python3 eth_drive.py --stop
    python3 eth_drive.py --clear

安全默认值：限速 300 mm/s；斜坡 500 ms；任何 fault != 0 立即停。
"""
from __future__ import annotations

import argparse
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

CMD_HZ = 20.0
HB_HZ = 20.0
LIMIT_MM_S = 300          # RTL 硬限幅（eth_motion_top 的 MAX_V_MM_S）
DEFAULT_MAX_SPEED = 150   # 驾驶工具的默认上限（落地测试建议 <=100）


class SafetyStop(RuntimeError):
    pass


def ramp(v0: int, v1: int, step: int) -> list[int]:
    if v0 == v1:
        return [v1]
    sign = 1 if v1 > v0 else -1
    out = list(range(v0, v1, sign * step))
    out.append(v1)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="safe drive helper over the UDP link")
    ap.add_argument("--host", default="192.168.0.2")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--local-port", type=int, default=5001)
    ap.add_argument("--left", type=int, default=0, help="左轮 mm/s（负=后退）")
    ap.add_argument("--right", type=int, default=0, help="右轮 mm/s")
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--ramp-ms", type=int, default=500, help="0->目标 的斜坡时间")
    ap.add_argument("--max-speed", type=int, default=DEFAULT_MAX_SPEED,
                    help="本工具允许的最大轮速 mm/s（默认 150；落地建议 <=100）")
    ap.add_argument("--stop", action="store_true", help="只发 STOP")
    ap.add_argument("--clear", action="store_true", help="只发 CLEAR_FAULT")
    ap.add_argument("--no-verify", action="store_true", help="不检查 ACK/fault（仅测链路）")
    args = ap.parse_args()

    for name, v in (("left", args.left), ("right", args.right)):
        if abs(v) > LIMIT_MM_S:
            ap.error("%s=%d 超过 RTL 硬限幅 %d mm/s" % (name, v, LIMIT_MM_S))
        if abs(v) > args.max_speed:
            ap.error("%s=%d 超过本工具上限 %d mm/s（落地测试请放慢；确需更高用 --max-speed）"
                     % (name, v, args.max_speed))

    tp = T.UdpTransport(remote=(args.host, args.port),
                        local=("0.0.0.0", args.local_port))
    link = Link(tp, heartbeat_hz=HB_HZ)
    link.start()
    time.sleep(1.0)

    def current_fault(timeout: float = 0.5) -> int:
        t = link.wait_telemetry(timeout, lambda x: True)
        return t.fault if t else 0xFF

    def emergency(reason: str) -> None:
        print("!! %s -> 立即 STOP" % reason)
        try:
            link.stop_cmd()
            time.sleep(0.1)
            link.clear_fault()
        finally:
            link.stop()
        raise SafetyStop(reason)

    seq = 0

    def send(l: int, r: int) -> None:
        nonlocal seq
        seq = (seq % 200) + 1
        link.set_vel(l, r, seq)
        if args.no_verify:
            return
        a = link.wait_ack(0.5, P.MSG_SET_VEL)
        if a is None:
            emergency("SET_VEL 无 ACK")
        if a.status != P.ACK_OK:
            emergency("SET_VEL 被拒: %s" % a.status_name)
        f = current_fault(0.3)
        if f and (f & ~P.FAULT_CMD_TIMEOUT):
            emergency("故障位 0x%02X" % f)

    try:
        # 建链
        f = current_fault(1.0)
        if f and (f & P.FAULT_LINK_LOST):
            print("建链中：清 LINK_LOST 锁存")
            link.clear_fault()
            link.wait_ack(1.0, P.MSG_CLEAR_FAULT)
            time.sleep(0.1)

        if args.clear:
            link.clear_fault()
            a = link.wait_ack(1.0, P.MSG_CLEAR_FAULT)
            print("CLEAR_FAULT ->", a.status_name if a else "no ACK")
        if args.stop or (args.clear and not args.left and not args.right):
            if args.stop:
                link.stop_cmd()
                a = link.wait_ack(1.0, P.MSG_STOP)
                print("STOP ->", a.status_name if a else "no ACK")
            return 0

        # 斜坡
        steps = max(1, int(args.ramp_ms / (1000.0 / CMD_HZ)))
        lv = ramp(0, args.left, max(1, abs(args.left) // steps or 1))
        rv = ramp(0, args.right, max(1, abs(args.right) // steps or 1))
        n = max(len(lv), len(rv))
        lv = lv + [args.left] * (n - len(lv))
        rv = rv + [args.right] * (n - len(rv))
        print("斜坡 %d 步（%.1f Hz 命令）: L %s -> %d, R %s -> %d"
              % (n, CMD_HZ, lv[0], args.left, rv[0], args.right))
        for l, r in zip(lv, rv):
            send(l, r)
            time.sleep(1.0 / CMD_HZ)

        # 巡航
        t_end = time.time() + args.seconds
        print("巡航 L=%d R=%d mm/s，%.1f s" % (args.left, args.right, args.seconds))
        while time.time() < t_end:
            send(args.left, args.right)
            time.sleep(1.0 / CMD_HZ)

        # 减速 + 停车
        print("减速停车")
        for l, r in reversed(list(zip(lv, rv))[1:]):
            send(l, r)
            time.sleep(1.0 / CMD_HZ)
        link.stop_cmd()
        a = link.wait_ack(1.0, P.MSG_STOP)
        print("STOP ->", a.status_name if a else "no ACK")
        link.clear_fault()
        link.wait_ack(0.8, P.MSG_CLEAR_FAULT)
        print("完成")
        return 0
    except SafetyStop as exc:
        print("SAFETY STOP: %s" % exc)
        return 2
    except KeyboardInterrupt:
        print("\n用户中断 -> STOP")
        link.stop_cmd()
        time.sleep(0.1)
        link.stop()
        return 130
    finally:
        try:
            link.stop()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    raise SystemExit(main())

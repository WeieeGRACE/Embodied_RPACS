#!/usr/bin/env python3
"""Orange Pi AIpro 侧的最小上位机示例（链路 v1）。

作用：在树莓派/Orange Pi 上一行命令跑通"心跳 + 遥测 + 速度命令"，用来确认
"深度相机板 → FPGA"这条链路在真机上可用。

用法（板子上）::

    # 1) 依赖（一般已装 python3；pyserial 需要装一次）
    sudo apt install -y python3-serial        # 或 pip3 install pyserial

    # 2) 权限：把当前用户加入 dialout 组，重新登录生效
    sudo usermod -aG dialout $USER

    # 3) 找串口
    python3 pi_host_example.py --list

    # 4) 监视遥测（只发心跳，不下发速度）
    python3 pi_host_example.py --port /dev/ttyUSB0 --monitor 10

    # 5) 发一条低速命令（车轮架空时再用！）
    python3 pi_host_example.py --port /dev/ttyUSB0 --setvel 80 80 --duration 2

注意：
  * 本脚本**默认不发送速度命令**，只做监视，避免误动作；
  * 下发速度前必须已经 `--clear`（或脚本会先自动发 CLEAR_FAULT）；
  * 停止命令（STOP）是锁存的，恢复需要 CLEAR_FAULT。

依赖：protocol.py / transport.py / uart_link.py 与本文件同目录（直接复用，不复制代码）。
"""
from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import protocol as P          # noqa: E402
import transport as T         # noqa: E402
from uart_link import Link    # noqa: E402


def cmd_list() -> int:
    print("可用串口：")
    for line in T.describe_serial_ports():
        print("  ", line)
    print("\n提示：FPGA 板载 USB 串口(115200) 通常显示为 CH9102/CH340 -> /dev/ttyUSB0")
    return 0


def cmd_monitor(args) -> int:
    tp = T.UartTransport(port=args.port, baud=args.baud)
    link = Link(tp, heartbeat_hz=args.heartbeat_hz)
    link.start()
    t0 = time.time()
    print("monitor %s  %.0f s (heartbeat %.0f Hz)" % (tp.name, args.monitor,
                                                      args.heartbeat_hz))
    try:
        last = 0.0
        while time.time() - t0 < args.monitor:
            time.sleep(0.1)
            now = time.time()
            if now - last < 1.0:
                continue
            last = now
            if link.telemetry:
                t = link.telemetry[-1]
                faults = ("[" + ",".join(t.fault_names) + "]") if t.fault_names else ""
                print("%6.1fs enc=(%d,%d) yaw=%+7.2f pitch=%+7.2f roll=%+7.2f "
                      "spd=(%d,%d) fault=0x%02X%s hb=%dms"
                      % (now - t0, t.enc_l, t.enc_r, t.yaw_deg, t.pitch_deg,
                         t.roll_deg, t.speed_l_mm_s, t.speed_r_mm_s, t.fault,
                         faults, t.hb_age_ms))
            else:
                print("%6.1fs 无遥测（检查接线/波特率/FPGA 是否在跑链路位流）" % (now - t0))
    finally:
        link.stop()
    return 0


def cmd_setvel(args) -> int:
    tp = T.UartTransport(port=args.port, baud=args.baud)
    link = Link(tp, heartbeat_hz=args.heartbeat_hz)
    link.start()
    try:
        time.sleep(0.3)
        if args.clear:
            link.clear_fault()
            ack = link.wait_ack(0.6, P.MSG_CLEAR_FAULT)
            print("CLEAR_FAULT -> %s" % (ack.status_name if ack else "no ACK"))
        print("SET_VEL %d %d for %.1f s ..." % (args.setvel[0], args.setvel[1],
                                                 args.duration))
        t0 = time.time()
        while time.time() - t0 < args.duration:
            link.set_vel(args.setvel[0], args.setvel[1])
            link.heartbeat()
            time.sleep(0.05)                    # 20 Hz
            if link.telemetry:
                t = link.telemetry[-1]
                if int((time.time() - t0) * 2) != int((time.time() - t0 - 0.05) * 2):
                    print("   spd=(%d,%d) fault=0x%02X"
                          % (t.speed_l_mm_s, t.speed_r_mm_s, t.fault))
        # 结束时一定停车
        link.stop_cmd()
        time.sleep(0.15)
        print("STOP sent (latch: 需要 CLEAR_FAULT 才能恢复)")
    finally:
        link.stop()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="Orange Pi host example (link v1)")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--heartbeat-hz", type=float, default=10.0)
    ap.add_argument("--list", action="store_true", help="list serial ports")
    ap.add_argument("--monitor", type=float, default=0.0, help="monitor seconds")
    ap.add_argument("--setvel", type=int, nargs=2, metavar=("LEFT", "RIGHT"),
                    help="send SET_VEL for --duration seconds (wheels off the ground!)")
    ap.add_argument("--duration", type=float, default=1.0)
    ap.add_argument("--clear", action="store_true",
                    help="send CLEAR_FAULT before SET_VEL")
    args = ap.parse_args()

    if args.list:
        return cmd_list()
    if args.setvel:
        return cmd_setvel(args)
    if args.monitor > 0:
        return cmd_monitor(args)
    ap.print_help()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""UDP 传输自检（为未来千兆网 v2 链路做准备）。

不接真板子，用本机回环验证：
  1. `UdpTransport` 发送的每个字节流恰好是一个 UDP 报文；
  2. 接收端把报文载荷拼起来后，`FrameParser` 能按与 UART 完全相同的逻辑解析出帧；
  3. 混合帧（TELEM/ACK/SET_VEL）+ 故意坏 CRC 的帧，错误分类也一致。

用法::

    python udp_loopback_test.py [--count 200]
"""
from __future__ import annotations

import argparse
import os
import random
import socket
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import protocol as P          # noqa: E402
import transport as T         # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="UDP transport loopback self-test")
    ap.add_argument("--count", type=int, default=200)
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args()

    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:  # noqa: BLE001
        pass

    # receiver
    rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx_sock.bind(("127.0.0.1", 0))
    rx_port = rx_sock.getsockname()[1]
    rx_sock.settimeout(1.0)
    received = []
    stop = threading.Event()

    def receiver():
        while not stop.is_set():
            try:
                data, _ = rx_sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            received.append(data)

    th = threading.Thread(target=receiver, daemon=True)
    th.start()

    # sender uses the real transport class
    tx = T.UdpTransport(remote=("127.0.0.1", rx_port), local=("127.0.0.1", 0))
    rnd = random.Random(args.seed)
    sent_frames = []
    bad = 0
    for i in range(args.count):
        kind = i % 4
        if kind == 0:
            payload = P.build_telem(enc_l=i, enc_r=-i, yaw_centideg=i * 10,
                                    pitch_centideg=-100, roll_centideg=50,
                                    speed_l=i % 300, speed_r=-(i % 300),
                                    fault=i & 0xFF, link_seq=i & 0xFF,
                                    cmd_seq=(i * 3) & 0xFF, hb_age_ms=i % 400)
            frame = P.build_frame(P.MSG_TELEM, payload)
        elif kind == 1:
            frame = P.build_ack(i & 0xFF, (i * 7) & 0xFF, 0x00)
        elif kind == 2:
            frame = P.msg_set_vel(rnd.randint(-300, 300), rnd.randint(-300, 300),
                                 i & 0xFF)
        else:
            f = bytearray(P.msg_heartbeat(i & 0xFF))
            f[-3] ^= 0xFF                       # broken CRC, must be rejected
            frame = bytes(f)
            bad += 1
        sent_frames.append(frame)
        tx.send(frame)
        time.sleep(0.001)
    time.sleep(0.2)
    stop.set()
    rx_sock.close()
    tx.close()

    # parse the received datagrams exactly like the UART path does
    parser = P.FrameParser()
    frames = []
    for dg in received:
        frames.extend(parser.feed(dg))

    print("sent datagrams : %d" % len(sent_frames))
    print("received       : %d (每报文应恰好一帧)" % len(received))
    print("parsed frames  : %d  (期望 %d 个合法帧)" % (len(frames), args.count - bad))
    print("stats          :", parser.stats.as_dict())

    ok = (len(received) == len(sent_frames)
          and len(frames) == args.count - bad
          and parser.stats.crc_err == bad)
    # 再验证：每个收到的报文要么解析出 0 帧（坏 CRC），要么恰好 1 帧
    per_datagram = []
    p2 = P.FrameParser()
    for dg in received:
        per_datagram.append(len(p2.feed(dg)))
    if any(n > 1 for n in per_datagram):
        print("FAIL 某个报文解析出多帧（报文边界被破坏）")
        ok = False

    print("\nUDP TRANSPORT %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())

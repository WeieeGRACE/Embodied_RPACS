#!/usr/bin/env python3
"""千兆以太网 UDP 回环测试（Pi 端跑）。

对端：ACG525 板 RGMII 以太网 + 厂商 `eth_udp_loopback_rgmii` 例程
    MAC 00:0a:35:01:fe:c0 / IP 192.168.0.2 / UDP 5000
该例程把收到的 UDP 负载原样回环，所以这里发带序号的报文并统计丢包/时延。

前置（Pi）：
    eth0 静态 192.168.0.1/24
    静态 ARP：ip neigh replace 192.168.0.2 lladdr 00:0a:35:01:fe:c0 dev eth0 nud permanent
    （例程不实现 ARP，必须手工填表）

用法::

    python3 udp_echo_test.py                          # 200 个报文，10 ms 间隔
    python3 udp_echo_test.py --count 2000 --interval 0.002 --json out.json
    python3 udp_echo_test.py --selftest                # 本机回环自检（无需硬件）
"""
from __future__ import annotations

import argparse
import json
import socket
import statistics
import struct
import sys
import threading
import time

MAGIC = b"PING"


def make_payload(seq: int, size: int) -> bytes:
    size = max(8, size)
    head = MAGIC + struct.pack("<I", seq)
    body = bytes(((seq + i) & 0xFF) for i in range(size - len(head)))
    return head + body


def parse_seq(data: bytes) -> int | None:
    if len(data) >= 8 and data[:4] == MAGIC:
        return struct.unpack("<I", data[4:8])[0]
    return None


def run_test(target_ip: str, port: int, count: int, interval: float,
             size: int, timeout: float, local_port: int = 0) -> dict:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", local_port))
    sock.settimeout(timeout)

    sent_at: dict[int, float] = {}
    rtts: list[float] = []
    got: list[int] = []
    bad_payload = 0

    t0 = time.time()
    end_send = t0
    n_sent = 0
    n_recv = 0
    next_recv_deadline = time.time() + timeout

    while True:
        now = time.time()
        if n_sent < count and now >= t0 + n_sent * interval:
            seq = n_sent
            payload = make_payload(seq, size)
            sock.sendto(payload, (target_ip, port))
            sent_at[seq] = time.time()
            n_sent += 1
            end_send = time.time()
            next_recv_deadline = time.time() + timeout
        try:
            data, _ = sock.recvfrom(4096)
        except socket.timeout:
            if n_sent >= count and time.time() > end_send + timeout:
                break
            if n_sent >= count:
                continue
            continue
        seq = parse_seq(data)
        if seq is None:
            bad_payload += 1
            continue
        if seq in sent_at:
            rtts.append((time.time() - sent_at.pop(seq)) * 1000.0)
            got.append(seq)
            n_recv += 1
        if n_sent >= count and not sent_at:
            break
        if n_sent >= count and time.time() > next_recv_deadline:
            break

    sock.close()
    total_s = time.time() - t0
    loss = (count - n_recv) / count * 100.0 if count else 0.0
    out_of_order = sum(1 for a, b in zip(got, got[1:]) if b < a)
    res = {
        "target": "%s:%d" % (target_ip, port),
        "sent": n_sent, "received": n_recv, "loss_percent": round(loss, 3),
        "duration_s": round(total_s, 3),
        "pps": round(n_sent / total_s, 1) if total_s > 0 else 0.0,
        "rtt_ms": ({
            "min": round(min(rtts), 3),
            "mean": round(statistics.mean(rtts), 3),
            "p50": round(statistics.median(rtts), 3),
            "p95": round(sorted(rtts)[max(0, int(len(rtts) * 0.95) - 1)], 3),
            "max": round(max(rtts), 3),
        } if rtts else {}),
        "out_of_order": out_of_order,
        "bad_payload": bad_payload,
        "ok": (n_recv > 0 and loss <= 1.0
               and (not rtts or statistics.median(rtts) < 5.0)),
    }
    return res


def selftest() -> int:
    """本地 UDP 回环：验证统计逻辑（不需要板子）。"""
    srv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    srv.bind(("127.0.0.1", 0))
    port = srv.getsockname()[1]

    stop = threading.Event()

    def echo() -> None:
        srv.settimeout(0.1)
        while not stop.is_set():
            try:
                data, addr = srv.recvfrom(4096)
                srv.sendto(data, addr)
            except socket.timeout:
                continue
            except OSError:
                break

    th = threading.Thread(target=echo, daemon=True)
    th.start()
    res = run_test("127.0.0.1", port, count=40, interval=0.002, size=32, timeout=0.2)
    stop.set()
    th.join(timeout=1.0)
    srv.close()
    print(json.dumps(res, indent=2, ensure_ascii=False))
    ok = res["sent"] == 40 and res["loss_percent"] == 0.0 and res["rtt_ms"]["p50"] < 5.0
    print("\nSELFTEST %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


def main() -> int:
    ap = argparse.ArgumentParser(description="gigabit UDP echo test (Pi side)")
    ap.add_argument("--target-ip", default="192.168.0.2")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--count", type=int, default=200)
    ap.add_argument("--interval", type=float, default=0.01, help="发送间隔 s")
    ap.add_argument("--size", type=int, default=32, help="负载字节数")
    ap.add_argument("--timeout", type=float, default=0.5)
    ap.add_argument("--local-port", type=int, default=0)
    ap.add_argument("--json", default=None)
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        return selftest()

    print("[udp] %s:%d  count=%d interval=%.3fs size=%d"
          % (args.target_ip, args.port, args.count, args.interval, args.size))
    res = run_test(args.target_ip, args.port, args.count, args.interval,
                   args.size, args.timeout, args.local_port)
    print(json.dumps(res, indent=2, ensure_ascii=False))
    print("\nUDP ECHO %s" % ("PASS" if res["ok"] else "FAIL"))
    if args.json:
        from pathlib import Path
        Path(args.json).parent.mkdir(parents=True, exist_ok=True)
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(res, f, indent=2, ensure_ascii=False)
        print("-> %s" % args.json)
    return 0 if res["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

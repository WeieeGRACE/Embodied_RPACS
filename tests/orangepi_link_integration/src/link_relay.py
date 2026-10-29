#!/usr/bin/env python3
"""链路中继：串口(无线串口) <-> UDP(千兆网) 双向转发。

新拓扑（2026-10-29 晚）：
     FPGA ──无线串口──> Orange Pi ──1GbE UDP──> PC / 其它订阅者
     FPGA <──无线串口── Orange Pi <──1GbE UDP── PC 下发的命令

一帧 = 一个 UDP 报文（与 docs/PROTOCOL_v1.md §7 的约定一致），所以 UDP 侧可以直接用
`transports.UdpTransport` 收发；串口侧沿用同一套 `protocol.FrameParser`。

用法（Pi 上）::

    # 串口 -> UDP(PC:50000)，同时把 PC 发给 50001 的命令写回串口
    python3 src/link_relay.py --serial /dev/ttyAMA1 --baud 115200 \
            --udp-host 172.30.51.25 --udp-port 50000 --local-port 50001

    # 只看串口侧速率、不转发（自检/排障）
    python3 src/link_relay.py --serial /dev/ttyAMA1 --stats-only

    # PC 上离线验证（file 传输 + 本地 UDP）
    python3 src/link_relay.py --transport file --file capture.hex --udp-host 127.0.0.1
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve()
TESTS = HERE.parents[2]
UART_TOOLS = Path(os.environ.get("UART_BRIDGE_TOOLS", TESTS / "uart_bridge" / "tools"))
if str(UART_TOOLS) not in sys.path:
    sys.path.insert(0, str(UART_TOOLS))

try:
    import protocol as P
    import transport as T
except ImportError as exc:  # pragma: no cover
    raise SystemExit("无法 import tests/uart_bridge/tools（%s）" % exc)


class Relay:
    def __init__(self, ser: T.Transport, udp_sock, udp_target, stats_only=False):
        self.ser = ser
        self.udp = udp_sock
        self.udp_target = udp_target
        self.stats_only = stats_only
        self.parser = P.FrameParser()
        self._stop = threading.Event()
        self.frames_to_udp = 0
        self.bytes_to_udp = 0
        self.frames_from_udp = 0
        self.bytes_from_udp = 0
        self.serial_bytes = 0
        self.udp_send_err = 0
        self.udp_recv_err = 0
        self.t0 = time.time()
        self._threads: list[threading.Thread] = []

    # ---------------------------------------------------------------- 串口侧
    def _serial_in(self) -> None:
        while not self._stop.is_set():
            try:
                data = self.ser.read(0.01)
            except Exception:  # noqa: BLE001
                data = b""
            if not data:
                continue
            self.serial_bytes += len(data)
            for frame in self.parser.feed(data):
                raw = frame.to_bytes() if hasattr(frame, "to_bytes") else None
                if raw is None:
                    raw = P.build_frame(frame.msg_type, frame.payload)
                if not self.stats_only:
                    try:
                        self.udp.sendto(raw, self.udp_target)
                        self.frames_to_udp += 1
                        self.bytes_to_udp += len(raw)
                    except OSError:
                        self.udp_send_err += 1

    # ---------------------------------------------------------------- UDP 侧
    def _udp_in(self) -> None:
        self.udp.settimeout(0.2)
        while not self._stop.is_set():
            try:
                data, _ = self.udp.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                if self._stop.is_set():
                    break
                self.udp_recv_err += 1
                continue
            self.frames_from_udp += 1
            self.bytes_from_udp += len(data)
            if not self.stats_only:
                try:
                    self.ser.send(data)
                except Exception:  # noqa: BLE001
                    pass

    def start(self) -> None:
        for fn in (self._serial_in, self._udp_in):
            th = threading.Thread(target=fn, daemon=True)
            th.start()
            self._threads.append(th)

    def stop(self) -> None:
        self._stop.set()
        for th in self._threads:
            th.join(timeout=1.0)

    def snapshot(self) -> dict:
        dt = max(1e-6, time.time() - self.t0)
        return {
            "uptime_s": round(dt, 1),
            "serial_bytes": self.serial_bytes,
            "serial_frames_to_udp": self.frames_to_udp,
            "serial_fps_to_udp": round(self.frames_to_udp / dt, 2),
            "udp_bytes_in": self.bytes_from_udp,
            "udp_frames_from_udp": self.frames_from_udp,
            "udp_send_err": self.udp_send_err,
            "udp_recv_err": self.udp_recv_err,
            "parser": self.parser.stats.as_dict(),
        }


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="serial <-> UDP link relay")
    ap.add_argument("--transport", default="uart", choices=["uart", "file"])
    ap.add_argument("--serial", default="/dev/ttyAMA1", help="uart 传输的端口")
    ap.add_argument("--file", default="", help="file 传输的回放文件（自检用）")
    ap.add_argument("--sink", default="", help="file 传输的写入文件（自检用）")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--udp-host", default="127.0.0.1")
    ap.add_argument("--udp-port", type=int, default=50000)
    ap.add_argument("--local-port", type=int, default=50001)
    ap.add_argument("--duration", type=float, default=0.0, help="0 = 一直跑")
    ap.add_argument("--json", default=None)
    ap.add_argument("--stats-only", action="store_true")
    args = ap.parse_args(argv)

    if args.transport == "uart":
        ser = T.UartTransport(port=args.serial, baud=args.baud)
    else:
        ser = T.FileTransport(path=args.file, sink=args.sink or None)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", args.local_port))
    target = (args.udp_host, args.udp_port)
    relay = Relay(ser, sock, target, args.stats_only)
    relay.start()
    print("[relay] serial=%s -> udp %s:%d ; udp %d -> serial  (stats_only=%s)"
          % (ser.name, args.udp_host, args.udp_port, args.local_port, args.stats_only))
    t_end = time.time() + args.duration if args.duration > 0 else None
    last = time.time()
    try:
        while t_end is None or time.time() < t_end:
            time.sleep(0.2)
            if time.time() - last >= 2.0:
                last = time.time()
                s = relay.snapshot()
                print("[relay] %5.0fs serial %6.1ffps  udp_in %5.1ffps  err(send/recv) %d/%d"
                      % (s["uptime_s"], s["serial_fps_to_udp"],
                         s["udp_frames_from_udp"] / max(1e-6, s["uptime_s"]),
                         s["udp_send_err"], s["udp_recv_err"]))
    except KeyboardInterrupt:
        pass
    finally:
        relay.stop()
        sock.close()
    res = relay.snapshot()
    print("\n[relay] summary:", json.dumps(res, ensure_ascii=False))
    if args.json:
        Path(args.json).parent.mkdir(parents=True, exist_ok=True)
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(res, f, indent=2, ensure_ascii=False)
        print("-> %s" % args.json)
    ok = res["parser"]["good"] > 0 or args.transport == "file"
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())

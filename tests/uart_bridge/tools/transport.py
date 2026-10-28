#!/usr/bin/env python3
"""传输层抽象：把「帧字节流」搬运到 UART / UDP / 日志文件。

协议层（protocol.py）完全不关心传输；换千兆网时只需新增/替换这里的实现，
帧格式、CRC、消息语义一字不改（见 docs/PROTOCOL_v1.md §1 与 §7）。

对外接口::

    t = open_transport("uart", port="COM14", baud=115200)
    t.send(frame_bytes)            # 发送一个完整帧（UDP 下恰好一个报文）
    data = t.read(timeout=0.05)    # 返回自上次调用以来收到的字节（可能为空）
    t.close()

三个实现：
    UartTransport   pyserial 串口（COM14，CH9102）
    UdpTransport    UDP 单播（未来千兆网；一个报文=一个帧）
    FileTransport   从 hexdump/二进制日志回放（离线复现测试用）
"""
from __future__ import annotations

import binascii
import os
import socket
import threading
import time
from typing import List, Optional


class Transport:
    """传输层基类。"""

    name = "base"

    def send(self, data: bytes) -> None:      # pragma: no cover - interface
        raise NotImplementedError

    def read(self, timeout: float = 0.0) -> bytes:   # pragma: no cover
        raise NotImplementedError

    def close(self) -> None:                  # pragma: no cover
        raise NotImplementedError

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


# --------------------------------------------------------------------------
# UART（当前链路）
# --------------------------------------------------------------------------
class UartTransport(Transport):
    """115200 8N1 串口。需要 pyserial。"""

    def __init__(self, port: str = "COM14", baud: int = 115200,
                 read_timeout: float = 0.02, write_timeout: float = 0.5):
        import serial  # 延迟导入，方便无硬件环境下只做编解码测试

        self.name = "uart:%s@%d" % (port, baud)
        self._ser = serial.Serial()
        self._ser.port = port
        self._ser.baudrate = baud
        self._ser.bytesize = serial.EIGHTBITS
        self._ser.parity = serial.PARITY_NONE
        self._ser.stopbits = serial.STOPBITS_ONE
        self._ser.timeout = read_timeout
        self._ser.write_timeout = write_timeout
        self._ser.xonxoff = False
        self._ser.rtscts = False
        self._ser.open()
        self._ser.reset_input_buffer()
        self._lock = threading.Lock()

    def send(self, data: bytes) -> None:
        with self._lock:
            self._ser.write(data)
            self._ser.flush()

    def read(self, timeout: float = 0.02) -> bytes:
        self._ser.timeout = max(0.001, timeout)
        n = self._ser.in_waiting
        if n <= 0:
            return b""
        return self._ser.read(n)

    def close(self) -> None:
        try:
            if self._ser.is_open:
                self._ser.close()
        except Exception:  # noqa: BLE001
            pass


# --------------------------------------------------------------------------
# UDP over 1 GbE（未来链路，帧格式不变）
# --------------------------------------------------------------------------
class UdpTransport(Transport):
    """UDP 单播。一个 UDP 报文承载一个完整帧（AA 55 ... 0D）。

    用法::

        t = UdpTransport(remote=("192.168.1.50", 50000),
                         local=("0.0.0.0", 50001))
        t.send(frame)        # 一个报文一帧，接收端按帧解析
        raw = t.read(0.05)   # 返回报文载荷拼接（多帧可直接喂 FrameParser）
    """

    def __init__(self, remote, local=None, timeout: float = 0.1):
        self.name = "udp:%s:%d" % (remote[0], remote[1])
        self._remote = tuple(remote)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if local is not None:
            self._sock.bind(tuple(local))
        self._sock.settimeout(timeout)
        self._pending = bytearray()

    def send(self, data: bytes) -> None:
        self._sock.sendto(bytes(data), self._remote)

    def read(self, timeout: float = 0.1) -> bytes:
        self._sock.settimeout(max(0.001, timeout))
        while True:
            try:
                payload, _ = self._sock.recvfrom(4096)
            except socket.timeout:
                break
            self._pending.extend(payload)
            if len(self._pending) > 65536:      # 防御性上限
                break
        out = bytes(self._pending)
        self._pending.clear()
        return out

    def close(self) -> None:
        try:
            self._sock.close()
        except Exception:  # noqa: BLE001
            pass


# --------------------------------------------------------------------------
# 文件回放（离线测试 / 复现现场日志）
# --------------------------------------------------------------------------
class FileTransport(Transport):
    """把文本日志里的 hex 或二进制数据当作接收流回放，发送写入 sink 文件。

    日志格式：每行 ``RX <hex>`` 或纯 hex 行；空行与 ``#`` 注释忽略。
    """

    def __init__(self, path: str, sink: Optional[str] = None):
        self.name = "file:%s" % path
        self._data = bytearray()
        with open(path, "rb") as f:
            raw = f.read()
        for line in raw.splitlines():
            line = line.strip()
            if not line or line.startswith(b"#"):
                continue
            if line.startswith(b"RX "):
                line = line[3:].strip()
            for tok in line.split():
                self._data.extend(binascii.unhexlify(tok))
        self._pos = 0
        self._sink = open(sink, "ab") if sink else None

    def send(self, data: bytes) -> None:
        if self._sink:
            self._sink.write(binascii.hexlify(data) + b"\n")
            self._sink.flush()

    def read(self, timeout: float = 0.0) -> bytes:
        out = bytes(self._data[self._pos:])
        self._pos = len(self._data)
        return out

    def close(self) -> None:
        if self._sink:
            self._sink.close()
            self._sink = None


# --------------------------------------------------------------------------
def open_transport(kind: str, **kwargs) -> Transport:
    """工厂函数，供 CLI 复用。"""
    kind = kind.lower()
    if kind in ("uart", "serial", "com"):
        return UartTransport(**kwargs)
    if kind in ("udp", "ethernet", "eth"):
        return UdpTransport(**kwargs)
    if kind in ("file", "replay", "log"):
        return FileTransport(**kwargs)
    raise ValueError("未知传输类型: %r" % (kind,))


def list_serial_ports() -> List[str]:
    try:
        from serial.tools import list_ports
    except Exception:  # noqa: BLE001
        return []
    return [p.device for p in list_ports.comports()]


def describe_serial_ports() -> List[str]:
    try:
        from serial.tools import list_ports
    except Exception:  # noqa: BLE001
        return []
    out = []
    for p in list_ports.comports():
        out.append("%s  %s  [%s]" % (p.device, p.description, p.hwid))
    return out


if __name__ == "__main__":  # pragma: no cover - 手工检查
    print("serial ports:")
    for line in describe_serial_ports():
        print("  ", line)
    print("udp transport available:", UdpTransport is not None)
    print("file transport available:", FileTransport is not None)
    print("cwd:", os.getcwd(), "time:", time.strftime("%H:%M:%S"))

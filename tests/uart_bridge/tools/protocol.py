#!/usr/bin/env python3
"""上位机 ↔ FPGA 链路协议 v1 —— 编解码层（与传输无关）。

本模块只负责「消息 ↔ 字节帧」，不关心字节从哪里来（UART / UDP / 日志回放）。
传输实现见 transport_uart.py / transport_udp.py，二者提供同样的接口。

规格见 docs/PROTOCOL_v1.md。

帧格式::

    AA 55 | TYPE(1) | LEN(1) | PAYLOAD(LEN) | CRC16-CCITT(2, 小端) | 0D

CRC 覆盖 TYPE..PAYLOAD；CRC 参数 CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF)。
多字节整数一律小端。
"""
from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import Iterable, List, Optional, Tuple

# --------------------------------------------------------------------------
# 常量
# --------------------------------------------------------------------------
SYNC0 = 0xAA
SYNC1 = 0x55
TAIL = 0x0D
MAX_PAYLOAD = 32
HEADER_LEN = 4          # AA 55 TYPE LEN
CRC_LEN = 2
FRAME_OVERHEAD = 7      # header + crc + tail

#: 下行（上位机 → FPGA）
MSG_SET_VEL = 0x01
MSG_STOP = 0x02
MSG_HEARTBEAT = 0x03
MSG_SET_LIMIT = 0x04
MSG_CLEAR_FAULT = 0x05

#: 上行（FPGA → 上位机）
MSG_TELEM = 0x81
MSG_ACK = 0x82

TYPE_NAMES = {
    MSG_SET_VEL: "SET_VEL",
    MSG_STOP: "STOP",
    MSG_HEARTBEAT: "HEARTBEAT",
    MSG_SET_LIMIT: "SET_LIMIT",
    MSG_CLEAR_FAULT: "CLEAR_FAULT",
    MSG_TELEM: "TELEM",
    MSG_ACK: "ACK",
}

#: 各下行 TYPE 的合法 PAYLOAD 长度
DOWNLINK_LEN = {
    MSG_SET_VEL: 5,
    MSG_STOP: 0,
    MSG_HEARTBEAT: 1,
    MSG_SET_LIMIT: 2,
    MSG_CLEAR_FAULT: 0,
}

#: ACK 状态码
ACK_OK = 0x00
ACK_REJECT_LINK = 0x01
ACK_REJECT_STOP = 0x02
ACK_REJECT_LIMIT = 0x03
ACK_UNSUPPORTED = 0x04

ACK_STATUS_NAMES = {
    ACK_OK: "OK",
    ACK_REJECT_LINK: "REJECT_LINK_LOST",
    ACK_REJECT_STOP: "REJECT_STOP_LATCHED",
    ACK_REJECT_LIMIT: "REJECT_OVER_LIMIT",
    ACK_UNSUPPORTED: "UNSUPPORTED",
}

#: 故障位
FAULT_STOP_LATCHED = 1 << 0
FAULT_LINK_LOST = 1 << 1
FAULT_ESTOP = 1 << 2
FAULT_OVER_LIMIT = 1 << 3
FAULT_CRC_ERR = 1 << 4
FAULT_CMD_TIMEOUT = 1 << 5

FAULT_BITS = {
    FAULT_STOP_LATCHED: "STOP_LATCHED",
    FAULT_LINK_LOST: "LINK_LOST",
    FAULT_ESTOP: "ESTOP",
    FAULT_OVER_LIMIT: "OVER_LIMIT",
    FAULT_CRC_ERR: "CRC_ERR",
    FAULT_CMD_TIMEOUT: "CMD_TIMEOUT",
}

TELEM_STRUCT = struct.Struct("<iihhhhhBBBH")   # 23 bytes
assert TELEM_STRUCT.size == 23


# --------------------------------------------------------------------------
# CRC
# --------------------------------------------------------------------------
def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE：poly 0x1021, init 0xFFFF, 不反射, 无最终异或。"""
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


# --------------------------------------------------------------------------
# 帧封装 / 解析
# --------------------------------------------------------------------------
def build_frame(msg_type: int, payload: bytes = b"") -> bytes:
    """把一条消息封成完整帧（含 CRC 与尾字节）。"""
    if not 0 <= msg_type <= 0xFF:
        raise ValueError("msg_type 越界: %r" % (msg_type,))
    payload = bytes(payload)
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload 过长: %d > %d" % (len(payload), MAX_PAYLOAD))
    body = bytes([msg_type, len(payload)]) + payload
    crc = crc16_ccitt(body)
    return bytes([SYNC0, SYNC1]) + body + struct.pack("<H", crc) + bytes([TAIL])


@dataclass
class Frame:
    """一帧解析结果。"""

    msg_type: int
    payload: bytes
    raw: bytes

    @property
    def name(self) -> str:
        return TYPE_NAMES.get(self.msg_type, "0x%02X" % self.msg_type)

    def __repr__(self) -> str:  # pragma: no cover - 调试用
        return "Frame(%s, len=%d, %s)" % (self.name, len(self.payload),
                                          self.payload.hex(" "))


@dataclass
class ParseStats:
    """流式解析统计（与 RTL 侧计数器一一对应）。"""

    good: int = 0
    sync_err: int = 0
    len_err: int = 0
    crc_err: int = 0
    tail_err: int = 0
    timeout: int = 0
    type_err: int = 0

    def as_dict(self) -> dict:
        return {
            "good": self.good, "sync_err": self.sync_err, "len_err": self.len_err,
            "crc_err": self.crc_err, "tail_err": self.tail_err,
            "timeout": self.timeout, "type_err": self.type_err,
        }


class FrameParser:
    """增量式帧解析器：喂任意切分的字节流，吐出完整帧并累计错误统计。

    与 RTL 的 host_frame_rx 行为对齐：任何错误都只丢弃，绝不把半帧内容交出去。
    """

    def __init__(self, validate_len: bool = True) -> None:
        self.validate_len = validate_len
        self._buf = bytearray()
        self.stats = ParseStats()

    def reset(self) -> None:
        self._buf.clear()

    def feed(self, data: bytes) -> List[Frame]:
        """喂入字节，返回本次新解析出的完整帧列表。"""
        self._buf.extend(data)
        frames: List[Frame] = []
        buf = self._buf
        while True:
            # 1) 找同步字
            while len(buf) >= 2 and not (buf[0] == SYNC0 and buf[1] == SYNC1):
                if buf[0] == SYNC0:
                    del buf[0]                      # 半同步，保留可能是 SYNC1 的字节
                else:
                    del buf[0]
                self.stats.sync_err += 1
            if len(buf) < 2:
                break
            # 2) 头部未齐
            if len(buf) < HEADER_LEN:
                break
            msg_type, length = buf[2], buf[3]
            if length > MAX_PAYLOAD:
                self.stats.len_err += 1
                del buf[0]
                continue
            total = FRAME_OVERHEAD + length
            if len(buf) < total:
                break
            body = bytes(buf[2:4 + length])
            crc_rx = struct.unpack("<H", buf[4 + length:6 + length])[0]
            crc_calc = crc16_ccitt(body)
            tail = buf[6 + length]
            if crc_rx != crc_calc:
                self.stats.crc_err += 1
                del buf[0]
                continue
            if tail != TAIL:
                self.stats.tail_err += 1
                del buf[0]
                continue
            if self.validate_len and msg_type in DOWNLINK_LEN \
                    and DOWNLINK_LEN[msg_type] != length:
                self.stats.type_err += 1
                del buf[0]
                continue
            raw = bytes(buf[:total])
            frames.append(Frame(msg_type, bytes(buf[4:4 + length]), raw))
            self.stats.good += 1
            del buf[:total]
        return frames


# --------------------------------------------------------------------------
# 下行消息构造
# --------------------------------------------------------------------------
def msg_set_vel(v_left_mm_s: int, v_right_mm_s: int, seq: int = 0) -> bytes:
    """SET_VEL：左右轮目标速度（mm/s，int16）+ 序号 seq（uint8，用于丢帧统计）。

    seq 会被 FPGA 原样回显到 TELEM.cmd_seq，上位机据此计算命令丢失率。
    """
    for v in (v_left_mm_s, v_right_mm_s):
        if not -32768 <= v <= 32767:
            raise ValueError("速度越界: %r" % (v,))
    return build_frame(MSG_SET_VEL, struct.pack("<hhB", v_left_mm_s, v_right_mm_s,
                                                seq & 0xFF))


def msg_stop() -> bytes:
    return build_frame(MSG_STOP)


def msg_heartbeat(seq: int) -> bytes:
    return build_frame(MSG_HEARTBEAT, bytes([seq & 0xFF]))


def msg_set_limit(limit_mm_s: int) -> bytes:
    if not 0 <= limit_mm_s <= 65535:
        raise ValueError("限速越界: %r" % (limit_mm_s,))
    return build_frame(MSG_SET_LIMIT, struct.pack("<H", limit_mm_s))


def msg_clear_fault() -> bytes:
    return build_frame(MSG_CLEAR_FAULT)


# --------------------------------------------------------------------------
# 上行消息解析
# --------------------------------------------------------------------------
@dataclass
class Telemetry:
    """TELEM 解码结果。"""

    enc_l: int
    enc_r: int
    yaw_deg: float
    pitch_deg: float
    roll_deg: float
    speed_l_mm_s: int
    speed_r_mm_s: int
    fault: int
    link_seq: int
    cmd_seq: int
    hb_age_ms: int

    @property
    def fault_names(self) -> List[str]:
        return [name for bit, name in FAULT_BITS.items() if self.fault & bit]

    def as_dict(self) -> dict:
        return {
            "enc_l": self.enc_l, "enc_r": self.enc_r,
            "yaw_deg": round(self.yaw_deg, 2),
            "pitch_deg": round(self.pitch_deg, 2),
            "roll_deg": round(self.roll_deg, 2),
            "speed_l_mm_s": self.speed_l_mm_s,
            "speed_r_mm_s": self.speed_r_mm_s,
            "fault": self.fault, "fault_names": self.fault_names,
            "link_seq": self.link_seq, "cmd_seq": self.cmd_seq,
            "hb_age_ms": self.hb_age_ms,
        }


def decode_telem(payload: bytes) -> Telemetry:
    if len(payload) != TELEM_STRUCT.size:
        raise ValueError("TELEM 长度错: %d != %d" % (len(payload), TELEM_STRUCT.size))
    (enc_l, enc_r, yaw, pitch, roll, sp_l, sp_r,
     fault, link_seq, cmd_seq, hb_age) = TELEM_STRUCT.unpack(payload)
    return Telemetry(enc_l, enc_r, yaw / 100.0, pitch / 100.0, roll / 100.0,
                     sp_l, sp_r, fault, link_seq, cmd_seq, hb_age)


def build_telem(enc_l: int, enc_r: int, yaw_centideg: int, pitch_centideg: int,
                roll_centideg: int, speed_l: int, speed_r: int, fault: int,
                link_seq: int, cmd_seq: int, hb_age_ms: int) -> bytes:
    """构造 TELEM 载荷（RTL 打包逻辑的镜像实现，供测试比对）。"""
    return TELEM_STRUCT.pack(enc_l, enc_r, yaw_centideg, pitch_centideg,
                             roll_centideg, speed_l, speed_r, fault,
                             link_seq & 0xFF, cmd_seq & 0xFF,
                             max(0, min(65535, hb_age_ms)))


@dataclass
class Ack:
    acked_type: int
    seq: int
    status: int

    @property
    def status_name(self) -> str:
        return ACK_STATUS_NAMES.get(self.status, "0x%02X" % self.status)


def decode_ack(payload: bytes) -> Ack:
    if len(payload) != 3:
        raise ValueError("ACK 长度错: %d != 3" % len(payload))
    return Ack(payload[0], payload[1], payload[2])


def build_ack(acked_type: int, seq: int, status: int) -> bytes:
    return build_frame(MSG_ACK, bytes([acked_type & 0xFF, seq & 0xFF, status & 0xFF]))


# --------------------------------------------------------------------------
# 便捷入口
# --------------------------------------------------------------------------
def describe(frame: Frame) -> str:
    """把一帧变成一行人类可读文本，供 CLI/日志使用。"""
    try:
        if frame.msg_type == MSG_SET_VEL and len(frame.payload) == 5:
            vl, vr, seq = struct.unpack("<hhB", frame.payload)
            return ("SET_VEL  v_left=%+6d mm/s  v_right=%+6d mm/s  seq=%d"
                    % (vl, vr, seq))
        if frame.msg_type == MSG_SET_LIMIT and len(frame.payload) == 2:
            (lim,) = struct.unpack("<H", frame.payload)
            return "SET_LIMIT  limit=%d mm/s" % lim
        if frame.msg_type == MSG_HEARTBEAT and len(frame.payload) == 1:
            return "HEARTBEAT  seq=%d" % frame.payload[0]
        if frame.msg_type == MSG_STOP:
            return "STOP"
        if frame.msg_type == MSG_CLEAR_FAULT:
            return "CLEAR_FAULT"
        if frame.msg_type == MSG_TELEM:
            t = decode_telem(frame.payload)
            return ("TELEM  enc=(%d,%d) yaw=%+.2f pitch=%+.2f roll=%+.2f "
                    "spd=(%d,%d) fault=%02X%s link_seq=%d cmd_seq=%d hb_age=%dms"
                    % (t.enc_l, t.enc_r, t.yaw_deg, t.pitch_deg, t.roll_deg,
                       t.speed_l_mm_s, t.speed_r_mm_s, t.fault,
                       ("[" + ",".join(t.fault_names) + "]") if t.fault_names else "",
                       t.link_seq, t.cmd_seq, t.hb_age_ms))
        if frame.msg_type == MSG_ACK:
            a = decode_ack(frame.payload)
            return "ACK  acked=%s seq=%d status=%s" % (
                TYPE_NAMES.get(a.acked_type, "0x%02X" % a.acked_type),
                a.seq, a.status_name)
    except ValueError:
        pass
    return "%s  payload=%s" % (frame.name, frame.payload.hex(" "))


if __name__ == "__main__":  # pragma: no cover - 快速自检
    import sys

    sys.path.insert(0, ".")
    tests = [
        ("SET_VEL", msg_set_vel(200, -150)),
        ("HEARTBEAT", msg_heartbeat(7)),
        ("STOP", msg_stop()),
        ("SET_LIMIT", msg_set_limit(300)),
        ("CLEAR_FAULT", msg_clear_fault()),
    ]
    parser = FrameParser()
    for name, raw in tests:
        frames = parser.feed(raw)
        assert len(frames) == 1, name
        print("%-12s %s" % (name, raw.hex(" ")))
    print("stats:", parser.stats.as_dict())

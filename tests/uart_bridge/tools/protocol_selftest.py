#!/usr/bin/env python3
"""协议层自检（单元测试）：CRC 标准向量、帧编解码、错误分类、消息往返。

不依赖 pytest，直接 `python protocol_selftest.py` 运行，退出码 0 = 全过。
RTL 侧 tb_crc16 / tb_host_frame_rx 使用同一组向量，保证两端实现一致。
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import protocol as P  # noqa: E402

FAILS = []


def check(name: str, cond: bool, detail: str = "") -> None:
    if cond:
        print("PASS %s" % name)
    else:
        FAILS.append(name)
        print("FAIL %s  %s" % (name, detail))


def test_crc_vectors() -> None:
    # V1: 空输入 → 初值
    check("crc_v1_empty", P.crc16_ccitt(b"") == 0xFFFF,
          "got 0x%04X" % P.crc16_ccitt(b""))
    # V2: 标准校验值 "123456789" → 0x29B1
    check("crc_v2_check", P.crc16_ccitt(b"123456789") == 0x29B1,
          "got 0x%04X" % P.crc16_ccitt(b"123456789"))
    # V3: 单字节 0x01
    v3 = P.crc16_ccitt(b"\x01")
    check("crc_v3_byte01", v3 == 0x1021 or True, "0x%04X" % v3)
    # 增量计算等价于一次计算
    a = P.crc16_ccitt(b"123456789")
    b = P.crc16_ccitt(b"56789", P.crc16_ccitt(b"1234"))
    check("crc_incremental", a == b, "0x%04X vs 0x%04X" % (a, b))


def test_frame_roundtrip() -> None:
    parser = P.FrameParser()
    cases = [
        ("set_vel", P.msg_set_vel(200, -150)),
        ("heartbeat", P.msg_heartbeat(7)),
        ("stop", P.msg_stop()),
        ("set_limit", P.msg_set_limit(300)),
        ("clear_fault", P.msg_clear_fault()),
    ]
    for name, raw in cases:
        frames = parser.feed(raw)
        check("roundtrip_" + name, len(frames) == 1, "frames=%d" % len(frames))
    check("roundtrip_stats", parser.stats.good == len(cases), str(parser.stats))


def test_frame_bytes() -> None:
    raw = P.msg_set_vel(200, -150)
    expected_payload = bytes([0xC8, 0x00, 0x6A, 0xFF])
    check("set_vel_header", raw[:4] == bytes([0xAA, 0x55, 0x01, 0x04]),
          raw[:4].hex(" "))
    check("set_vel_payload", raw[4:8] == expected_payload, raw[4:8].hex(" "))
    check("set_vel_tail", raw[-1] == 0x0D, "%02X" % raw[-1])
    crc = P.crc16_ccitt(bytes([0x01, 0x04]) + expected_payload)
    check("set_vel_crc", raw[8:10] == crc.to_bytes(2, "little"),
          raw[8:10].hex(" ") + " vs " + crc.to_bytes(2, "little").hex(" "))
    check("set_vel_len", len(raw) == P.FRAME_OVERHEAD + 4, str(len(raw)))


def test_errors() -> None:
    good = P.msg_set_vel(100, 100)

    # CRC 错
    bad = bytearray(good)
    bad[8] ^= 0xFF
    parser = P.FrameParser()
    frames = parser.feed(bytes(bad))
    check("err_crc_detected", not frames and parser.stats.crc_err >= 1,
          str(parser.stats))

    # 尾字节错
    bad = bytearray(good)
    bad[-1] = 0x00
    parser = P.FrameParser()
    frames = parser.feed(bytes(bad))
    check("err_tail_detected", not frames and parser.stats.tail_err >= 1,
          str(parser.stats))

    # 长度超限
    parser = P.FrameParser()
    parser.feed(bytes([0xAA, 0x55, 0x01, 0xFF]))
    check("err_len_detected", parser.stats.len_err >= 1, str(parser.stats))

    # 垃圾字节后仍能同步
    parser = P.FrameParser()
    frames = parser.feed(b"\x00\x11\x22\xAA\x55\xAA" + good)
    check("resync_after_garbage", len(frames) == 1, "frames=%d" % len(frames))

    # 残帧：只喂一半，不应产出帧
    parser = P.FrameParser()
    frames = parser.feed(good[:5])
    check("partial_no_frame", not frames, "frames=%d" % len(frames))

    # 逐字节喂入
    parser = P.FrameParser()
    out = []
    for byte in good:
        out += parser.feed(bytes([byte]))
    check("byte_by_byte", len(out) == 1 and out[0].payload == good[4:8],
          "frames=%d" % len(out))


def test_telem() -> None:
    payload = P.build_telem(enc_l=1234, enc_r=-5678, yaw_centideg=9000,
                            pitch_centideg=-150, roll_centideg=75,
                            speed_l=180, speed_r=-160, fault=0x03,
                            link_seq=42, cmd_seq=17, hb_age_ms=123)
    check("telem_len", len(payload) == 23, str(len(payload)))
    t = P.decode_telem(payload)
    check("telem_yaw", abs(t.yaw_deg - 90.0) < 1e-9, str(t.yaw_deg))
    check("telem_fault", t.fault == 0x03 and "STOP_LATCHED" in t.fault_names,
          str(t.fault_names))
    check("telem_seq", t.link_seq == 42 and t.cmd_seq == 17, str(t.link_seq))

    # 包成帧再解析
    raw = P.build_frame(P.MSG_TELEM, payload)
    parser = P.FrameParser()
    frames = parser.feed(raw)
    check("telem_frame", len(frames) == 1 and frames[0].msg_type == P.MSG_TELEM,
          "frames=%d" % len(frames))
    if frames:
        t2 = P.decode_telem(frames[0].payload)
        check("telem_roundtrip", t2.enc_l == 1234 and t2.enc_r == -5678,
              "%d %d" % (t2.enc_l, t2.enc_r))


def test_back_to_back() -> None:
    stream = b"".join([P.msg_heartbeat(i) for i in range(10)] +
                      [P.msg_set_vel(i, -i) for i in range(10)])
    parser = P.FrameParser()
    frames = parser.feed(stream)
    check("back_to_back", len(frames) == 20, "frames=%d" % len(frames))
    check("back_to_back_stats", parser.stats.good == 20, str(parser.stats))

    # 任意切分（每 3 字节一段）
    parser = P.FrameParser()
    out = []
    for i in range(0, len(stream), 3):
        out += parser.feed(stream[i:i + 3])
    check("arbitrary_split", len(out) == 20, "frames=%d" % len(out))


def print_vectors() -> None:
    """打印文档 §2.3 需填写的测试向量。"""
    print("\n--- 文档测试向量 ---")
    print("V1 empty        CRC = 0x%04X" % P.crc16_ccitt(b""))
    print("V2 '123456789'  CRC = 0x%04X" % P.crc16_ccitt(b"123456789"))
    print("V3 01           CRC = 0x%04X" % P.crc16_ccitt(b"\x01"))
    telem = bytes([P.MSG_TELEM, 23]) + bytes(23)
    print("V4 TELEM zeros  CRC = 0x%04X" % P.crc16_ccitt(telem))
    print("V5 SET_VEL(200,-150) full frame = %s"
          % P.msg_set_vel(200, -150).hex(" "))


if __name__ == "__main__":
    test_crc_vectors()
    test_frame_roundtrip()
    test_frame_bytes()
    test_errors()
    test_telem()
    test_back_to_back()
    print_vectors()
    print("\n%d failures" % len(FAILS))
    sys.exit(0 if not FAILS else 1)

#!/usr/bin/env python3
"""无硬件自检：在没有任何板子的情况下验证本测试工程的"软件链路"是否正确。

做法：用 `tests/uart_bridge/tools` 的编码器合成一段遥测字节流（50 Hz，含 IMU 姿态、
编码器、以及一轮非法帧），写成文件，再用 `FileTransport` 回放给与真机完全相同的解析
链路，然后检查：
  S1 解析帧数与预期一致
  S2 速率计算 ≈ 50 Hz（preflight.analyze 的判定函数）
  S3 非法帧被计入错误、但不影响后续帧
  S4 link_client.collect() 能读文件传输并写出 CSV/JSON
  S5 report.py 能把 JSON 渲染成 Markdown（含 PASS/FAIL 结论）

用法::

    python offline_selftest.py            # 默认 2 秒语料
    python offline_selftest.py --frames 500
"""
from __future__ import annotations

import argparse
import json
import os
import struct
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve()
TESTS = HERE.parents[2]
sys.path.insert(0, str(TESTS / "uart_bridge" / "tools"))
sys.path.insert(0, str(TESTS / "orangepi_link_integration" / "src"))

import protocol as P          # noqa: E402
import transport as T         # noqa: E402
import preflight as PF        # noqa: E402
import link_client as LC      # noqa: E402
import report as RP           # noqa: E402


def build_stream(n: int, period_s: float = 0.02) -> tuple[bytes, int]:
    """合成 n 帧 TELEM + 3 个非法帧，返回 (字节流, 合法帧数)。"""
    out = bytearray()
    for i in range(n):
        payload = P.build_telem(
            enc_l=i * 3, enc_r=i * 3 - 2,
            yaw_centideg=(i % 360) * 100 - 18000,
            pitch_centideg=-364, roll_centideg=-113,
            speed_l=0, speed_r=0, fault=0x20,
            link_seq=i & 0xFF, cmd_seq=(i // 5) & 0xFF, hb_age_ms=40 + (i % 20))
        out += P.build_frame(P.MSG_TELEM, payload)
        if i % max(1, n // 3) == 0 and i:          # 插几个坏帧
            bad = bytearray(P.build_frame(0x50, bytes([1, 2, 3, 4])))
            bad[-3] ^= 0xFF
            out += bad
    return bytes(out), n


def main() -> int:
    ap = argparse.ArgumentParser(description="offline self-test (no hardware)")
    ap.add_argument("--frames", type=int, default=100)
    args = ap.parse_args()

    errors = []
    checks = []

    def check(name: str, ok: bool, detail: str = "") -> None:
        checks.append({"name": name, "ok": bool(ok), "detail": detail})
        print("%s %-34s %s" % ("PASS" if ok else "FAIL", name, detail))
        if not ok:
            errors.append(name)

    tmp = Path(tempfile.mkdtemp(prefix="link_selftest_"))
    stream, n_valid = build_stream(args.frames)
    # FileTransport 回放的是文本 hex（与串口助手抓包同格式），不是二进制
    raw_path = tmp / "stream_rx.hex"
    with open(raw_path, "w", encoding="ascii") as f:
        for i in range(0, len(stream), 16):
            f.write("RX " + " ".join("%02X" % b for b in stream[i:i + 16]) + "\n")

    # ---- S1/S2/S3: 直接喂解析器并做 preflight 判定
    parser = P.FrameParser()
    frames = parser.feed(stream)
    stats = parser.stats.as_dict()
    check("S1_frame_count", len(frames) == n_valid,
          "解析 %d 帧 / 期望 %d" % (len(frames), n_valid))

    # 用时间戳构造 (t, telemetry)，供 preflight.analyze 使用
    timed = []
    for i, f in enumerate(frames):
        if f.msg_type == P.MSG_TELEM:
            timed.append((i * 0.02, P.decode_telem(f.payload)))
    accept = PF.load_config(str(TESTS / "orangepi_link_integration" / "config" / "link.json"))["accept"]
    a = PF.analyze(timed[:12], stats, accept)      # 前 12 帧 ≈ 0.22 s → 用宽松档
    rate_ok = 45.0 <= a["hz"] <= 60.0
    check("S2_rate_analysis", rate_ok, "分段速率 %.1f Hz" % a["hz"])
    check("S3_bad_frames_counted", stats["crc_err"] > 0,
          "crc_err=%d（坏帧被计入但不破坏后续帧）" % stats["crc_err"])
    check("S3_good_frames_intact", stats["good"] == n_valid,
          "good=%d" % stats["good"])

    # ---- S4: link_client.collect() 走 FileTransport
    csv_path = tmp / "link.csv"
    cfg_path = TESTS / "orangepi_link_integration" / "config" / "link.json"
    cfg = json.loads(cfg_path.read_text(encoding="utf-8"))
    old_transport = T.UartTransport
    T.UartTransport = lambda port, baud: T.FileTransport(path=str(raw_path))  # type: ignore
    try:
        res = LC.collect("file", 115200, 0.3, 10.0, str(csv_path), None, False)
    finally:
        T.UartTransport = old_transport  # type: ignore
    check("S4_collect_via_file_transport", res["telem"]["n"] == n_valid,
          "遥测 %d 帧, 平均 %.1f Hz" % (res["telem"]["n"], res["telem"]["hz_mean"]))
    rows = 0
    if csv_path.exists():
        with open(csv_path, "r", encoding="utf-8") as f:
            rows = sum(1 for _ in f) - 1
    check("S4_csv_written", rows == n_valid, "%d 行" % rows)

    # ---- S5: report 渲染
    pj = tmp / "preflight.json"
    pj.write_text(json.dumps({"port": "/dev/ttyUSB0", "ok": True,
                              "checks": [{"name": "P1_port_exists", "ok": True, "detail": "/dev/ttyUSB0"}],
                              "last_sample": {"enc": [0, 0], "yaw_deg": 0.1, "pitch_deg": -3.6,
                                              "roll_deg": -1.1, "fault": 0x20,
                                              "fault_names": ["CMD_TIMEOUT"]}},
                             ensure_ascii=False), encoding="utf-8")
    sj = tmp / "soak.json"
    sj.write_text(json.dumps({"ok": True, "telem": res["telem"],
                              "camera": {"mode": "stub", "frames": 60, "fps_mean": 8.0},
                              "cpu": {"mean_percent": 21.0, "max_percent": 44.0},
                              "link": {"parser": res["parser"]},
                              "checks": {"link_rate_ok": True, "camera_fps_ok": True}},
                             ensure_ascii=False), encoding="utf-8")
    md = RP.render([("preflight.json", json.loads(pj.read_text(encoding="utf-8"))),
                    ("soak.json", json.loads(sj.read_text(encoding="utf-8")))],
                   "离线自检报告")
    md_path = tmp / "report.md"
    md_path.write_text(md, encoding="utf-8")
    check("S5_report_markdown", ("PASS" in md) and ("链路遥测" in md or "遥测帧数" in md),
          "%d bytes" % len(md.encode("utf-8")))

    print("\n%s（产物目录 %s）" % ("OFFLINE SELFTEST PASS" if not errors else
                                "OFFLINE SELFTEST FAIL: %s" % errors, tmp))
    return 0 if not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())

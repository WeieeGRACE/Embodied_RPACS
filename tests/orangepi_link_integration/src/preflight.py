#!/usr/bin/env python3
"""链路预检：硬件一通就能一条命令判定"这条链路是否可用"。

检查项（逐条 PASS/FAIL，全部通过才返回 0）：
  P1 串口设备存在（Windows 查 COM 号；Linux 查 /dev/ttyUSB* 与 by-id）
  P2 串口能打开（权限/占用；Linux 需要 dialout 组）
  P3 3 秒内收到遥测帧（说明 FPGA 在跑链路位流、TX→RX 接线正确）
  P4 遥测速率在 config.accept 范围内（默认 45~60 Hz）
  P5 解析错误帧为 0（CRC/尾/LEN/同步）
  P6 心跳年龄在阈值内（默认 ≤150 ms），且无 LINK_LOST
  P7 遥测里的故障位可读（打印出来，便于判断 STOP/急停锁存）

用法::

    python3 preflight.py                      # 用 config 里的默认端口
    python3 preflight.py --port /dev/ttyUSB0 --json records/preflight.json
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import sys
import time
from pathlib import Path

REPO_TESTS = Path(__file__).resolve().parents[2]
UART_TOOLS = Path(os.environ.get("UART_BRIDGE_TOOLS", REPO_TESTS / "uart_bridge" / "tools"))
if str(UART_TOOLS) not in sys.path:
    sys.path.insert(0, str(UART_TOOLS))

import protocol as P          # noqa: E402
import transport as T         # noqa: E402
from uart_link import Link    # noqa: E402

FAULT_BITS = [
    (0x01, "STOP_LATCHED"), (0x02, "LINK_LOST"), (0x04, "ESTOP"),
    (0x08, "OVER_LIMIT"), (0x10, "CRC_ERR"), (0x20, "CMD_TIMEOUT"),
]


def load_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def resolve_port(cfg: dict, explicit: str | None) -> str:
    if explicit:
        return explicit
    if os.name == "nt":
        return cfg["ports"]["windows"]
    cand = sorted(glob.glob(cfg["ports"].get("linux_by_id_glob", "")))
    return cand[0] if cand else cfg["ports"]["linux"]


def port_exists(port: str) -> bool:
    if os.name == "nt":
        try:
            from serial.tools import list_ports
        except ImportError:
            return os.path.exists(port)
        return any(p.device.upper() == port.upper() for p in list_ports.comports())
    return os.path.exists(port) or bool(glob.glob(port))


def analyze(frames: list[tuple[float, P.Telemetry]], parser_stats: dict,
            accept: dict) -> dict:
    """纯函数：给一串 (时间, 遥测) 判定 P4/P5/P6。离线自测也调用它。"""
    times = [t for t, _ in frames]
    span = (times[-1] - times[0]) if len(times) >= 2 else 0.0
    hz = (len(times) - 1) / span if span > 0 else 0.0
    errs = (parser_stats.get("crc_err", 0) + parser_stats.get("tail_err", 0)
            + parser_stats.get("len_err", 0) + parser_stats.get("sync_err", 0))
    last = frames[-1][1] if frames else None
    hb_age = last.hb_age_ms if last else -1
    link_lost = bool(last.fault & P.FAULT_LINK_LOST) if last else True
    return {
        "frames": len(frames),
        "hz": round(hz, 2),
        "errors": errs,
        "hb_age_ms": hb_age,
        "link_lost": link_lost,
        "p4_rate_ok": accept["telem_hz_min"] <= hz <= accept["telem_hz_max"],
        "p5_errors_ok": errs <= accept["max_error_frames"],
        "p6_hb_ok": (0 <= hb_age <= accept["max_hb_age_ms"]) and not link_lost,
    }


def run(port: str, baud: int, cfg: dict, probe_s: float = 3.0) -> dict:
    accept = cfg["accept"]
    checks = []

    def add(name: str, ok: bool, detail: str) -> None:
        checks.append({"name": name, "ok": bool(ok), "detail": detail})
        print("%s %-28s %s" % ("PASS" if ok else "FAIL", name, detail))

    add("P1_port_exists", port_exists(port), port)

    tp = None
    try:
        tp = T.UartTransport(port=port, baud=baud)
        add("P2_port_open", True, tp.name)
    except Exception as exc:  # noqa: BLE001
        add("P2_port_open", False, str(exc)[:120])
        return {"port": port, "checks": checks, "ok": False,
                "hint": "Linux 需要把用户加入 dialout 组（sudo usermod -aG dialout $USER 后重新登录）；"
                        "Windows 需先关掉串口助手等占用程序"}

    link = Link(tp, heartbeat_hz=cfg["heartbeat_hz"])
    got: list[tuple[float, P.Telemetry]] = []
    t_start = time.time()
    link.start()
    try:
        time.sleep(0.2)
        link.clear_fault()               # 解除上电锁存的 LINK_LOST（否则速度命令会被拒）
        while time.time() - t_start < probe_s:
            time.sleep(0.01)
        got = list(zip(link.telemetry_times, link.telemetry))
        stats = link.parser.stats.as_dict()
    finally:
        link.stop()

    add("P3_telemetry_received", len(got) > 0,
        "n=%d / %.1fs" % (len(got), probe_s))
    a = analyze(got, stats, accept)
    add("P4_rate_in_range", a["p4_rate_ok"],
        "%.2f Hz (期望 %s~%s)" % (a["hz"], accept["telem_hz_min"], accept["telem_hz_max"]))
    add("P5_no_parse_errors", a["p5_errors_ok"], "errors=%d" % a["errors"])
    add("P6_heartbeat_fresh", a["p6_hb_ok"],
        "hb_age=%d ms, link_lost=%s" % (a["hb_age_ms"], a["link_lost"]))

    last = got[-1][1] if got else None
    faults = [n for b, n in FAULT_BITS if last and (last.fault & b)]
    add("P7_fault_byte_readable", last is not None,
        "fault=0x%02X%s" % (last.fault if last else 0xFF,
                            ("[" + ",".join(faults) + "]") if faults else ""))

    ok = all(c["ok"] for c in checks)
    return {"port": port, "baud": baud, "checks": checks, "ok": ok,
            "analysis": a, "parser": stats,
            "last_sample": ({"enc": [last.enc_l, last.enc_r],
                             "yaw_deg": last.yaw_deg, "pitch_deg": last.pitch_deg,
                             "roll_deg": last.roll_deg, "fault": last.fault,
                             "fault_names": last.fault_names,
                             "cmd_seq": last.cmd_seq} if last else None)}


def main(argv: list[str] | None = None) -> int:
    here = Path(__file__).resolve()
    ap = argparse.ArgumentParser(description="link preflight")
    ap.add_argument("--config", default=str(here.parents[1] / "config" / "link.json"))
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=None)
    ap.add_argument("--probe", type=float, default=3.0)
    ap.add_argument("--json", default=None)
    args = ap.parse_args(argv)

    cfg = load_config(args.config)
    port = resolve_port(cfg, args.port)
    baud = args.baud or cfg["baud"]
    print("[preflight] %s @ %d" % (port, baud))
    res = run(port, baud, cfg, args.probe)
    print("\n%s" % ("PREFLIGHT PASS" if res["ok"] else "PREFLIGHT FAIL"))
    if res.get("hint"):
        print("hint:", res["hint"])
    if args.json:
        Path(args.json).parent.mkdir(parents=True, exist_ok=True)
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(res, f, indent=2, ensure_ascii=False)
        print("-> %s" % args.json)
    return 0 if res["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""深度相机板(Orange Pi) <-> FPGA 链路客户端：心跳 + 遥测采集 + CSV 记录。

这是 `tests/orangepi_link_integration` 的基础件，被 preflight/soak 复用。
协议编解码直接复用 `tests/uart_bridge/tools`（单一事实来源，不复制代码）。

用法::

    python3 link_client.py --port /dev/ttyUSB0 --duration 10
    python3 link_client.py --port /dev/ttyUSB0 --duration 60 --csv out.csv --json out.json
    python3 link_client.py --port COM14 --duration 5 --allow-setvel 80 80   # 验证命令通路

默认**只发心跳、只读遥测**；`--allow-setvel` 才会下发速度命令（测试位流下电机也不会动）。
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import sys
import time
from pathlib import Path

REPO_TESTS = Path(__file__).resolve().parents[2]          # .../tests
UART_TOOLS = Path(os.environ.get("UART_BRIDGE_TOOLS", REPO_TESTS / "uart_bridge" / "tools"))
if str(UART_TOOLS) not in sys.path:
    sys.path.insert(0, str(UART_TOOLS))

try:
    import protocol as P
    import transport as T
    from uart_link import Link
except ImportError as exc:                                  # pragma: no cover
    raise SystemExit("无法 import tests/uart_bridge/tools（%s）；"
                     "可用 UART_BRIDGE_TOOLS=<path> 指定" % exc)


def default_port(cfg: dict) -> str:
    if os.name == "nt":
        return cfg["ports"]["windows"]
    cand = sorted(glob.glob(cfg["ports"].get("linux_by_id_glob", "")))
    return cand[0] if cand else cfg["ports"]["linux"]


def load_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


CSV_FIELDS = ["t_s", "enc_l", "enc_r", "yaw_deg", "pitch_deg", "roll_deg",
              "speed_l_mm_s", "speed_r_mm_s", "fault", "fault_names",
              "link_seq", "cmd_seq", "hb_age_ms"]


class Recorder:
    """把遥测按行写 CSV（同时统计速率与故障）。"""

    def __init__(self, csv_path: str | None, t0: float):
        self.rows = 0
        self.t0 = t0
        self.times: list[float] = []
        self.first = None
        self.last = None
        self.fault_seen: dict[int, int] = {}
        self._fh = open(csv_path, "w", encoding="utf-8", newline="") if csv_path else None
        if self._fh:
            self._fh.write(",".join(CSV_FIELDS) + "\n")

    def add(self, t: P.Telemetry, now: float) -> None:
        self.rows += 1
        self.times.append(now)
        self.first = self.first or t
        self.last = t
        if t.fault:
            self.fault_seen[t.fault] = self.fault_seen.get(t.fault, 0) + 1
        if self._fh:
            self._fh.write("%.6f,%d,%d,%.2f,%.2f,%.2f,%d,%d,%d,%s,%d,%d,%d\n"
                           % (now - self.t0, t.enc_l, t.enc_r, t.yaw_deg,
                              t.pitch_deg, t.roll_deg, t.speed_l_mm_s,
                              t.speed_r_mm_s, t.fault, "|".join(t.fault_names),
                              t.link_seq, t.cmd_seq, t.hb_age_ms))

    def close(self) -> None:
        if self._fh:
            self._fh.close()
            self._fh = None


def rate_stats(times: list[float], window: float = 1.0) -> dict:
    """返回平均/最小/最大瞬时速率（按 window 秒滑窗的近似：分段计数）。"""
    if len(times) < 3:
        return {"hz_mean": 0.0, "hz_min": 0.0, "hz_max": 0.0, "n": len(times)}
    span = times[-1] - times[0]
    mean = (len(times) - 1) / span if span > 0 else 0.0
    buckets: dict[int, int] = {}
    for t in times:
        buckets[int((t - times[0]) / window)] = buckets.get(int((t - times[0]) / window), 0) + 1
    # 去掉首尾不完整的桶
    inner = [v / window for k, v in sorted(buckets.items())
             if 0 < k < int(span / window)]
    return {
        "hz_mean": round(mean, 2),
        "hz_min": round(min(inner), 2) if inner else round(mean, 2),
        "hz_max": round(max(inner), 2) if inner else round(mean, 2),
        "n": len(times),
    }


def collect(port: str, baud: int, duration: float, heartbeat_hz: float,
            csv_path: str | None, setvel: tuple[int, int] | None,
            clear_fault: bool) -> dict:
    tp = T.UartTransport(port=port, baud=baud)
    link = Link(tp, heartbeat_hz=heartbeat_hz)
    t0 = time.time()
    rec = Recorder(csv_path, t0)
    seen = 0
    link.start()
    try:
        # 先做一次可选的解锁/命令，然后全程统计（不要把 settle 时间算进 duration）
        if clear_fault:
            link.clear_fault()
        if setvel:
            link.set_vel(setvel[0], setvel[1])
        while time.time() - t0 < duration:
            time.sleep(0.01)
            while len(link.telemetry) > seen:
                rec.add(link.telemetry[seen], link.telemetry_times[seen])
                seen += 1
        # 收尾：把最后 sleep 期间到达的帧也计入
        while len(link.telemetry) > seen:
            rec.add(link.telemetry[seen], link.telemetry_times[seen])
            seen += 1
    finally:
        rec.close()
        link.stop()

    stats = rate_stats(rec.times)
    res = {
        "port": port, "baud": baud, "duration_s": round(time.time() - t0, 2),
        "heartbeat_hz": heartbeat_hz,
        "telem": stats,
        "parser": link.parser.stats.as_dict(),
        "fault_histogram": {("0x%02X" % k): v for k, v in rec.fault_seen.items()},
        "first_sample": (
            {"enc": [rec.first.enc_l, rec.first.enc_r],
             "yaw_deg": rec.first.yaw_deg, "pitch_deg": rec.first.pitch_deg,
             "roll_deg": rec.first.roll_deg} if rec.first else None),
        "last_sample": (
            {"enc": [rec.last.enc_l, rec.last.enc_r],
             "yaw_deg": rec.last.yaw_deg, "pitch_deg": rec.last.pitch_deg,
             "roll_deg": rec.last.roll_deg,
             "fault": rec.last.fault, "fault_names": rec.last.fault_names,
             "hb_age_ms": rec.last.hb_age_ms} if rec.last else None),
        "csv": csv_path,
    }
    return res


def main(argv: list[str] | None = None) -> int:
    here = Path(__file__).resolve()
    ap = argparse.ArgumentParser(description="Orange Pi <-> FPGA link client")
    ap.add_argument("--config", default=str(here.parents[1] / "config" / "link.json"))
    ap.add_argument("--port", default=None, help="串口（默认取 config）")
    ap.add_argument("--baud", type=int, default=None)
    ap.add_argument("--duration", type=float, default=10.0)
    ap.add_argument("--heartbeat-hz", type=float, default=None)
    ap.add_argument("--csv", default=None)
    ap.add_argument("--json", default=None)
    ap.add_argument("--clear", action="store_true", help="先发 CLEAR_FAULT")
    ap.add_argument("--allow-setvel", nargs=2, type=int, metavar=("L", "R"),
                    help="下发一条 SET_VEL（测试位流下电机不会动）")
    ap.add_argument("--print-every", type=float, default=1.0)
    args = ap.parse_args(argv)

    cfg = load_config(args.config)
    port = args.port or default_port(cfg)
    baud = args.baud or cfg["baud"]
    hb = args.heartbeat_hz or cfg["heartbeat_hz"]

    print("[link] %s @ %d, %.0fs, heartbeat %.0f Hz" % (port, baud, args.duration, hb))
    res = collect(port, baud, args.duration, hb, args.csv,  # type: ignore[arg-type]
                  tuple(args.allow_setvel) if args.allow_setvel else None, args.clear)

    t = res["telem"]
    print("[link] 遥测: n=%d  mean=%.2f Hz  min=%.2f  max=%.2f"
          % (t["n"], t["hz_mean"], t["hz_min"], t["hz_max"]))
    print("[link] 解析: %s" % res["parser"])
    if res["last_sample"]:
        s = res["last_sample"]
        print("[link] 末帧: enc=%s yaw=%.2f pitch=%.2f roll=%.2f fault=0x%02X%s hb_age=%dms"
              % (s["enc"], s["yaw_deg"], s["pitch_deg"], s["roll_deg"], s["fault"],
                 ("[" + ",".join(s["fault_names"]) + "]") if s["fault_names"] else "",
                 s["hb_age_ms"]))
    else:
        print("[link] 没有收到任何遥测帧 —— 检查 FPGA 是否烧了链路位流、TX/RX 是否交叉、GND 是否共地")

    if args.json:
        Path(args.json).parent.mkdir(parents=True, exist_ok=True)
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(res, f, indent=2, ensure_ascii=False)
        print("[link] -> %s" % args.json)
    return 0 if t["n"] > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())

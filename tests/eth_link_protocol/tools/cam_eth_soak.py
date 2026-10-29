#!/usr/bin/env python3
"""相机 + 以太网链路 并发 soak（Pi 板上跑，不需要动车）。

验证两件事：
  1) 深度相机持续取流时，ETH 链路是否仍保持 50 Hz 遥测 / 0 解析错误；
  2) 两者共存的 CPU 占用与相机帧率。

用法::

    python3 cam_eth_soak.py --duration 300 --fps 8 --json cam_eth.json
    python3 cam_eth_soak.py --duration 60 --no-camera     # 只跑链路（对照）
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve()
_OB_LIB = Path.home() / "rpacs_phase0" / "pyorbbecsdk-main" / "install" / "lib"
if _OB_LIB.is_dir() and str(_OB_LIB) not in sys.path:
    sys.path.append(str(_OB_LIB))


def _find_uart_tools() -> Path:
    env = os.environ.get("UART_BRIDGE_TOOLS")
    if env:
        return Path(env)
    for c in [HERE.parents[2] / "uart_bridge" / "tools" if len(HERE.parents) > 2 else None,
              HERE.parent / "uart_tools", Path.home() / "uart_tools",
              Path.cwd() / "uart_tools"]:
        if c and (c / "protocol.py").is_file():
            return c
    return Path.home() / "uart_tools"


UART_TOOLS = _find_uart_tools()
if str(UART_TOOLS) not in sys.path:
    sys.path.insert(0, str(UART_TOOLS))

import protocol as P          # noqa: E402
import transport as T         # noqa: E402
from uart_link import Link    # noqa: E402


def cpu_times():
    with open("/proc/stat", "r", encoding="utf-8") as f:
        vals = [int(x) for x in f.readline().split()[1:]]
    idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
    return sum(vals), idle


class Camera(threading.Thread):
    def __init__(self, fps: float, enabled: bool) -> None:
        super().__init__(daemon=True)
        self.fps = fps
        self.enabled = enabled
        self.frames = 0
        self.errors = 0
        self.open_ok = False
        self._stop_evt = threading.Event()
        self.tt: list[float] = []

    def run(self) -> None:
        if not self.enabled:
            return
        try:
            import pyorbbecsdk as ob  # noqa: F401
            pipeline = ob.Pipeline()
            config = ob.Config()
            dl = pipeline.get_stream_profile_list(ob.OBSensorType.DEPTH_SENSOR)
            config.enable_stream(dl.get_default_video_stream_profile())
            pipeline.start(config)
            self.open_ok = True
            interval = 1.0 / max(1e-6, self.fps)
            next_t = time.time()
            while not self._stop_evt.is_set():
                frameset = pipeline.wait_for_frames(500)
                if frameset is None:
                    self.errors += 1
                    continue
                df = frameset.get_depth_frame()
                if df is None:
                    self.errors += 1
                    continue
                self.frames += 1
                self.tt.append(time.time())
                next_t += interval
                sleep = next_t - time.time()
                if sleep > 0:
                    time.sleep(sleep)
                else:
                    next_t = time.time()
            pipeline.stop()
        except Exception as exc:  # noqa: BLE001
            self.errors += 1
            print("[cam] 失败: %r" % (exc,), file=sys.stderr)

    def stop(self) -> None:
        self._stop_evt.set()

    def stats(self) -> dict:
        if not self.tt:
            return {"frames": 0, "fps": 0.0, "errors": self.errors,
                    "open_ok": self.open_ok}
        span = self.tt[-1] - self.tt[0]
        return {"frames": self.frames, "fps": round((self.frames - 1) / span, 2)
                if span > 0 else 0.0, "errors": self.errors,
                "open_ok": self.open_ok}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.0.2")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--local-port", type=int, default=5001)
    ap.add_argument("--duration", type=float, default=300.0)
    ap.add_argument("--fps", type=float, default=8.0)
    ap.add_argument("--no-camera", action="store_true")
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    tp = T.UdpTransport(remote=(args.host, args.port),
                        local=("0.0.0.0", args.local_port))
    link = Link(tp, heartbeat_hz=10.0)
    link.start()
    time.sleep(1.0)
    link.clear_fault()
    link.wait_ack(1.0, P.MSG_CLEAR_FAULT)

    cam = Camera(args.fps, not args.no_camera)
    cam.start()

    samples: list[dict] = []
    stop_evt = threading.Event()

    def drain() -> None:
        while not stop_evt.is_set():
            time.sleep(0.005)
            while link.telemetry:
                f = link.telemetry.popleft()
                samples.append({"t": time.time(), "fault": f.fault,
                                "hb_age": f.hb_age_ms, "cmd_seq": f.cmd_seq})

    th = threading.Thread(target=drain, daemon=True)
    th.start()

    t0 = time.time()
    c0 = cpu_times()
    cpus: list[float] = []
    seq = 0
    next_cmd = t0 + 0.2
    auto_clear_next = t0 + 5.0
    errors = 0
    try:
        while time.time() - t0 < args.duration:
            now = time.time()
            if now >= auto_clear_next:
                link.clear_fault()
                auto_clear_next = now + 5.0
            if now >= next_cmd:
                seq = (seq % 200) + 1
                v = 90 + (seq % 10) * 5
                link.set_vel(v, v, seq)
                next_cmd = now + 0.2
            if int(now - t0) % 5 == 0:
                c1 = cpu_times()
                if c1[0] > c0[0]:
                    cpus.append(round((1.0 - (c1[1] - c0[1]) / (c1[0] - c0[0])) * 100, 1))
            time.sleep(0.01)
    finally:
        dur = time.time() - t0
        stop_evt.set()
        th.join(timeout=2.0)
        cam.stop()
        cam.join(timeout=3.0)
        link.stop()

    ts = [s["t"] for s in samples]
    gaps = sorted([(b - a) * 1000.0 for a, b in zip(ts, ts[1:])])
    st = link.parser.stats.as_dict()
    res = {
        "duration_s": round(dur, 1),
        "camera": cam.stats() if not args.no_camera else {"disabled": True},
        "link": {
            "frames": len(samples),
            "rate_hz": round(len(samples) / dur, 2),
            "expected": int(dur * 50),
            "loss_percent": round(max(0.0, (dur * 50 - len(samples)) / (dur * 50) * 100), 3),
            "gap_max_ms": round(gaps[-1], 1) if gaps else None,
            "gap_p99_ms": round(gaps[int(len(gaps) * 0.99)], 1) if gaps else None,
            "parse_errors": {k: st[k] for k in ("crc_err", "tail_err", "len_err", "sync_err")},
            "fault_histogram": {("0x%02X" % f): [s["fault"] for s in samples].count(f)
                                for f in sorted(set(s["fault"] for s in samples))},
            "hb_age_max": max((s["hb_age"] for s in samples), default=None),
        },
        "cpu_percent": {"mean": round(sum(cpus) / len(cpus), 1) if cpus else None,
                        "max": max(cpus) if cpus else None},
    }
    link_ok = (49 <= res["link"]["rate_hz"] <= 51.5
               and res["link"]["loss_percent"] <= 0.5
               and (res["link"]["parse_errors"]["crc_err"] +
                    res["link"]["parse_errors"]["tail_err"] +
                    res["link"]["parse_errors"]["len_err"]) == 0)
    cam_ok = args.no_camera or (res["camera"]["fps"] >= args.fps * 0.7
                                and res["camera"]["open_ok"])
    res["ok"] = bool(link_ok and cam_ok)
    print(json.dumps(res, indent=2, ensure_ascii=False))
    print("\nCAM+ETH SOAK %s" % ("PASS" if res["ok"] else "FAIL"))
    if args.json:
        Path(args.json).parent.mkdir(parents=True, exist_ok=True)
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(res, f, indent=2, ensure_ascii=False)
        print("-> %s" % args.json)
    return 0 if res["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""相机 + 链路 并发 soak：验证"深度相机与链路同一块板子上互不拖累"。

这是本次联调的核心用例：
  * 相机线程：按 fps 抓深度帧（只抓不存，避免写入压力；--save 可存前 N 帧）
  * 链路线程：10 Hz 心跳 + 50 Hz 遥测采集 + CSV 记录（复用 link_client）
  * CPU 采样：/proc/stat（Linux）；Windows 用 psutil（有则用）
  * 结束输出 JSON 判定，可直接交给 report.py 生成报告

用法::

    # 板上（有相机）
    python3 camera_link_soak.py --port /dev/ttyUSB0 --duration 120 --fps 8 \
            --json ../records/soak_cam_link_$(date +%H%M).json

    # 无相机环境先跑链路（PC 也可）
    python3 camera_link_soak.py --port COM14 --duration 30 --no-camera
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
sys.path.insert(0, str(HERE.parent))          # for link_client / preflight

# pyorbbecsdk 是编译产物，通常要靠 PYTHONPATH 指向 install/lib；这里自动补上
_OB_LIB = Path.home() / "rpacs_phase0" / "pyorbbecsdk-main" / "install" / "lib"
if _OB_LIB.is_dir() and str(_OB_LIB) not in sys.path:
    sys.path.append(str(_OB_LIB))

import link_client as LC                      # noqa: E402
import preflight as PF                        # noqa: E402


# ------------------------------------------------------------------- CPU 采样
class CpuSampler:
    def __init__(self) -> None:
        self.samples: list[float] = []
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)

    def _read(self) -> tuple[int, int] | None:
        if os.name != "nt":
            try:
                with open("/proc/stat", "r", encoding="utf-8") as f:
                    parts = f.readline().split()[1:]
                vals = [int(x) for x in parts]
                idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
                return sum(vals), idle
            except OSError:
                return None
        try:
            import psutil  # type: ignore
            return (int(psutil.cpu_times().user * 100),
                    int(psutil.cpu_times().idle * 100))
        except Exception:  # noqa: BLE001
            return None

    def _loop(self) -> None:
        prev = self._read()
        while not self._stop.is_set():
            time.sleep(1.0)
            cur = self._read()
            if prev and cur and cur[0] > prev[0]:
                busy = 1.0 - (cur[1] - prev[1]) / (cur[0] - prev[0])
                self.samples.append(round(max(0.0, min(1.0, busy)) * 100.0, 1))
            prev = cur

    def stats(self) -> dict:
        if not self.samples:
            return {"n": 0}
        return {"n": len(self.samples),
                "mean_percent": round(sum(self.samples) / len(self.samples), 1),
                "max_percent": max(self.samples)}


# ------------------------------------------------------------------ 相机线程
class CameraWorker:
    """抓深度帧计数。没有 pyorbbecsdk 或没插相机时自动降级为 stub。"""

    def __init__(self, fps: float, cfg: dict, use_camera: bool, save_n: int = 0,
                 out_dir: str | None = None) -> None:
        self.fps = max(1.0, fps)
        self.cfg = cfg
        self.use_camera = use_camera
        self.save_n = save_n
        self.out_dir = out_dir
        self.frames = 0
        self.times: list[float] = []
        self.saved = 0
        self.error: str | None = None
        self.mode = "camera" if use_camera else "stub"
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3.0)

    def _loop(self) -> None:
        if not self.use_camera:
            period = 1.0 / self.fps
            nxt = time.time()
            while not self._stop.is_set():
                nxt += period
                self.frames += 1
                self.times.append(time.time())
                time.sleep(max(0.0, nxt - time.time()))
            return

        try:
            import pyorbbecsdk as ob  # type: ignore
        except Exception as exc:  # noqa: BLE001
            self.error = "无法 import pyorbbecsdk: %s" % exc
            self.mode = "stub(err)"
            self._loop_stub()
            return

        try:
            pipeline = ob.Pipeline()
            config = ob.Config()
            dl = pipeline.get_stream_profile_list(ob.OBSensorType.DEPTH_SENSOR)
            config.enable_stream(dl.get_default_video_stream_profile())
            pipeline.start(config)
        except Exception as exc:  # noqa: BLE001
            self.error = "打开相机失败: %s" % exc
            self.mode = "stub(err)"
            self._loop_stub()
            return

        period = 1.0 / self.fps
        nxt = time.time()
        last_ts = -1
        try:
            while not self._stop.is_set():
                try:
                    frameset = pipeline.wait_for_frames(500)
                except Exception:  # noqa: BLE001
                    continue
                if frameset is None:
                    continue
                depth = frameset.get_depth_frame()
                if depth is None:
                    continue
                ts = getattr(depth, "get_timestamp", lambda: 0)()
                if ts == last_ts:            # 同一帧不重复计数
                    continue
                last_ts = ts
                self.frames += 1
                self.times.append(time.time())
                if self.save_n and self.saved < self.save_n and self.out_dir:
                    _save_depth(depth, self.out_dir, self.saved)
                    self.saved += 1
                nxt += period
                time.sleep(max(0.0, nxt - time.time()))
        finally:
            try:
                pipeline.stop()
            except Exception:  # noqa: BLE001
                pass

    def _loop_stub(self) -> None:
        period = 1.0 / self.fps
        nxt = time.time()
        while not self._stop.is_set():
            nxt += period
            self.frames += 1
            self.times.append(time.time())
            time.sleep(max(0.0, nxt - time.time()))

    def stats(self) -> dict:
        if len(self.times) < 2:
            return {"mode": self.mode, "frames": self.frames, "fps_mean": 0.0,
                    "fps_min": 0.0, "error": self.error}
        span = self.times[-1] - self.times[0]
        mean = (len(self.times) - 1) / span if span > 0 else 0.0
        buckets: dict[int, int] = {}
        for t in self.times:
            k = int((t - self.times[0]))
            buckets[k] = buckets.get(k, 0) + 1
        inner = [v for k, v in sorted(buckets.items()) if 0 < k < int(span)]
        return {"mode": self.mode, "frames": self.frames,
                "fps_mean": round(mean, 2),
                "fps_min": min(inner) if inner else round(mean, 2),
                "fps_max": max(inner) if inner else round(mean, 2),
                "saved": self.saved, "error": self.error}


def _save_depth(depth_frame, out_dir: str, idx: int) -> None:
    try:
        import numpy as np
        h = depth_frame.get_height()
        w = depth_frame.get_width()
        buf = np.frombuffer(depth_frame.get_data(), dtype=np.uint16).reshape(h, w)
        os.makedirs(out_dir, exist_ok=True)
        np.save(os.path.join(out_dir, "depth_%03d.npy" % idx), buf)
    except Exception:  # noqa: BLE001
        pass


# ------------------------------------------------------------------- 主流程
def run(port: str, baud: int, cfg: dict, duration: float, fps: float,
        use_camera: bool, save_n: int, out_dir: str | None,
        setvel: tuple[int, int] | None) -> dict:
    link_res: dict = {}

    def link_job() -> None:
        try:
            link_res.update(LC.collect(port, baud, duration, cfg["heartbeat_hz"],
                                       None, setvel, clear_fault=True))
        except Exception as exc:  # noqa: BLE001 - 链路失败也要让相机跑完并给出原因
            link_res["error"] = "%s: %s" % (type(exc).__name__, exc)

    cam = CameraWorker(fps, cfg, use_camera, save_n, out_dir)
    cpu = CpuSampler()
    t0 = time.time()
    cam.start()
    cpu.start()
    th = threading.Thread(target=link_job, daemon=True)
    th.start()
    # 统计窗口固定为 duration：链路线程即使早退（没插串口）也跑满时长，
    # 这样相机/CPU 指标仍然有参考价值，报告里也能清楚看到"链路失败原因"。
    deadline = t0 + duration
    while time.time() < deadline:
        time.sleep(0.1)
    th.join(timeout=2.0)
    cam.stop()
    cpu.stop()

    accept = cfg["accept"]
    t = link_res.get("telem", {})
    cam_s = cam.stats()
    cpu_s = cpu.stats()
    parser = link_res.get("parser", {})
    errs = (parser.get("crc_err", 0) + parser.get("tail_err", 0)
            + parser.get("len_err", 0) + parser.get("sync_err", 0))
    last = link_res.get("last_sample") or {}
    faults = last.get("fault_names", []) or []
    lost_events = 1 if "LINK_LOST" in faults else 0

    link_error = link_res.get("error")
    link_ok = link_error is None and bool(t)
    checks = {
        "link_ok": link_ok,
        "link_rate_ok": link_ok and t.get("hz_min", 0.0) >= accept["telem_hz_min"],
        "link_errors_ok": link_ok and errs <= accept["max_error_frames"],
        "link_no_loss": link_ok and lost_events <= accept["max_link_loss_events"],
        "link_hb_age_ok": link_ok and (last.get("hb_age_ms", 10 ** 6) <= accept["max_hb_age_ms"]),
        # 相机：--no-camera 时这两项不判定（stub 只为让时长/CPU 有意义）；
        # 要求开相机但打开失败时，camera_opened 直接 FAIL，避免"假通过"。
        "camera_opened": (not use_camera) or cam_s["mode"] == "camera",
        "camera_frames_ok": (not use_camera)
                            or (cam_s["mode"] == "camera"
                                and cam_s["frames"] >= accept["camera_frames_min"]),
        "camera_fps_ok": (not use_camera)
                         or (cam_s["mode"] == "camera"
                             and cam_s["fps_mean"] >= accept["camera_fps_min"]),
        "cpu_ok": (cpu_s.get("mean_percent", 0.0) <= accept["cpu_percent_max"]),
    }
    return {
        "port": port, "baud": baud, "duration_s": round(time.time() - t0, 2),
        "camera": cam_s, "camera_expected": use_camera,
        "link": link_res, "link_error": link_error, "cpu": cpu_s,
        "checks": checks,
        "ok": all(checks.values()),
        "started": time.strftime("%Y-%m-%dT%H:%M:%S"),
    }


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="camera + link concurrent soak")
    ap.add_argument("--config", default=str(HERE.parents[1] / "config" / "link.json"))
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=None)
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--fps", type=float, default=None)
    ap.add_argument("--no-camera", action="store_true", help="不打开相机，只跑链路（PC 上用）")
    ap.add_argument("--save", type=int, default=0, help="保存前 N 帧深度到 out/")
    ap.add_argument("--out-dir", default=None)
    ap.add_argument("--allow-setvel", nargs=2, type=int, metavar=("L", "R"))
    ap.add_argument("--json", default=None)
    args = ap.parse_args(argv)

    cfg = PF.load_config(args.config)
    port = PF.resolve_port(cfg, args.port)
    baud = args.baud or cfg["baud"]
    fps = args.fps or cfg["camera"]["fps"]
    print("[soak] port=%s baud=%d duration=%.0fs fps=%.1f camera=%s"
          % (port, baud, args.duration, fps, "off" if args.no_camera else "on"))
    res = run(port, baud, cfg, args.duration, fps, not args.no_camera,
              args.save, args.out_dir,
              tuple(args.allow_setvel) if args.allow_setvel else None)

    print("\n[soak] camera: %s" % res["camera"])
    if res.get("link_error"):
        print("[soak] link  : FAILED -> %s" % res["link_error"])
    else:
        print("[soak] link  : %s" % res["link"].get("telem"))
    print("[soak] cpu   : %s" % res["cpu"])
    for k, v in res["checks"].items():
        print("  %s %s" % ("PASS" if v else "FAIL", k))
    print("\n%s" % ("SOAK PASS" if res["ok"] else "SOAK FAIL"))

    if args.json:
        Path(args.json).parent.mkdir(parents=True, exist_ok=True)
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(res, f, indent=2, ensure_ascii=False)
        print("-> %s" % args.json)
    return 0 if res["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

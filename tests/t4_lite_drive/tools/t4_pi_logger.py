#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""t4_pi_logger —— 在香橙派本地持续采集，断网也不丢数据。

数据源：
  1) ETH 链路遥测（UDP 5001，来自 FPGA 192.168.0.2:5000）——主数据
  2) 可选：无线串口（--serial），掉线自动重连，只记录文本行
  3) 可选：深度相机距离（--camera，每秒 1 次统计）

输出：~/t4_logs/session_YYYYmmdd_HHMMSS/
  telem.jsonl   每帧遥测（epoch_ms + enc/yaw/speed/fault/seq）
  events.log    链路中断、fault 变化、操作标记、串口行
  meta.json     运行参数
  size.log      文件大小（供空间管理）

空间管理：单会话超过 --max-mb 自动滚动到 telem.2.jsonl…；保留最近 --keep 个会话。
"""
import argparse
import json
import os
import socket
import struct
import sys
import threading
import time

TOOLS = os.path.dirname(os.path.abspath(__file__))
if TOOLS not in sys.path:
    sys.path.insert(0, TOOLS)
import t4_lite_drive as T  # noqa: E402

BASE = os.path.expanduser("~/t4_logs")


def now_ms():
    return int(time.time() * 1000)


class Logger:
    def __init__(self, max_mb=200.0, keep=6):
        os.makedirs(BASE, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.dir = os.path.join(BASE, "session_" + stamp)
        os.makedirs(self.dir, exist_ok=True)
        self.max_bytes = int(max_mb * 1024 * 1024)
        self.keep = keep
        self.roll = 0
        self.fh = None
        self.fh_bytes = 0
        self.events = open(os.path.join(self.dir, "events.log"), "a", encoding="utf-8")
        self.meta = {"start": time.strftime("%Y-%m-%d %H:%M:%S"), "sessions_kept": keep}
        self.lock = threading.Lock()
        self._open_telem()
        self._prune_old()

    def _open_telem(self):
        name = "telem.jsonl" if self.roll == 0 else "telem.%d.jsonl" % self.roll
        path = os.path.join(self.dir, name)
        self.fh = open(path, "a", encoding="utf-8")
        self.fh_bytes = os.path.getsize(path) if os.path.exists(path) else 0
        self.telem_path = path

    def _prune_old(self):
        sessions = sorted([d for d in os.listdir(BASE) if d.startswith("session_")])
        for d in sessions[:-self.keep] if len(sessions) > self.keep else []:
            full = os.path.join(BASE, d)
            try:
                for f in os.listdir(full):
                    os.remove(os.path.join(full, f))
                os.rmdir(full)
                print("[logger] pruned", d)
            except OSError as exc:
                print("[logger] prune failed", d, exc)

    def event(self, text):
        with self.lock:
            self.events.write("%d %s\n" % (now_ms(), text))
            self.events.flush()

    def telem(self, t):
        rec = {"t": now_ms(), "enc_l": t.enc_l, "enc_r": t.enc_r,
               "yaw": t.yaw_cdeg, "v_l": t.speed_l, "v_r": t.speed_r,
               "fault": t.fault, "ls": t.link_seq, "cs": t.cmd_seq, "hb": t.hb_age_ms}
        line = json.dumps(rec, separators=(",", ":")) + "\n"
        with self.lock:
            self.fh.write(line)
            self.fh_bytes += len(line)
            if self.fh_bytes >= self.max_bytes:
                self.fh.close()
                self.roll += 1
                self._open_telem()
                self.event("roll to " + os.path.basename(self.telem_path))

    def write_meta(self):
        self.meta["telem_file"] = os.path.basename(self.telem_path)
        with open(os.path.join(self.dir, "meta.json"), "w", encoding="utf-8") as f:
            json.dump(self.meta, f, ensure_ascii=False, indent=2)

    def close(self):
        try:
            self.fh.close()
            self.events.close()
        except Exception:
            pass


def udp_loop(logger, stop, port=5001):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(0.2)
    last_rx = time.time()
    gap = False
    fault_prev = None
    frames = 0
    while not stop.is_set():
        try:
            data, addr = sock.recvfrom(2048)
        except socket.timeout:
            if not gap and time.time() - last_rx > 1.0:
                gap = True
                logger.event("LINK_GAP start (no telem >1s)")
            continue
        t = T.parse_telemetry(data, time.time())
        if t is None:
            continue
        frames += 1
        last_rx = time.time()
        if gap:
            gap = False
            logger.event("LINK_GAP end")
        logger.telem(t)
        if fault_prev is None or t.fault != fault_prev:
            logger.event("FAULT 0x%02X" % t.fault)
            fault_prev = t.fault
    sock.close()
    logger.event("udp loop stopped, frames=%d" % frames)


def serial_loop(logger, stop, port, baud=115200):
    try:
        import serial
    except Exception as exc:
        logger.event("serial: pyserial missing (%s)" % exc)
        return
    while not stop.is_set():
        try:
            sp = serial.Serial(port, baud, timeout=0.2)
            logger.event("serial open %s" % port)
            while not stop.is_set():
                line = sp.readline()
                if line:
                    logger.event("UART " + line.decode("utf-8", "replace").rstrip())
            sp.close()
        except Exception as exc:
            logger.event("serial error: %s" % exc)
            time.sleep(1.0)


def camera_loop(logger, stop, period=1.0):
    from pyorbbecsdk import Pipeline, Config, OBSensorType
    cyc = 0
    while not stop.is_set():
        try:
            pipeline = Pipeline()
            cfg = Config()
            prof = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR).get_default_video_stream_profile()
            cfg.enable_stream(prof)
            pipeline.start(cfg)
            while not stop.is_set():
                fs = pipeline.wait_for_frames(1000)
                if fs and fs.get_depth_frame():
                    df = fs.get_depth_frame()
                    import numpy as np
                    a = np.frombuffer(df.get_data(), dtype=np.uint16).reshape(
                        df.get_height(), df.get_width())
                    top = a[0:max(6, a.shape[0] // 8), :]
                    v = top[top > 0]
                    if v.size:
                        logger.event("CAM dist_mm=%.0f" % float(np.median(v)))
                cyc += 1
                time.sleep(max(0.0, period - 0.05))
            pipeline.stop()
        except Exception as exc:
            logger.event("camera error: %s" % exc)
            time.sleep(2.0)


def main():
    ap = argparse.ArgumentParser(description="t4 local logger (runs on the Orange Pi)")
    ap.add_argument("--port", type=int, default=5001, help="UDP 接收端口")
    ap.add_argument("--serial", default=None, help="可选无线串口，如 /dev/ttyACM0")
    ap.add_argument("--camera", action="store_true", help="记录深度相机墙面距离")
    ap.add_argument("--duration", type=float, default=0.0, help="运行秒数（0=一直）")
    ap.add_argument("--max-mb", type=float, default=200.0)
    ap.add_argument("--keep", type=int, default=6)
    args = ap.parse_args()

    logger = Logger(max_mb=args.max_mb, keep=args.keep)
    logger.event("session start dir=%s serial=%s camera=%s" % (logger.dir, args.serial, args.camera))
    logger.write_meta()
    print("[logger] dir:", logger.dir)
    stop = threading.Event()
    threads = [threading.Thread(target=udp_loop, args=(logger, stop, args.port), daemon=True)]
    if args.serial:
        threads.append(threading.Thread(target=serial_loop, args=(logger, stop, args.serial), daemon=True))
    if args.camera:
        threads.append(threading.Thread(target=camera_loop, args=(logger, stop), daemon=True))
    for th in threads:
        th.start()
    t0 = time.time()
    try:
        while True:
            if args.duration > 0 and time.time() - t0 >= args.duration:
                break
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    stop.set()
    time.sleep(0.5)
    logger.event("session stop")
    logger.write_meta()
    logger.close()
    print("[logger] done:", logger.dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

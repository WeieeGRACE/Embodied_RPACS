#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""t4_lite_drive —— 深度相机测距 + 编码器执行 + ToF 兜底停车的单段驾驶工具。

流程：
  1) 深度相机测“车头→墙”的真实距离 D（中央 ROI 多帧中值，mm）
  2) 目标行程 = D - margin（默认 120 mm）
  3) 千兆 UDP 链路下发 SET_VEL（带 seq/心跳），50 Hz 读 TELEM 里的编码器里程
  4) 里程到达目标 → STOP；全程监控 fault / 链路丢失 / 速度异常
  5) FPGA 侧 ToF<=120 mm 会硬件切断电机（独立于本工具）

用法（Pi，需 pyorbbecsdk 的 venv + PYTHONPATH=install/lib）：
  python t4_lite_drive.py --measure-only          # 只测距
  python t4_lite_drive.py --speed 100             # 测距后自动开一段
  python t4_lite_drive.py --target 800 --no-camera
"""
from __future__ import annotations

import argparse
import socket
import struct
import sys
import time

# ------------------------------------------------------------------ 协议 v1.1
SYNC = b"\xAA\x55"
TAIL = 0x0D
T_SET_VEL = 0x01
T_STOP = 0x02
T_HEARTBEAT = 0x03
T_SET_LIMIT = 0x04
T_CLEAR_FAULT = 0x05
T_TELEM = 0x81
T_ACK = 0x82

# 协议 v1.1 §4.3 故障位图
FAULT_NAMES = {
    0: "NONE",
    1: "STOP_ESTOP_LATCH",
    2: "LINK_LOST",
    4: "ESTOP",
    8: "OVER_LIMIT",
    16: "CRC_ERR",
    32: "CMD_TIMEOUT",
}
TELEM_LEN = 23
CAM_OFFSET_MM = 244.0   # 相机->车头保险杠固定偏移（卷尺 800/1046 标定）


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_frame(ftype: int, payload: bytes = b"") -> bytes:
    body = bytes([ftype, len(payload)]) + payload
    return SYNC + body + struct.pack("<H", crc16_ccitt(body)) + bytes([TAIL])


class Telemetry:
    __slots__ = ("enc_l", "enc_r", "yaw_cdeg", "dbg_a", "dbg_b", "speed_l", "speed_r",
                 "fault", "link_seq", "cmd_seq", "hb_age_ms", "t")

    @property
    def yaw_deg(self) -> float:
        return self.yaw_cdeg / 100.0

    def __init__(self, payload: bytes, t: float):
        (self.enc_l, self.enc_r, self.yaw_cdeg,
         self.dbg_a, self.dbg_b, self.speed_l, self.speed_r,
         self.fault, self.link_seq, self.cmd_seq, self.hb_age_ms) = struct.unpack(
             "<iihhhhhBBBH", payload[:23])
        self.t = t

    @property
    def distance_mm(self) -> float:
        return (self.enc_l + self.enc_r) / 2.0


def parse_telemetry(raw: bytes, now: float):
    """从 UDP 负载里找 TELEM 帧（一包一帧；容忍多余字节）。"""
    idx = raw.find(SYNC)
    while idx >= 0 and idx + 4 <= len(raw):
        ftype = raw[idx + 2]
        length = raw[idx + 3]
        crc_end = idx + 4 + length + 2
        if crc_end > len(raw):
            idx = raw.find(SYNC, idx + 1)
            continue
        payload = raw[idx + 4: idx + 4 + length]
        body = raw[idx + 2: idx + 4 + length]
        crc = struct.unpack("<H", raw[idx + 4 + length: crc_end])[0]
        # 实测该链路在帧前会多出一个 0x0D（TX 管线首字节错位），因此不强制要求帧尾。
        if crc16_ccitt(body) == crc and ftype == T_TELEM:
            try:
                return Telemetry(payload, now)
            except struct.error:
                return None
        idx = raw.find(SYNC, idx + 1)
    return None


# ------------------------------------------------------------------ 深度相机
def measure_wall_distance(frames: int = 20, roi_frac: float = 0.25):
    """返回 (墙面中值 mm, 最小有效 mm, 统计字典)。

    相机固定俯角 23°、墙板 30 cm：墙面出现在图像上方，地面在下方。所以取图像
    上方墙带（顶部 12% 行）的中值作为墙面距离；该距离是“相机->墙”，需要再减去
    相机到车头的固定偏移（--cam-offset）。
    """
    import numpy as np
    from pyorbbecsdk import Pipeline, Config, OBSensorType

    pipeline = Pipeline()
    cfg = Config()
    plist = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
    prof = plist.get_default_video_stream_profile()
    cfg.enable_stream(prof)
    pipeline.start(cfg)
    samples = []
    stats = {"frames": 0, "valid_frames": 0}
    try:
        deadline = time.time() + 5.0
        while stats["frames"] < frames and time.time() < deadline:
            fs = pipeline.wait_for_frames(1000)
            if not fs:
                continue
            df = fs.get_depth_frame()
            if not df:
                continue
            stats["frames"] += 1
            w, h = df.get_width(), df.get_height()
            data = np.frombuffer(df.get_data(), dtype=np.uint16).reshape(h, w)
            cy, cx = h // 2, w // 2
            rh = max(8, int(h * roi_frac / 2))
            rw = max(8, int(w * roi_frac / 2))
            # 墙带：图像上方 12% 行（俯角固定，墙在上方）
            top_rows = max(6, int(h * 0.12))
            band = data[0:top_rows, :]
            valid = band[band > 0]
            if valid.size < 200:
                # 兜底：若上方无有效深度，用中心 ROI
                roi = data[cy - rh:cy + rh, cx - rw:cx + rw]
                valid = roi[roi > 0]
            if valid.size == 0:
                continue
            stats["valid_frames"] += 1
            samples.append(float(np.median(valid)))
    finally:
        try:
            pipeline.stop()
        except Exception:
            pass
    if not samples:
        raise RuntimeError("深度相机没有拿到有效深度（检查 USB/驱动）")
    med = float(np.median(samples))
    stats["min"] = min(samples)
    stats["max"] = max(samples)
    return med, stats["min"], stats


# ------------------------------------------------------------------ 链路驾驶
class Link:
    def __init__(self, remote_ip: str = "192.168.0.2", remote_port: int = 5000,
                 local_port: int = 5001):
        self.remote = (remote_ip, remote_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", local_port))
        self.sock.settimeout(0.001)
        self.seq = 0
        self.hb_seq = 0
        self.last_telem = None
        self.telem_time = 0.0
        self.acks = []

    def send(self, ftype: int, payload: bytes = b"") -> None:
        self.sock.sendto(build_frame(ftype, payload), self.remote)

    def set_vel(self, left: int, right: int) -> int:
        seq = self.seq & 0xFF
        self.seq = (self.seq + 1) & 0xFF
        self.send(T_SET_VEL, struct.pack("<hhB", left, right, seq))
        return seq

    def stop(self) -> None:
        self.send(T_STOP)

    def clear_fault(self) -> None:
        self.send(T_CLEAR_FAULT)

    def set_limit(self, mm_s: int) -> None:
        self.send(T_SET_LIMIT, struct.pack("<h", mm_s))

    def heartbeat(self) -> None:
        # 心跳负载固定 1 字节序号（协议 v1.1；0 字节会被 FPGA 判为非法帧）
        seq = self.hb_seq & 0xFF
        self.hb_seq = (self.hb_seq + 1) & 0xFF
        self.send(T_HEARTBEAT, bytes([seq]))

    def poll(self, timeout_s: float = 0.0) -> list:
        out = []
        deadline = time.time() + timeout_s
        while True:
            try:
                data, _ = self.sock.recvfrom(2048)
            except socket.timeout:
                break
            now = time.time()
            t = parse_telemetry(data, now)
            if t is not None:
                self.last_telem = t
                self.telem_time = now
                out.append(t)
            if time.time() >= deadline:
                break
        return out

    def wait_telemetry(self, timeout_s: float = 3.0):
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            got = self.poll(0.02)
            if got:
                return got[-1]
        return None


def run_drive(link: Link, target_mm: float, speed_mm_s: int,
              max_time_s: float = 60.0, ramp_s: float = 1.0) -> int:
    link.heartbeat()
    t0 = link.wait_telemetry(3.0)
    if t0 is None:
        print("[drive] 收不到 TELEM：检查网线/IP（Pi eth0 192.168.0.1/24）、FPGA 是否 192.168.0.2")
        return 2
    print("[drive] 链路在线：enc=(%d, %d) fault=%s" %
          (t0.enc_l, t0.enc_r, FAULT_NAMES.get(t0.fault, hex(t0.fault))))
    base = t0.distance_mm
    print("[drive] 目标 %.0f mm（起点里程 %.1f）" % (target_mm, base))

    link.set_limit(abs(speed_mm_s) + 30)
    # 先让 link_ok 恢复，再清锁存故障（CLEAR_FAULT 在 link_ok=0 时会被拒绝）
    for _ in range(40):
        link.heartbeat()
        link.poll(0.02)
    for _ in range(40):
        link.heartbeat()
        link.clear_fault()
        got = link.poll(0.03)
        if got and (got[-1].fault & ~0x20) == 0:
            break
    start = time.time()
    last_hb = 0.0
    last_cmd = 0.0
    stuck_since = None
    last_dist = base
    last_progress = time.time()
    last_report = time.time()
    final = t0
    final_yaw = 0.0
    yaw_ref = 0.0
    yaw_ref_set = False
    try:
        while True:
            now = time.time()
            elapsed = now - start
            if now - last_hb >= 0.1:
                link.heartbeat()
                last_hb = now
            # 斜坡：前 ramp_s 秒线性加到目标速度
            ratio = min(1.0, elapsed / ramp_s) if ramp_s > 0 else 1.0
            v = max(25, int(abs(speed_mm_s) * ratio))
            # 陀螺仪航向保持：航向偏差 -> 左右轮差速（正=向右修正）
            yaw_err = yaw_ref - final_yaw
            # 实测：cmd(vl>vr) 使 yaw 增大（左转）。yaw 超参考值(err<0) 时要右转 -> vl<vr -> corr<0
            # 增益 1.5 mm/s per degree（err 单位=度）
            corr = max(-20.0, min(20.0, 1.5 * yaw_err))
            if now - last_cmd >= 0.05:
                link.set_vel(int(v + corr), int(v - corr))
                last_cmd = now
            fresh = link.poll(0.01)
            if fresh:
                final = fresh[-1]
                final_yaw = final.yaw_deg
                if not yaw_ref_set:
                    yaw_ref = final_yaw
                    yaw_ref_set = True
            if fresh and final is not None:
                if final.fault and (final.fault & ~0x20):
                    print("[drive] FPGA 报告故障 fault=0x%02X，停车" % final.fault)
                    link.stop()
                    return 3
                if now - link.telem_time > 0.5:
                    print("[drive] TELEM 超时（>0.5 s），可能链路丢失/ToF 停车，停车")
                    link.stop()
                    return 4
                dist = final.distance_mm - base
                # 里程无进展（>0.8 s 且 <2 mm）→ 被安全门控/ToF 切断或堵转
                if now - last_progress > 1.6 and abs(dist - last_dist) < 2.0 and elapsed > 2.0:
                    print("[drive] 里程无进展（%.1f mm，0.8 s 未前进），可能被 ToF 兜底停车切断" % dist)
                    link.stop()
                    return 5
                if abs(dist - last_dist) >= 2.0:
                    last_dist = dist
                    last_progress = now
                if now - last_report >= 0.5:
                    last_report = now
                    print("[drive] %.2f s  里程 %.1f / %.0f mm  yaw=%.2f° (err=%.2f°)  v=(%d,%d) fault=0x%02X"
                          % (elapsed, dist, target_mm, final_yaw, yaw_err, final.speed_l, final.speed_r, final.fault))
                if dist >= target_mm:
                    print("[drive] 到达目标：里程 %.1f mm >= %.0f mm，发 STOP" % (dist, target_mm))
                    link.stop()
                    break
            if elapsed > max_time_s:
                print("[drive] 超时 %.0f s，里程 %.1f mm，停车" % (elapsed, final.distance_mm - base))
                link.stop()
                return 6
            time.sleep(0.005)
    finally:
        pass

    # 停车后收尾：确认速度归零
    end_deadline = time.time() + 1.0
    last = final
    while time.time() < end_deadline:
        link.heartbeat()
        got = link.poll(0.02)
        if got:
            last = got[-1]
    dist = last.distance_mm - base
    print("[drive] 完成：最终里程 %.1f mm（目标 %.0f），fault=%s，v=(%d,%d)" %
          (dist, target_mm, FAULT_NAMES.get(last.fault, hex(last.fault)),
           last.speed_l, last.speed_r))
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="t4_lite_drive: depth-camera distance + encoder drive")
    ap.add_argument("--measure-only", action="store_true", help="只测距，不驱动")
    ap.add_argument("--link-check", action="store_true", help="只检查链路/遥测，不发运动命令")
    ap.add_argument("--no-camera", action="store_true", help="跳过相机，用 --target")
    ap.add_argument("--margin", type=float, default=120.0, help="安全余量 mm（默认 120）")
    ap.add_argument("--cam-offset", type=float, default=244.0,
                    help="相机到车头保险杠的固定偏移 mm（实测 244，卷尺标定）")
    ap.add_argument("--target", type=float, default=None, help="直接给目标行程 mm（跳过相机计算）")
    ap.add_argument("--speed", type=int, default=100, help="轮速 mm/s（默认 100）")
    ap.add_argument("--max-time", type=float, default=60.0, help="最大行驶时间 s")
    ap.add_argument("--frames", type=int, default=20, help="测距采样帧数")
    ap.add_argument("--remote", default="192.168.0.2", help="FPGA IP")
    ap.add_argument("--local-port", type=int, default=5001, help="本机 UDP 接收端口")
    args = ap.parse_args()

    if args.link_check:
        link = Link(remote_ip=args.remote, local_port=args.local_port)
        link.heartbeat()
        t = link.wait_telemetry(3.0)
        if t is None:
            print("[link] 收不到 TELEM：检查网线/IP/FPGA 位流")
            return 2
        print("[link] TELEM OK: enc_l=%d enc_r=%d fault=%s v=(%d,%d) hb_age=%d ms"
              % (t.enc_l, t.enc_r, FAULT_NAMES.get(t.fault, hex(t.fault)),
                 t.speed_l, t.speed_r, t.hb_age_ms))
        print("[link] dbg_a(tof_cal_mm)=%d dbg_b(flags)=0x%04X  [tof_stop=%d valid=%d err=%d key_en=%d key_estop=%d allow=%d link_ok=%d]"
              % (t.dbg_a, t.dbg_b, t.dbg_b & 1, (t.dbg_b >> 1) & 1, (t.dbg_b >> 2) & 1,
                 (t.dbg_b >> 3) & 1, (t.dbg_b >> 4) & 1, (t.dbg_b >> 5) & 1, (t.dbg_b >> 6) & 1))
        return 0

    wall = None
    if not args.no_camera:
        print("[cam] 打开深度相机测距…")
        try:
            med, mn, stats = measure_wall_distance(frames=args.frames)
            wall = med
            print("[cam] 墙面距离 中值=%.0f mm  最小=%.0f mm  (帧 %d/%d)"
                  % (med, mn, stats["valid_frames"], stats["frames"]))
        except Exception as exc:
            print("[cam] 测距失败：%s" % exc)
            if args.target is None:
                return 2
    if args.measure_only:
        return 0

    target = args.target
    if target is None:
        if wall is None:
            print("[drive] 没有相机距离，也没有 --target")
            return 2
        bumper = wall - args.cam_offset
        target = bumper - args.margin
        print("[drive] 墙面(相机)=%.0f - 偏移 %.0f = 车头距墙 %.0f mm" %
              (wall, args.cam_offset, bumper))
        print("[drive] 目标行程 = %.0f - %.0f = %.0f mm" % (bumper, args.margin, target))
    if target <= 100:
        print("[drive] 目标行程 %.0f mm 太小，拒绝执行" % target)
        return 2

    link = Link(remote_ip=args.remote, local_port=args.local_port)
    try:
        return run_drive(link, target, abs(args.speed), args.max_time)
    finally:
        try:
            link.stop()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())

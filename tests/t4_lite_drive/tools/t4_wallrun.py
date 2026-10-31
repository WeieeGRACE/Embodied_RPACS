#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""t4_wallrun.py —— 按墙触发（不再用固定里程）。

每个阶段可以是：
  {"type": "wall",  "gap": 250, "speed": 30}   前进直到相机测到车头离墙 <= gap
  {"type": "arc",   "deg": -90, "speed": 50}   弧线转向（边走边转）
  {"type": "drive", "mm": 200,  "speed": 30}   固定里程（很少用）
  {"type": "turn",  "deg": 90}                 原地转（仅空旷处）
  {"type": "reverse_until", "gap": 500, "speed": 25}  倒车直到车头离墙 >= gap

用法（Pi venv python）：
  python3 t4_wallrun.py --route wall_route_A_to_B.json --from 0 --count 1
  python3 t4_wallrun.py --route wall_route_A_to_B.json --from 0 --count 1 --speed 25 --probing
  进度存 steps_state_<route>.json（与 t4_steps.py 共用同一个名字不会冲突，因为文件名不同）。

安全：车头离墙 < 70 mm 立即停车报 ABORT。
"""
import argparse
import io
import json
import os
import sys
import time

TOOLS = os.path.dirname(os.path.abspath(__file__))
if TOOLS not in sys.path:
    sys.path.insert(0, TOOLS)
import route_run as R
import t4_lite_drive as T

CAM_OFFSET = 244


class Cam:
    def __init__(self):
        import numpy as np
        from pyorbbecsdk import Pipeline, Config, OBSensorType
        self.np = np
        self.pipeline = Pipeline()
        cfg = Config()
        plist = self.pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
        prof = plist.get_default_video_stream_profile()
        cfg.enable_stream(prof)
        self.pipeline.start(cfg)
        self.last = None

    def gap_mm(self):
        """相机->墙的距离（mm），失败返回 None。"""
        fs = self.pipeline.wait_for_frames(200)
        if not fs:
            return None
        df = fs.get_depth_frame()
        if not df:
            return None
        w, h = df.get_width(), df.get_height()
        data = self.np.frombuffer(df.get_data(), dtype=self.np.uint16).reshape(h, w)
        # 取图像上半部分（含顶带与中部）的有效深度，排除远墙(>3500)与天空(0)
        band = data[0:max(10, int(h * 0.55)), :]
        valid = band[(band > 200) & (band < 3500)]
        if valid.size < 200:
            return None
        self.last = float(self.np.median(valid))
        return self.last

    def close(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass


def bump(ctl, cam, target_gap, speed, timeout=90.0, slow_gap=450.0):
    """前进直到 ToF 测到车头离墙 <= target_gap（ToF 为主，相机交叉校验）。

    ToF 标定偏移：实测 tape 710 mm 时 ToF 读 737，故阈值 = target_gap + 30。
    """
    if hasattr(ctl, 'rearm'):
        ctl.rearm()
    base = ctl.last.distance_mm
    t0 = time.time()
    tof_off = 30.0
    cam_gap = cam.gap_mm()
    tof = getattr(ctl.last, 'dbg_a', None)
    print('  start tof=%s cam=%s (cam bumper %.0f)' %
          (tof, ('%.0f' % cam_gap) if cam_gap else 'n/a',
           (cam_gap - CAM_OFFSET) if cam_gap else float('nan')))
    last_tof = None
    while True:
        ctl.link.heartbeat()
        ctl.pump(0.02)
        tof = getattr(ctl.last, 'dbg_a', None)
        cam_gap = cam.gap_mm()
        bumper_cam = (cam_gap - CAM_OFFSET) if cam_gap else None
        trigger = None
        if tof is not None and 40 < tof < 3000:
            last_tof = tof
            trigger = tof - tof_off
        elif bumper_cam is not None:
            trigger = bumper_cam
        if trigger is not None and trigger < 70:
            ctl.link.stop()
            print('  ABORT: gap %.0f mm too close (tof=%s cam=%s)' % (trigger, tof, bumper_cam))
            return False, trigger, ctl.last.distance_mm - base
        if trigger is not None and trigger <= target_gap:
            break
        v = speed if (trigger is None or trigger > slow_gap) else max(15, int(speed * 0.5))
        ctl.link.set_vel(v, v)
        if time.time() - t0 > timeout:
            ctl.link.stop()
            print('  TIMEOUT gap=%s tof=%s cam=%s' % (trigger, tof, bumper_cam))
            return False, trigger, ctl.last.distance_mm - base
    ctl.link.stop()
    tw = time.time()
    while time.time() - tw < 0.5:
        ctl.link.heartbeat()
        ctl.pump(0.03)
    cam_gap = cam.gap_mm()
    tof = getattr(ctl.last, 'dbg_a', None)
    print('  stop: tof=%s cam=%s (cam bumper %.0f) travel %.1f mm' %
          (tof, ('%.0f' % cam_gap) if cam_gap else 'n/a',
           (cam_gap - CAM_OFFSET) if cam_gap else float('nan'),
           ctl.last.distance_mm - base))
    return True, (tof - tof_off) if tof else None, ctl.last.distance_mm - base


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--route", required=True)
    ap.add_argument("--from", dest="frm", type=int, default=None)
    ap.add_argument("--count", type=int, default=1)
    ap.add_argument("--speed", type=int, default=30)
    ap.add_argument("--reset", action="store_true")
    args = ap.parse_args()
    path = os.path.join(TOOLS, args.route)
    stages = json.load(io.open(path, encoding="utf-8"))["stages"]
    state = os.path.join(TOOLS, "steps_state_" + args.route.replace(".json", "") + ".json")
    start = args.frm if args.frm is not None else 0
    if not args.reset and args.frm is None and os.path.exists(state):
        try:
            start = json.load(io.open(state, encoding="utf-8"))["next"]
        except Exception:
            start = 0
    ctl = R.Ctl(args.speed)
    if not ctl.bring_up():
        print("link bring-up failed")
        return 2
    cam = Cam()
    print("pose dist=%.0f yaw=%.2f fault=0x%02X" %
          (ctl.last.distance_mm, ctl.yaw_deg, ctl.last.fault))
    end = min(start + args.count, len(stages))
    rc = 0
    try:
        for i in range(start, end):
            st = stages[i]
            print("-- stage %d: %s" % (i, st))
            t0 = time.time()
            d0, y0 = ctl.last.distance_mm, ctl.yaw_deg
            ok = True
            if st["type"] == "wall":
                ok, gap, trav = bump(ctl, cam, float(st.get("gap", 250)),
                                     int(st.get("speed", args.speed)))
            elif st["type"] == "reverse_until":
                ctl.rearm()
                base = ctl.last.distance_mm
                t1 = time.time()
                while True:
                    ctl.link.heartbeat()
                    g = cam.gap_mm()
                    bumper = (g - CAM_OFFSET) if g else None
                    if bumper is not None and bumper >= float(st.get("gap", 500)):
                        break
                    ctl.link.set_vel(-int(st.get("speed", 25)), -int(st.get("speed", 25)))
                    ctl.pump(0.02)
                    if time.time() - t1 > 30:
                        ok = False
                        break
                ctl.link.stop()
            elif st["type"] == "arc":
                ok = R.arc_turn(ctl, float(st["deg"]))
            elif st["type"] == "drive":
                ok = R.drive(ctl, float(st["mm"]), st.get("speed", args.speed))
            elif st["type"] == "turn":
                ok = R.turn(ctl, float(st["deg"]))
            print("   => dd=%.1f mm dyaw=%+.2f %s  (%.1fs)" %
                  (ctl.last.distance_mm - d0, ctl.yaw_deg - y0, "OK" if ok else "FAIL",
                   time.time() - t0))
            if not ok:
                print("   stop: stage failed")
                json.dump({"next": i}, io.open(state, "w", encoding="utf-8"))
                rc = 1
                break
    finally:
        cam.close()
    if rc == 0:
        json.dump({"next": end}, io.open(state, "w", encoding="utf-8"))
        print("saved next=%d" % end)
    return rc


if __name__ == "__main__":
    raise SystemExit(main())

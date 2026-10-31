#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""t4_steps.py —— 边走边测：按路线文件顺序执行 N 个动作，每段结束实测并打印，进度存盘。

用法（Pi，venv python 才能用相机）：
  # 先看当前实测（不动作）
  python3 t4_steps.py --route route1_A_to_B_plan.json --probe
  # 从第 0 个动作开始走 3 个
  python3 t4_steps.py --route route1_A_to_B_plan.json --from 0 --count 3 --speed 25
  # 继续（自动从上次进度）
  python3 t4_steps.py --route route1_A_to_B_plan.json --count 3 --speed 25
  # 重来
  python3 t4_steps.py --route ... --from 0 --reset
每段结束打印：序号/动作、实际里程或航向、相机前墙距离、fault。
进度写 ~/t4_tools/steps_state_<route>.json。
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


def cam_dist(frames=5):
    try:
        med, _mn, _st = T.measure_wall_distance(frames=frames)
        return med
    except Exception as e:
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--route", required=True)
    ap.add_argument("--from", dest="frm", type=int, default=None)
    ap.add_argument("--count", type=int, default=3)
    ap.add_argument("--speed", type=int, default=25)
    ap.add_argument("--probe", action="store_true")
    ap.add_argument("--reset", action="store_true")
    args = ap.parse_args()

    path = os.path.join(TOOLS, args.route)
    legs = json.load(io.open(path, encoding="utf-8"))["legs"]
    state_path = os.path.join(TOOLS, "steps_state_%s.json" % args.route.replace(".json", ""))
    start = 0
    if args.frm is not None:
        start = args.frm
    elif os.path.exists(state_path) and not args.reset:
        try:
            start = json.load(io.open(state_path, encoding="utf-8"))["next"]
        except Exception:
            start = 0
    if args.reset:
        start = 0

    ctl = R.Ctl(args.speed)
    if not ctl.bring_up():
        print("link bring-up failed")
        return 2
    print("pose: dist=%.0f yaw=%.2f fault=0x%02X" %
          (ctl.last.distance_mm, ctl.yaw_deg, ctl.last.fault))
    d = cam_dist()
    print("camera to front wall: %s mm" % ("%.0f" % d if d else "n/a"))
    if args.probe:
        json.dump({"next": start}, io.open(state_path, "w", encoding="utf-8"))
        return 0

    end = min(start + args.count, len(legs))
    print("running legs %d..%d of %d" % (start, end - 1, len(legs)))
    for i in range(start, end):
        leg = legs[i]
        print("-- leg %d: %s" % (i, leg))
        t0 = time.time()
        d0, y0 = ctl.last.distance_mm, ctl.yaw_deg
        ok = True
        if leg["type"] == "drive":
            ok = R.drive(ctl, float(leg["mm"]), leg.get("speed", args.speed))
        elif leg["type"] == "reverse":
            ok = R.reverse(ctl, float(leg["mm"]), leg.get("speed", args.speed))
        elif leg["type"] == "arc":
            ok = R.arc_turn(ctl, float(leg["deg"]))
        elif leg["type"] == "turn":
            ok = R.turn(ctl, float(leg["deg"]))
        elif leg["type"] == "threepoint":
            ok = R.three_point(ctl, float(leg["deg"]))
        dd = ctl.last.distance_mm - d0
        dy = ctl.yaw_deg - y0
        cam = cam_dist()
        print("   -> dd=%.1f mm dyaw=%+.2f deg %s  cam=%s fault=0x%02X" %
              (dd, dy, "OK" if ok else "FAIL", ("%.0f" % cam) if cam else "n/a", ctl.last.fault))
        if not ok:
            print("   stop: leg failed")
            json.dump({"next": i}, io.open(state_path, "w", encoding="utf-8"))
            return 1
    json.dump({"next": end}, io.open(state_path, "w", encoding="utf-8"))
    print("saved progress next=%d" % end)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

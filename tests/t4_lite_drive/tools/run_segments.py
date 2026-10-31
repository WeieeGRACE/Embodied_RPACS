#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""run_segments.py —— 分段演示编排器（每段人工摆车，跑完一段再下一段）。

默认顺序：seg1_forward_450, seg2_forward_1000, seg3_turn_left90, seg4_turn_right90,
seg5_turn_180, seg6_L_shape, seg7_U_shape, seg8_wall_stop_120。

用法：
  python3 ~/t4_tools/run_segments.py                 # 逐段提示，回车开始
  python3 ~/t4_tools/run_segments.py --auto          # 不提示
  python3 ~/t4_tools/run_segments.py --only seg3_turn_left90 --speed 30
  python3 ~/t4_tools/run_segments.py --start 3       # 从第 3 段开始
结果写入 ~/t4_logs/segments_YYYYmmdd_HHMMSS.json。
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

ORDER = ["seg1_forward_450", "seg2_forward_1000", "seg3_turn_left90",
         "seg4_turn_right90", "seg5_turn_180", "seg6_L_shape", "seg7_U_shape",
         "seg8_wall_stop_120"]
ROUTES = ["route1_A_to_B_plan", "route2_A_to_C_plan",
          "route3_A_to_D_plan", "route4_A_to_E_plan"]


MANUAL_TURN = False
THREE_POINT = False
ARC_TURN = False


def run_segment(ctl, name, speed):
    path = os.path.join(TOOLS, name + ".json")
    legs = json.load(io.open(path, encoding="utf-8"))["legs"]
    ctl.rearm()
    d0 = ctl.last.distance_mm
    y0 = ctl.yaw_deg
    print("  legs:", legs)
    ok = True
    for leg in legs:
        if leg["type"] == "drive":
            ok = R.drive(ctl, float(leg["mm"]), leg.get("speed", speed)) and ok
        elif leg["type"] == "reverse":
            ok = R.reverse(ctl, float(leg["mm"]), leg.get("speed", speed)) and ok
        elif leg["type"] == "arc":
            ok = R.arc_turn(ctl, float(leg["deg"])) and ok
        elif leg["type"] == "threepoint":
            ok = R.three_point(ctl, float(leg["deg"])) and ok
        elif leg["type"] == "turn":
            if ARC_TURN:
                ok = R.arc_turn(ctl, float(leg["deg"])) and ok
                continue
            if THREE_POINT:
                ok = R.three_point(ctl, float(leg["deg"])) and ok
                continue
            if MANUAL_TURN:
                print("  [人工转向] 请把车原地转到 %+.1f°（相对当前车头），摆好后回车…" % leg["deg"])
                try:
                    input()
                except EOFError:
                    pass
                ctl.rearm()
            else:
                ok = R.turn(ctl, float(leg["deg"])) and ok
        elif leg["type"] == "to_wall":
            margin = float(leg.get("margin", 120))
            med, _mn, _st = T.measure_wall_distance(frames=10)
            bumper = med - T.CAM_OFFSET_MM
            travel = bumper - margin
            print("  相机 %.0f -> 车头 %.0f -> 行程 %.0f" % (med, bumper, travel))
            if travel < 80:
                ok = False
            else:
                ok = R.drive(ctl, travel, leg.get("speed", speed)) and ok
        if not ok:
            break
    dist = ctl.last.distance_mm - d0
    dyaw = ctl.yaw_deg - y0
    print("  => 里程 %.1f mm, 航向变化 %+.2f deg" % (dist, dyaw))
    return ok, {"segment": name, "legs": legs, "distance_mm": round(dist, 1),
                "yaw_change_deg": round(dyaw, 2)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--auto", action="store_true")
    ap.add_argument("--start", type=int, default=1)
    ap.add_argument("--only", default=None)
    ap.add_argument("--suite", choices=["segs", "routes"], default="segs")
    ap.add_argument("--speed", type=int, default=30)
    ap.add_argument("--arc", action="store_true",
                    help="用弧线转向代替原地转向（边走边转，600 走廊可过）")
    ap.add_argument("--three-point", action="store_true",
                    help="用三点掉头代替原地转向（推荐；走廊里原地转不开）")
    ap.add_argument("--manual-turn", action="store_true",
                    help="遇到转向段时提示人工把车转到目标角度，不自动原地转（走廊放不下）")
    args = ap.parse_args()

    base = ROUTES if args.suite == "routes" else ORDER
    global MANUAL_TURN, THREE_POINT, ARC_TURN
    MANUAL_TURN = args.manual_turn
    THREE_POINT = args.three_point
    ARC_TURN = args.arc
    names = [args.only] if args.only else base[max(0, args.start - 1):]
    ctl = R.Ctl(args.speed)
    if not ctl.bring_up():
        print("link bring-up failed")
        return 2
    print("start dist=%.1f yaw=%.2f fault=0x%02X" %
          (ctl.last.distance_mm, ctl.yaw_deg, ctl.last.fault))
    results = []
    for i, name in enumerate(names, start=1):
        if not args.auto:
            print("\n===== 第 %d 段 %s =====" % (i, name))
            print("请摆好车（按该段要求），回车开始…")
            try:
                input()
            except EOFError:
                pass
        else:
            print("\n===== 第 %d 段 %s =====" % (i, name))
        ok, rec = run_segment(ctl, name, args.speed)
        rec["ok"] = bool(ok)
        results.append(rec)
    ctl.stop()
    stamp = time.strftime("%Y%m%d_%H%M%S")
    out = os.path.expanduser("~/t4_logs/segments_%s.json" % stamp)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with io.open(out, "w", encoding="utf-8") as f:
        json.dump({"time": stamp, "speed": args.speed, "results": results}, f,
                  ensure_ascii=False, indent=2)
    print("\n===== 汇总 =====")
    for r in results:
        print("  %-22s %s  dist=%7.1f mm  dyaw=%+7.2f" %
              (r["segment"], "OK  " if r["ok"] else "FAIL", r["distance_mm"], r["yaw_change_deg"]))
    print("results ->", out)
    return 0 if all(r["ok"] for r in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())

# -*- coding: utf-8 -*-
"""Monte-Carlo simulation of every route executed by the host-side runner.

For each route JSON we simulate the physical execution with the calibrated error model:
  * turn: sigma 0.8 deg (measured -90 -> -88.9, +90 -> -0.2), plus 0.3 deg quantization
  * drive: distance gain error sigma 0.8 % + 3 mm per leg, heading hold sigma 0.6 deg/leg
The car pose starts at A=(2700,300) heading +Y.  We check:
  * final position error vs the planned end waypoint
  * minimum wall clearance along the simulated path (car 320 x 240 mm, sampled 25 mm)
  * collision (clearance < 40 mm) -> FAIL
Outputs a markdown report and prints a summary table.
"""
import io
import json
import math
import os
import random

ROOT = (r"D:\Gowin\Project\Contest\Embodied_RPACS\Embodied_RPACS\Embodied_RPACS")
TOOLS = os.path.join(ROOT, "tests", "t4_lite_drive", "tools")
MAP = os.path.join(ROOT, "tests", "task4_navigation", "data", "map_v1.json")
OUT_MD = os.path.join(ROOT, "tests", "t4_lite_drive", "records", "route_simulation.md")

RUNS = 200
WALLS = [(tuple(w["a"]), tuple(w["b"])) for w in
         json.load(io.open(MAP, encoding="utf-8"))["walls"]]
GOALS = {k: tuple(v) for k, v in
         json.load(io.open(MAP, encoding="utf-8"))["goals_mm"].items()}
GOALS["OPEN1"] = (2700, 750)

CAR_HALF_L = 160.0                 # 车轴到前/后保险杠
CAR_HALF_W = 120.0                 # 车轴到车身两侧


def seg_dist(px, py, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    d2 = dx * dx + dy * dy
    t = 0.0 if d2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / d2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def pose_point(x, y, hdg, fwd, lat):
    """车体局部坐标(fwd 前向, lat 左正) -> 世界坐标."""
    fx, fy = math.sin(math.radians(hdg)), math.cos(math.radians(hdg))
    sx, sy = -math.cos(math.radians(hdg)), math.sin(math.radians(hdg))
    return (x + fwd * fx + lat * sx, y + fwd * fy + lat * sy)


def seg_int(p1, p2, q1, q2):
    x1, y1 = p1; x2, y2 = p2; x3, y3 = q1; x4, y4 = q2
    d = (x2-x1)*(y4-y3) - (y2-y1)*(x4-x3)
    if abs(d) < 1e-12:
        return None
    tt = ((x3-x1)*(y4-y3) - (y3-y1)*(x4-x3)) / d
    uu = ((x3-x1)*(y2-y1) - (y3-y1)*(x2-x1)) / d
    if -1e-9 <= tt <= 1 + 1e-9 and -1e-9 <= uu <= 1 + 1e-9:
        return (tt, uu)
    return None

def seg_seg_dist(p1, p2, q1, q2):
    if seg_int(p1, p2, q1, q2):
        return 0.0
    return min(seg_dist(p1[0], p1[1], q1, q2), seg_dist(p2[0], p2[1], q1, q2),
               seg_dist(q1[0], q1[1], p1, p2), seg_dist(q2[0], q2[1], p1, p2))

def car_edges(x, y, hdg):
    c = [pose_point(x, y, hdg, f, l) for f, l in ((-CAR_HALF_L, -CAR_HALF_W), (CAR_HALF_L, -CAR_HALF_W),
                                                  (CAR_HALF_L, CAR_HALF_W), (-CAR_HALF_L, CAR_HALF_W))]
    return [(c[0], c[1]), (c[1], c[2]), (c[2], c[3]), (c[3], c[0])]

def min_clearance(x, y, hdg):
    best = 1e9
    for (a, b) in car_edges(x, y, hdg):
        for (w1, w2) in WALLS:
            d = seg_seg_dist(a, b, w1, w2)
            if d < best:
                best = d
    return best


def simulate(legs, start=(2700.0, 300.0), hdg0=0.0, rng=None):
    rng = rng or random.Random(1)
    x, y, hdg = start[0], start[1], hdg0
    min_clr = 1e9
    for leg in legs:
        if leg["type"] == "turn":
            target = hdg + leg["deg"]
            err = rng.gauss(0.0, 0.8) + rng.uniform(-0.3, 0.3)
            hdg = target + err
            # 原地转（绕车轴）：按 8 个角度采样扫掠车体
            for k in range(9):
                h = hdg0 + (hdg - hdg0) * k / 8.0
                d = min_clearance(x, y, h)
                if d < min_clr:
                    min_clr = d
        else:
            dist = float(leg["mm"])
            actual = dist * (1.0 + rng.gauss(0.0, 0.008)) + rng.gauss(0.0, 3.0)
            steps = max(2, int(abs(actual) / 25.0))
            # 航向保持误差：每段一个固定偏差（σ=0.6°），不做无界随机游走
            hdg_leg = hdg + rng.gauss(0.0, 0.6)
            hdg = hdg_leg
            for s in range(steps):
                fx, fy = math.sin(math.radians(hdg_leg)), math.cos(math.radians(hdg_leg))
                x += actual / steps * fx
                y += actual / steps * fy
                c = min_clearance(x, y, hdg_leg)
                if c < min_clr:
                    min_clr = c
    return x, y, hdg, min_clr


def route_targets(name):
    base = name.replace("_plan", "")
    if base.startswith("route1"):
        return ["B"]
    if base.startswith("route2"):
        return ["C"]
    if base.startswith("route3"):
        return ["D"]
    if base.startswith("route4"):
        return ["E"]
    if base.startswith("route5"):
        return ["E"]
    if base.startswith("route6"):
        return ["OPEN1"]
    return []


def main():
    random.seed(7)
    rows = []
    detail = []
    names = [f for f in sorted(os.listdir(TOOLS))
             if f.startswith("route") and f.endswith(".json")]
    for name in names:
        path = os.path.join(TOOLS, name)
        try:
            legs = json.load(io.open(path, encoding="utf-8"))["legs"]
        except Exception:
            continue
        tg = route_targets(name)
        if not tg:
            continue
        target = GOALS[tg[-1]]
        n_turn = sum(1 for l in legs if l["type"] == "turn")
        total = sum(l["mm"] for l in legs if l["type"] == "drive")
        finals = []
        clrs = []
        for i in range(RUNS):
            x, y, h, clr = simulate(legs, rng=random.Random(1000 + i))
            err = math.hypot(x - target[0], y - target[1])
            finals.append(err)
            clrs.append(clr)
        finals.sort()
        med = finals[len(finals) // 2]
        p95 = finals[int(len(finals) * 0.95)]
        min_clr = min(clrs)
        verdict = "PASS" if (p95 <= 150 and min_clr >= 40) else "FAIL"
        rows.append((name, total, n_turn, med, p95, min_clr, verdict))
        detail.append((name, legs))
        print("%-28s total=%5.0f turns=%2d  err_med=%6.1f p95=%6.1f  min_clr=%6.1f  %s"
              % (name, total, n_turn, med, p95, min_clr, verdict))

    lines = ["# 路线仿真报告（每段执行的可达性/误差验证）", "",
             "模型：转向误差 σ=0.8°+0.3° 量化；直行增益误差 σ=0.8% + 3 mm/段；",
             "行驶中航向保持 σ=0.6°/段；车体 320×240 mm 沿路径每 25 mm 采样做墙距检查；",
             "起点 A=(2700,300) 朝向 +Y；每条路线 %d 次蒙特卡洛。" % RUNS, "",
             "| 路线 | 总里程(mm) | 转向次数 | 终点误差中值(mm) | P95(mm) | 最小墙距(mm) | 判定 |",
             "| --- | --- | --- | --- | --- | --- | --- |"]
    for (name, total, n_turn, med, p95, clr, verdict) in rows:
        lines.append("| %s | %.0f | %d | %.1f | %.1f | %.1f | **%s** |" %
                     (name, total, n_turn, med, p95, clr, verdict))
    lines += ["", "## 说明", "",
              "- `min_clr` 是仿真轨迹上四角到任意墙段的最小距离；<40 mm 视为碰撞。",
              "- 直线到点路线（未规划版本）按地图会穿墙，因此判定 FAIL 属于预期。",
              "- 误差为纯运动学模型，未包含轮子打滑/地面差异；场地实测应在同一量级。"]
    io.open(OUT_MD, "w", encoding="utf-8", newline="\n").write("\n".join(lines) + "\n")
    print("\nreport ->", OUT_MD)


if __name__ == "__main__":
    main()

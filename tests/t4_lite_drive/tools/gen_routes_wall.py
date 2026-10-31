# -*- coding: utf-8 -*-
"""Generate wall-triggered stage routes (A->C/D/E) for the junction-square arena.

Search space: pose = (x, y, heading). Action = "drive straight until the ToF gap to the next wall
is STOP_GAP" (this snaps the position to a real wall, exactly like the field runner does), then
an optional +-90 deg arc turn (validated with the full car footprint). BFS over those actions.
Every action is checked for clearance; the best plan is written as a stage JSON.
"""
import collections
import io
import json
import math
import sys

sys.path.insert(0, r"D:\Gowin\Project\Contest\Embodied_RPACS\work")
import route_sim as S

ROOT = r"D:\Gowin\Project\Contest\Embodied_RPACS\Embodied_RPACS\Embodied_RPACS"
TOOLS = ROOT + r"\tests\t4_lite_drive\tools"
MAP = ROOT + r"\tests\task4_navigation\data\map_v1.json"

STOP_GAP = 180.0
FINAL_GAP = 200.0
ARC_R = 200.0
ARC_MIN_CLR = 20.0
STRAIGHT_MIN_CLR = 60.0
WHEEL_FRONT = 295.0
WHEEL_REAR = 25.0
WALLS = None


def load():
    global WALLS
    m = json.load(io.open(MAP, encoding="utf-8"))
    WALLS = [(tuple(w["a"]), tuple(w["b"])) for w in m["walls"]]
    return m


def ray_dist(x, y, hdg, maxd=4000.0):
    """distance from (x,y) along heading to the first wall (ignoring the wall behind)."""
    fx, fy = math.sin(math.radians(hdg)), math.cos(math.radians(hdg))
    best = maxd
    for a, b in WALLS:
        ax, ay = a
        ex, ey = b[0] - ax, b[1] - ay
        den = fx * ey - fy * ex
        if abs(den) < 1e-12:
            continue
        t = ((ax - x) * ey - (ay - y) * ex) / den
        u = ((ax - x) * fy - (ay - y) * fx) / den
        if t > 1.0 and -1e-9 <= u <= 1 + 1e-9 and t < best:
            best = t
    return best


def straight_ok(p, q, hdg):
    dx, dy = q[0] - p[0], q[1] - p[1]
    dist = math.hypot(dx, dy)
    if dist < 1.0:
        return True
    steps = max(2, int(dist / 20))
    for s in range(steps + 1):
        t = s / steps
        if S.min_clearance(p[0] + dx * t, p[1] + dy * t, hdg) < STRAIGHT_MIN_CLR:
            return False
    return True


def arc_clear(x, y, hdg0, turn, R=ARC_R, n=60):
    best = 1e9
    for k in range(n + 1):
        ang = turn * k / n
        h = hdg0 + ang
        if turn > 0:
            ex = x + R * (math.cos(math.radians(hdg0)) - math.cos(math.radians(hdg0 + ang)))
            ey = y + R * (math.sin(math.radians(hdg0 + ang)) - math.sin(math.radians(hdg0)))
        else:
            ex = x - R * (math.cos(math.radians(hdg0)) - math.cos(math.radians(hdg0 + ang)))
            ey = y - R * (math.sin(math.radians(hdg0 + ang)) - math.sin(math.radians(hdg0)))
        c = S.min_clearance(ex, ey, h)
        if c < best:
            best = c
    return best


def front_of(pose):
    x, y, hdg = pose
    return (x + WHEEL_FRONT * math.sin(math.radians(hdg)),
            y + WHEEL_FRONT * math.cos(math.radians(hdg)))


def drive_to_wall(pose, gap=STOP_GAP):
    x, y, hdg = pose
    d = ray_dist(x, y, hdg)
    if d > 3900:
        return None
    target = d - WHEEL_FRONT - gap          # 轴心到墙的距离 → 落到这个位置
    if target <= 5:
        return None
    q = (x + target * math.sin(math.radians(hdg)),
         y + target * math.cos(math.radians(hdg)), hdg)
    if S.min_clearance(q[0], q[1], hdg) < 40:
        return None
    if not straight_ok(pose, q, hdg):
        return None
    return q


def search(start, goal_center, max_depth=14):
    """BFS: ((pose, plan)) ; plan = list of ('wall', gap) / ('arc', deg)."""
    goals = goal_center
    q = collections.deque()
    q.append((start, []))
    seen = {start}

    def goal_extra(pose):
        fx, fy = front_of(pose)
        dx, dy = goals[0] - fx, goals[1] - fy
        h = math.radians(pose[2])
        fwd = dx * math.sin(h) + dy * math.cos(h)
        lat = abs(-dx * math.cos(h) + dy * math.sin(h))
        if lat <= 120 and 0 <= fwd <= 400:
            return [("drive", int(round(fwd)))] if fwd > 10 else []
        return None

    while q:
        pose, plan = q.popleft()
        if len(plan) >= max_depth:
            continue
        # 直行到墙（墙触发）→ 也可以在这里到达目标
        q1 = drive_to_wall(pose)
        if q1 is None:
            continue
        ex = goal_extra(q1)
        if ex is not None:
            return plan + [("wall", int(round(STOP_GAP)))] + ex
        npl = plan + [("wall", int(round(STOP_GAP)))]
        key1 = (round(q1[0] / 50), round(q1[1] / 50), int(q1[2]) % 360 // 90)
        if key1 not in seen:
            seen.add(key1)
            q.append((q1, npl))
        # 在墙前做 ±90 弧线转向
        for deg in (-90, 90):
            c = arc_clear(q1[0], q1[1], q1[2], deg)
            if c < ARC_MIN_CLR:
                continue
            if deg > 0:
                ex0 = q1[0] + ARC_R * (math.cos(math.radians(q1[2])) - math.cos(math.radians(q1[2] + deg)))
                ey0 = q1[1] + ARC_R * (math.sin(math.radians(q1[2] + deg)) - math.sin(math.radians(q1[2])))
            else:
                ex0 = q1[0] - ARC_R * (math.cos(math.radians(q1[2])) - math.cos(math.radians(q1[2] + deg)))
                ey0 = q1[1] - ARC_R * (math.sin(math.radians(q1[2] + deg)) - math.sin(math.radians(q1[2])))
            pose2 = (round(ex0, 1), round(ey0, 1), q1[2] + deg)
            key = (round(pose2[0] / 50), round(pose2[1] / 50), int(pose2[2]) % 360 // 90)
            if key in seen:
                continue
            seen.add(key)
            nxt_plan = npl + [("arc", deg)]
            ex2 = goal_extra(pose2)
            if ex2 is not None:
                return nxt_plan + ex2
            q.append((pose2, nxt_plan))
    return None


def main():
    m = load()
    centers = {k: tuple(v["centre_mm"]) for k, v in m["goal_regions_mm"].items()}
    start = (2400.0, 300.0, 0.0)   # A 区中心（轴心）
    out = {}
    for name in ("C", "D", "E"):
        plan = search(start, centers[name])
        if plan is None:
            print(name, "NO PLAN")
            continue
        stages = []
        for kind, val in plan:
            if kind == "wall":
                stages.append({"type": "wall", "gap": int(val), "speed": 20})
            elif kind == "drive":
                stages.append({"type": "drive", "mm": int(val)})
            else:
                stages.append({"type": "arc", "deg": val})
        out[name] = stages
        print(name, "stages:", len(stages))
        for i, st in enumerate(stages):
            print("   %2d %s" % (i, st))
    for name, stages in out.items():
        p = TOOLS + r"\route_A_to_%s_wall.json" % name
        io.open(p, "w", encoding="utf-8", newline="\n").write(
            json.dumps({"note": "A -> %s 按墙触发（方块路口法，图纸验算通过）" % name,
                        "stages": stages}, ensure_ascii=False, indent=2) + "\n")
        print("written", p)


if __name__ == "__main__":
    main()

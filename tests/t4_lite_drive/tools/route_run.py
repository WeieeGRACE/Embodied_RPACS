"""Route executor for t4_lite_drive: sequence of straight legs and gyro turns.

Legs (JSON or inline):
  {"type": "drive", "mm": 500}          # 直线，编码器执行，陀螺仪保持航向
  {"type": "turn",  "deg": -90}         # 原地转：正=左转(yaw+)，负=右转
  {"type": "drive", "mm": 500, "speed": 60}
Safety: per-leg progress/timeout aborts, fault abort, final STOP.

Usage: python3 route_run.py route.json [--speed 40]
"""
import io
import io
import json
import math
import os
import sys
import time

sys.path.insert(0, "/home/HwHiAiUser/t4_tools")
import t4_lite_drive as T

CAL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calib.json")
CAL = {}
if os.path.exists(CAL_PATH):
    try:
        CAL = json.load(io.open(CAL_PATH, encoding="utf-8"))
    except Exception:
        CAL = {}


def measure_wall():
    """返回 (相机墙距, 车头墙距)。"""
    import t4_lite_drive as TD
    med, _mn, _st = TD.measure_wall_distance(frames=10)
    return med, med - TD.CAM_OFFSET_MM


WRAP = 18000  # cdeg


class Ctl:
    def __init__(self, speed=40):
        self.link = T.Link()
        self.speed = speed
        self.last = None
        self.yaw_cont = 0.0
        self.last_wrap = None

    def pump(self, timeout=0.04):
        got = self.link.poll(timeout)
        if got:
            self.last = got[-1]
            self.unwrap(self.last.yaw_cdeg)
        return self.last

    def unwrap(self, w):
        w = int(w)
        if self.last_wrap is None:
            self.last_wrap = w
            return self.yaw_cont
        d = w - self.last_wrap
        if d > WRAP:
            d -= 2 * WRAP
        elif d < -WRAP:
            d += 2 * WRAP
        self.yaw_cont += d
        self.last_wrap = w
        return self.yaw_cont

    @property
    def yaw_deg(self):
        return self.yaw_cont / 100.0

    def bring_up(self):
        for _ in range(25):
            self.link.heartbeat()
            self.pump(0.02)
        self.link.set_limit(300)
        for _ in range(40):
            self.link.heartbeat()
            self.link.clear_fault()
            self.pump(0.02)
            if self.last and (self.last.fault & ~0x20) == 0:
                break
        return self.last

    def rearm(self):
        """每个 leg 前清 STOP 锁存，否则下一段电机不响应。"""
        for _ in range(20):
            self.link.heartbeat()
            self.pump(0.02)
        for _ in range(30):
            self.link.heartbeat()
            self.link.clear_fault()
            self.pump(0.02)
            if self.last and (self.last.fault & ~0x20) == 0:
                break
        return self.last

    def stop(self):
        self.link.stop()
        for _ in range(15):
            self.link.heartbeat()
            self.pump(0.03)


def drive(ctl, mm, speed=None, tol=8.0, timeout=40.0):
    if hasattr(ctl, "rearm"):
        ctl.rearm()
    speed = speed or ctl.speed
    base = ctl.last.distance_mm
    yaw_ref = ctl.yaw_deg
    t0 = time.time()
    last_prog_t = t0
    last_dist = 0.0
    v = 0
    while True:
        now = time.time()
        elapsed = now - t0
        ctl.link.heartbeat()
        yaw_err = yaw_ref - ctl.yaw_deg
        corr = max(-20.0, min(20.0, 1.5 * yaw_err))
        scale = float(CAL.get("cmd_scale", 1.0) or 1.0)
        v = min(speed, 25 + elapsed * 60) * scale
        ctl.link.set_vel(int(v + corr), int(v - corr))
        if not ctl.pump(0.03):
            if now - t0 > 1.0 and time.time() - getattr(ctl, "t_last_telem", 0) > 0.5:
                print("  TELEM lost")
                return False
        dist = ctl.last.distance_mm - base
        if abs(dist - last_dist) >= 2.0:
            last_dist = dist
            last_prog_t = now
        if dist >= mm - tol:
            ctl.link.stop()
            for _ in range(20):
                ctl.link.heartbeat()
                ctl.pump(0.03)
            print("  drive %.0f -> %.1f mm  yaw_err=%.2f" %
                  (mm, ctl.last.distance_mm - base, yaw_err))
            return True
        if now - last_prog_t > 1.6 and elapsed > 2.0:
            print("  drive STALL at %.1f / %.0f" % (dist, mm))
            ctl.link.stop()
            return False
        if elapsed > timeout:
            print("  drive TIMEOUT at %.1f / %.0f" % (dist, mm))
            ctl.link.stop()
            return False


def reverse(ctl, mm, speed=None, tol=10.0, timeout=25.0):
    """倒车：与 drive 相反方向，按编码器里程执行。"""
    speed = speed or ctl.speed
    if hasattr(ctl, "rearm"):
        ctl.rearm()
    mm = abs(float(mm))
    base = ctl.last.distance_mm
    t0 = time.time()
    last_prog = t0
    last_dist = 0.0
    while True:
        now = time.time()
        elapsed = now - t0
        ctl.link.heartbeat()
        scale = float(CAL.get("cmd_scale", 1.0) or 1.0)
        v = min(speed, 25 + elapsed * 60) * scale
        ctl.link.set_vel(int(-v), int(-v))
        ctl.pump(0.03)
        dist = base - ctl.last.distance_mm          # 倒车时里程减少
        if abs(dist - last_dist) >= 2.0:
            last_dist = dist
            last_prog = now
        if dist >= mm - tol:
            ctl.link.stop()
            for _ in range(15):
                ctl.link.heartbeat()
                ctl.pump(0.03)
            print("  reverse %.0f -> %.1f mm" % (mm, base - ctl.last.distance_mm))
            return True
        if now - last_prog > 1.6 and elapsed > 2.0:
            print("  reverse STALL at %.1f / %.0f" % (dist, mm))
            ctl.link.stop()
            return False
        if elapsed > timeout:
            ctl.link.stop()
            print("  reverse TIMEOUT at %.1f / %.0f" % (dist, mm))
            return False


def arc_turn(ctl, deg, radius=None, v=None, tol=4.0, timeout=25.0):
    """边走边转 90 度：绕半径 R 的圆弧，用左右轮差速实现（600 mm 走廊可过，余量约 65 mm）。

    v_l = v*(1 - track/(2R)), v_r = v*(1 + track/(2R))；yaw 速率 = v/R。
    参数可在 calib.json 调：arc_radius（默认 200）、arc_v（默认 60）、wheel_track 默认 240。
    """
    if hasattr(ctl, "rearm"):
        ctl.rearm()
    if radius is None:
        radius = float(CAL.get("arc_radius", 200.0))
    if v is None:
        v = float(CAL.get("arc_v", 60.0))
    track = float(CAL.get("wheel_track", 240.0))
    sign = 1.0 if deg >= 0 else -1.0
    ratio = track / (2.0 * radius)
    slow = v * (1.0 - ratio)
    fast = v * (1.0 + ratio)
    # 实测约定：cmd(vl>vr) -> yaw 增大。左转(yaw+) => vl=fast, vr=slow。
    if sign > 0:
        cmd_l, cmd_r = fast, slow
    else:
        cmd_l, cmd_r = slow, fast
    target = ctl.yaw_deg + deg
    base = ctl.last.distance_mm
    arc_len = abs(math.radians(deg)) * radius
    limit = max(600.0, 2.5 * arc_len + 200.0)
    t0 = time.time()
    while True:
        ctl.link.heartbeat()
        rem = target - ctl.yaw_deg
        if rem * sign <= 1.0:
            break
        travelled = abs(ctl.last.distance_mm - base)
        if travelled > limit:
            ctl.link.stop()
            print("  arc ABORT: travelled %.0f mm > limit %.0f (wrong direction?)" %
                  (travelled, limit))
            return False
        ctl.link.set_vel(int(cmd_l), int(cmd_r))
        ctl.pump(0.025)
        if time.time() - t0 > timeout:
            ctl.link.stop()
            print("  arc TIMEOUT err=%.1f" % rem)
            return False
    ctl.link.stop()
    tw = time.time()
    while time.time() - tw < 0.4:
        ctl.link.heartbeat()
        ctl.pump(0.03)
    err = target - ctl.yaw_deg
    print("  arc %.0f -> yaw %+.2f (err %.2f, R=%.0f)" % (deg, ctl.yaw_deg, err, radius))
    return abs(err) <= tol


def three_point(ctl, deg, v_arc=None, diff=None, fracs=(0.45, 0.35, 0.20),
                tol=4.0, timeout=30.0):
    """三点掉头：前进弧 → 倒车弧 → 前进弧，绕轮轴转向（走廊里比原地转省空间）。

    三段旋转方向相同（左转 yaw+ 时：前进时 vl>vr；倒车时 vl=(-v+diff), vr=(-v-diff)）。
    每段按"目标航向增量"闭环停止，参数可在 calib.json 里调：
      tp_v（弧线基础速度）、tp_diff（左右轮差速）、tp_fracs（三段分配比例）。
    """
    if hasattr(ctl, "rearm"):
        ctl.rearm()
    if v_arc is None:
        v_arc = float(CAL.get("tp_v", 40))
    if diff is None:
        diff = float(CAL.get("tp_diff", 40))
    if CAL.get("tp_fracs"):
        fracs = tuple(CAL["tp_fracs"])
    target = ctl.yaw_deg + deg
    sign = 1.0 if deg >= 0 else -1.0
    t0 = time.time()
    for phase, frac in enumerate(fracs):
        seg_goal = ctl.yaw_deg + sign * abs(deg) * frac
        while True:
            ctl.link.heartbeat()
            rem = seg_goal - ctl.yaw_deg
            if rem * sign <= 1.0:
                break
            if phase == 1:   # 倒车弧
                vl = -v_arc + sign * diff
                vr = -v_arc - sign * diff
            else:            # 前进弧
                vl = v_arc + sign * diff
                vr = v_arc - sign * diff
            ctl.link.set_vel(int(vl), int(vr))
            ctl.pump(0.025)
            if time.time() - t0 > timeout:
                ctl.link.stop()
                print("  threepoint TIMEOUT err=%.1f" % (target - ctl.yaw_deg))
                return False
        ctl.link.stop()
        tw = time.time()
        while time.time() - tw < 0.35:
            ctl.link.heartbeat()
            ctl.pump(0.03)
        # 每段 STOP 会锁存，下一段必须先重新使能，否则电机不响应
        if hasattr(ctl, "rearm"):
            ctl.rearm()
    err = target - ctl.yaw_deg
    print("  threepoint %.0f -> yaw %+.2f (err %.2f)" % (deg, ctl.yaw_deg, err))
    return abs(err) <= tol


def turn(ctl, deg, tol=5.0, mag=None, timeout=20.0, latency=0.18):
    if mag is None:
        mag = float(CAL.get("turn_mag", 70))
    latency = float(CAL.get("turn_latency", latency))
    if hasattr(ctl, "rearm"):
        ctl.rearm()
    """转到 target：先等转速稳定（|rate|>40 deg/s），再按 rate*latency 提前停。"""
    target = ctl.yaw_deg + deg
    t0 = time.time()
    for attempt in range(3):
        err = target - ctl.yaw_deg
        if abs(err) <= tol:
            print("  turn %.0f -> yaw %+.2f (err %.2f, attempt %d)" %
                  (deg, ctl.yaw_deg, err, attempt))
            return True
        sign = 1.0 if err > 0 else -1.0
        y_prev, t_prev = ctl.yaw_deg, time.time()
        rate = 0.0
        while True:
            ctl.link.heartbeat()
            ctl.link.set_vel(int(mag) if sign > 0 else int(-mag),
                             int(-mag) if sign > 0 else int(mag))
            ctl.pump(0.02)
            now = time.time()
            if now - t_prev >= 0.2:
                rate = (ctl.yaw_deg - y_prev) / (now - t_prev)
                y_prev, t_prev = ctl.yaw_deg, now
            rem = target - ctl.yaw_deg
            lead = abs(rate) * latency
            if abs(rate) > 8.0 and sign * rem <= max(3.0, lead):
                break
            if time.time() - t0 > timeout:
                break
        ctl.link.stop()
        tw = time.time()
        while time.time() - tw < 0.8:
            ctl.link.heartbeat()
            ctl.pump(0.03)
    err = target - ctl.yaw_deg
    print("  turn %.0f -> yaw %+.2f (final err %.2f)" % (deg, ctl.yaw_deg, err))
    return abs(err) <= tol


def main():
    route_path = sys.argv[1]
    speed = 40
    if "--speed" in sys.argv:
        speed = float(sys.argv[sys.argv.index("--speed") + 1])
    route = json.load(open(route_path))
    legs = route["legs"] if isinstance(route, dict) else route
    ctl = Ctl(speed)
    if not ctl.bring_up():
        print("link bring-up failed")
        return 2
    print("start: dist=%.1f yaw=%.2f fault=0x%02X" %
          (ctl.last.distance_mm, ctl.yaw_deg, ctl.last.fault))
    ok = True
    for i, leg in enumerate(legs):
        print("leg %d: %s" % (i, leg))
        ctl.rearm()
        if leg["type"] == "drive":
            ok = drive(ctl, float(leg["mm"]), leg.get("speed"))
        elif leg["type"] == "turn":
            ok = turn(ctl, float(leg["deg"]))
        elif leg["type"] == "to_wall":
            margin = float(leg.get("margin", 120))
            cam, bumper = measure_wall()
            travel = bumper - margin
            print("  相机墙距 %.0f -> 车头距墙 %.0f -> 目标行程 %.0f" % (cam, bumper, travel))
            if travel < 80:
                print("  距离太小，跳过")
                ok = False
            else:
                ok = drive(ctl, travel, leg.get("speed"))
        else:
            print("  unknown leg type")
            ok = False
        if not ok:
            break
    ctl.stop()
    print("end: dist=%.1f yaw=%.2f fault=0x%02X" %
          (ctl.last.distance_mm, ctl.yaw_deg, ctl.last.fault))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())

"""Route executor for t4_lite_drive: sequence of straight legs and gyro turns.

Legs (JSON or inline):
  {"type": "drive", "mm": 500}          # 直线，编码器执行，陀螺仪保持航向
  {"type": "turn",  "deg": -90}         # 原地转：正=左转(yaw+)，负=右转
  {"type": "drive", "mm": 500, "speed": 60}
Safety: per-leg progress/timeout aborts, fault abort, final STOP.

Usage: python3 route_run.py route.json [--speed 40]
"""
import json
import sys
import time

sys.path.insert(0, "/home/HwHiAiUser/t4_tools")
import t4_lite_drive as T

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
        v = min(speed, 25 + elapsed * 60)
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


def turn(ctl, deg, tol=3.5, mag=70, timeout=20.0, latency=0.18):
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
            if abs(rate) > 40.0 and sign * rem <= max(3.0, lead):
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

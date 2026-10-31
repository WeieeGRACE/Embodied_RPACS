#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""closed_loop_calib.py —— 闭环位流的一次性上电标定（速度 + 转向）。

做三件事，写 calib.json 给 route_run.py 用：
  1) 50 mm/s 与 100 mm/s 直行 2 s：用编码器里程算实测平均速度，判断闭环是否跟得上；
  2) 右转/左转各 1 s（差速 ±TURN_MAG）：测角速度（deg/s）与 STOP 后滑行角；
  3) 输出 PASS/FAIL 表 + calib.json（cmd_scale / turn_mag / turn_rate / turn_lead）。

用法（Pi 上，车放在前方 ≥1.5 m 空旷处）：
  python3 ~/t4_tools/closed_loop_calib.py
"""
import json
import math
import os
import sys
import time

TOOLS = os.path.dirname(os.path.abspath(__file__))
if TOOLS not in sys.path:
    sys.path.insert(0, TOOLS)
import t4_lite_drive as T

TURN_MAG = 50          # 闭环下 50 mm/s 差速 ≈ 24 deg/s（track 240）
WRAP = 18000


class Cal:
    def __init__(self):
        self.link = T.Link()
        self.last = None
        self.yaw_cont = 0.0
        self.last_wrap = None

    def pump(self, timeout=0.03):
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
    def yaw(self):
        return self.yaw_cont / 100.0

    def up(self):
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

    def stop(self):
        self.link.stop()
        for _ in range(20):
            self.link.heartbeat()
            self.pump(0.03)


def speed_test(cal, cmd, seconds=2.0):
    cal.up()
    base = cal.last.distance_mm
    t0 = time.time()
    speeds = []
    while time.time() - t0 < seconds:
        cal.link.heartbeat()
        cal.link.set_vel(cmd, cmd)
        if not cal.pump(0.03):
            continue
        if time.time() - t0 > 0.5:
            speeds.append((cal.last.speed_l + cal.last.speed_r) / 2.0)
    cal.stop()
    dt = time.time() - t0
    dist = cal.last.distance_mm - base
    v_enc = dist / dt
    v_tele = sum(speeds) / len(speeds) if speeds else 0.0
    return {"cmd": cmd, "dist_mm": dist, "seconds": dt, "v_enc": v_enc, "v_tele": v_tele}


def turn_test(cal, direction, mag=TURN_MAG, seconds=1.0):
    cal.up()
    y0 = cal.yaw
    t0 = time.time()
    while time.time() - t0 < seconds:
        cal.link.heartbeat()
        if direction > 0:
            cal.link.set_vel(mag, -mag)      # vl>vr -> yaw+
        else:
            cal.link.set_vel(-mag, mag)
        cal.pump(0.03)
    y1 = cal.yaw
    rate_start = time.time()
    cal.link.stop()
    while time.time() - rate_start < 0.8:
        cal.link.heartbeat()
        cal.pump(0.03)
    y2 = cal.yaw
    rate = (y1 - y0) / seconds
    coast = (y2 - y1)
    return {"dir": direction, "mag": mag, "rate_deg_s": rate, "coast_deg": coast}


def main():
    cal = Cal()
    if cal.up() is None:
        print("FAIL: 收不到 TELEM（检查位流/网线）")
        return 2
    print("[calib] 链路在线 enc=(%d,%d)" % (cal.last.enc_l, cal.last.enc_r))

    res = {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "bitstream_hint": "closed-loop PID"}
    print("\n== 直行速度 ==")
    s50 = speed_test(cal, 50)
    s100 = speed_test(cal, 100)
    for r in (s50, s100):
        print("  cmd=%3d mm/s  实测 %.1f mm/s（遥测 %.1f）  2s 里程 %.1f mm" %
              (r["cmd"], r["v_enc"], r["v_tele"], r["dist_mm"]))
    res["speed"] = {"50": s50, "100": s100}
    res["cmd_scale"] = round(s50["cmd"] / s50["v_enc"], 4) if s50["v_enc"] > 5 else 1.0
    ok50 = abs(s50["v_enc"] - 50) <= 10
    ok100 = abs(s100["v_enc"] - 100) <= 15

    print("\n== 转向 ==")
    r_neg = turn_test(cal, -1)
    r_pos = turn_test(cal, +1)
    for r in (r_neg, r_pos):
        print("  diff ±%d : %+.1f deg/s, STOP 后滑行 %+.1f deg" %
              (r["mag"], r["rate_deg_s"], r["coast_deg"]))
    rate = (abs(r_neg["rate_deg_s"]) + abs(r_pos["rate_deg_s"])) / 2.0
    coast = (abs(r_neg["coast_deg"]) + abs(r_pos["coast_deg"])) / 2.0
    res["turn"] = {"neg": r_neg, "pos": r_pos, "mag": TURN_MAG}
    res["turn_rate_deg_s"] = round(rate, 2)
    res["turn_coast_deg"] = round(coast, 2)
    res["turn_lead"] = round(max(2.0, rate * 0.18), 2)
    ok_turn = 15.0 <= rate <= 40.0 and coast < 15.0

    calib = {
        "calib_time": res["time"],
        "cmd_scale": res["cmd_scale"],
        "turn_mag": TURN_MAG,
        "turn_rate_deg_s": res["turn_rate_deg_s"],
        "turn_coast_deg": res["turn_coast_deg"],
        "turn_lead": res["turn_lead"],
        "ok_speed_50": ok50, "ok_speed_100": ok100, "ok_turn": ok_turn,
    }
    path = os.path.join(TOOLS, "calib.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(calib, f, ensure_ascii=False, indent=2)
    print("\n== 标定结果 ==")
    print(json.dumps(calib, ensure_ascii=False, indent=2))
    print("written:", path)
    verdict = ok50 and ok100 and ok_turn
    print("\nVERDICT:", "PASS — 可以按分段测试计划开始" if verdict else
          "CHECK — 某一项偏差大，先看 data（速度/转向）再决定是否换开环位流")
    return 0 if verdict else 1


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""t4_autonav.py —— Pi 端自主执行（断网可用）：一次下发整条按墙触发路线，Pi 自己跑完。

特点：
  * 只用 UDP 遥测 + ToF，不依赖 PC / WiFi 连接（SSH 断了也继续跑）
  * 两段速：离目标墙 > slow_gap 时用 fast 速度，接近后切 slow
  * 精简使能、ToF 触发，不再每步开相机（相机只做可选校验）
  * 每步写本地日志 ~/t4_logs/autonav_<stamp>.jsonl

用法（Pi 上一条命令启动后可断开网络）：
  setsid nohup python3 ~/t4_tools/t4_autonav.py --route route_A_to_B_wall.json \
        --fast 55 --slow 20 --slow-gap 400 > ~/t4_logs/autonav.out 2>&1 &
  tail -f ~/t4_logs/autonav.out          # 看进度（可选）
  python3 ~/t4_tools/t4_autonav.py --route-file file.json --status   # 查看已有日志

阶段格式与 t4_wallrun 相同：
  {"type":"wall","gap":180,"speed":20,"slow_gap":400}
  {"type":"arc","deg":-90}
  {"type":"drive","mm":300}
安全检查：ToF < 70 mm 立即停车并终止路线。
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
import t4_lite_drive as T

TOF_OFF = 30.0
LOG_DIR = os.path.expanduser("~/t4_logs")


class Nav:
    def __init__(self, fast=55, slow=20, slow_gap=400, arc_v=50, arc_radius=200):
        self.link = T.Link()
        self.last = None
        self.fast = fast
        self.slow = slow
        self.slow_gap = slow_gap
        self.arc_v = arc_v
        self.arc_r = arc_radius
        self.yaw = 0.0
        self.last_wrap = None
        self.events = []
        self.logf = None

    # ---------- 基础 ----------
    def pump(self, timeout=0.02):
        got = self.link.poll(timeout)
        if got:
            self.last = got[-1]
            w = int(self.last.yaw_cdeg)
            if self.last_wrap is None:
                self.last_wrap = w
            else:
                d = w - self.last_wrap
                if d > 18000:
                    d -= 36000
                elif d < -18000:
                    d += 36000
                self.yaw += d / 100.0
                self.last_wrap = w
        return self.last

    def log(self, ev):
        rec = {"t": round(time.time(), 3), "yaw": round(self.yaw, 2)}
        rec.update(ev)
        self.events.append(rec)
        if self.logf:
            self.logf.write(json.dumps(rec, ensure_ascii=False) + "\n")
            self.logf.flush()
        print(json.dumps(rec, ensure_ascii=False), flush=True)

    def tof(self):
        if self.last is None:
            return None
        v = getattr(self.last, "dbg_a", None)
        if v is None or not (40 < v < 3000):
            return None
        return v

    def rearm(self):
        """精简使能：心跳 + 清故障，最多 8 轮。"""
        for _ in range(8):
            self.link.heartbeat()
            self.link.clear_fault()
            self.pump(0.02)
            if self.last and (self.last.fault & ~0x20) == 0:
                return True
        return False

    def stop(self):
        self.link.stop()
        for _ in range(4):
            self.link.heartbeat()
            self.pump(0.02)

    # ---------- 动作 ----------
    def drive_until(self, target_gap):
        """前进直到 ToF 车头距 target_gap；两段速。返回 (ok, gap)。"""
        ok = self.rearm()
        t0 = time.time()
        tof = self.tof()
        # ToF 无效时也要缓慢前进，等它读到有效值再按两段速控制
        while time.time() - t0 < 120:
            gap = (tof - TOF_OFF) if tof is not None else None
            if gap is not None and gap <= target_gap:
                break
            if gap is not None and gap < 70:
                self.stop()
                self.log({"ev": "ABORT_CLOSE", "tof": tof})
                return False, gap
            v = self.slow if gap is None else (self.fast if gap > self.slow_gap else self.slow)
            self.link.set_vel(v, v)
            self.pump(0.02)
            self.link.heartbeat()
            tof = self.tof()
        self.stop()
        gap = (tof - TOF_OFF) if tof else None
        self.log({"ev": "wall_stop", "gap": None if gap is None else round(gap, 1), "ok": True})
        return True, gap

    def drive(self, mm):
        ok = self.rearm()
        base = self.last.distance_mm if self.last else 0
        t0 = time.time()
        while time.time() - t0 < 60:
            self.link.set_vel(self.fast, self.fast)
            self.pump(0.02)
            self.link.heartbeat()
            if abs(self.last.distance_mm - base) >= mm - 5:
                break
        self.stop()
        self.log({"ev": "drive", "mm": mm, "ok": ok})
        return ok

    def reverse(self, mm):
        ok = self.rearm()
        base = self.last.distance_mm if self.last else 0
        t0 = time.time()
        while time.time() - t0 < 40:
            self.link.set_vel(-self.fast, -self.fast)
            self.pump(0.02)
            self.link.heartbeat()
            moved = base - self.last.distance_mm
            if moved >= mm - 5:
                break
        self.stop()
        self.log({"ev": "reverse", "mm": mm, "moved": round(base - self.last.distance_mm, 1), "ok": True})
        return True

    def arc(self, deg):
        ok = self.rearm()
        sign = 1.0 if deg >= 0 else -1.0
        ratio = 240.0 / (2.0 * self.arc_r)
        slow = self.arc_v * (1 - ratio)
        fast = self.arc_v * (1 + ratio)
        cmd_l, cmd_r = (fast, slow) if sign > 0 else (slow, fast)
        target = self.yaw + deg
        base = self.last.distance_mm if self.last else 0
        limit = max(600.0, 2.5 * abs(deg) * 3.1416 / 180 * self.arc_r + 200)
        t0 = time.time()
        while time.time() - t0 < 30:
            self.link.heartbeat()
            if (target - self.yaw) * sign <= 1.0:
                break
            if abs(self.last.distance_mm - base) > limit:
                self.stop()
                self.log({"ev": "arc_abort", "deg": deg})
                return False
            self.link.set_vel(int(cmd_l), int(cmd_r))
            self.pump(0.02)
        self.stop()
        err = target - self.yaw
        self.log({"ev": "arc", "deg": deg, "err": round(err, 2), "ok": abs(err) <= 5.0})
        return abs(err) <= 5.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--route", required=True)
    ap.add_argument("--fast", type=int, default=55)
    ap.add_argument("--slow", type=int, default=20)
    ap.add_argument("--slow-gap", type=float, default=400)
    ap.add_argument("--arc-v", type=int, default=50)
    ap.add_argument("--from", dest="frm", type=int, default=0)
    ap.add_argument("--once", action="store_true", help="只跑 --from 指定的一个阶段")
    args = ap.parse_args()

    route = json.load(io.open(os.path.join(TOOLS, args.route), encoding="utf-8"))
    stages = route["stages"]
    os.makedirs(LOG_DIR, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    logpath = os.path.join(LOG_DIR, "autonav_%s.jsonl" % stamp)

    nav = Nav(args.fast, args.slow, args.slow_gap, args.arc_v)
    nav.logf = io.open(logpath, "w", encoding="utf-8")
    nav.log({"ev": "start", "route": args.route, "stages": len(stages),
             "fast": args.fast, "slow": args.slow})
    nav.rearm()
    end = args.frm + 1 if args.once else len(stages)
    t0 = time.time()
    rc = 0
    try:
        for i in range(args.frm, end):
            st = stages[i]
            nav.log({"ev": "stage", "i": i, "st": st})
            if st["type"] == "wall":
                ok, gap = nav.drive_until(float(st.get("gap", 180)))
            elif st["type"] == "arc":
                ok = nav.arc(float(st["deg"]))
            elif st["type"] == "reverse":
                ok = nav.reverse(float(st["mm"]))
            elif st["type"] == "drive":
                ok = nav.drive(float(st["mm"]))
            else:
                ok = True
            if not ok:
                nav.log({"ev": "stage_fail", "i": i})
                rc = 1
                break
    finally:
        nav.stop()
        nav.log({"ev": "done", "elapsed_s": round(time.time() - t0, 1), "rc": rc})
        nav.logf.close()
    print("LOG:", logpath)
    return rc


if __name__ == "__main__":
    raise SystemExit(main())

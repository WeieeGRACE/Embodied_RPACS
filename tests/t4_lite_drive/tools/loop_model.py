#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""t4_lite_drive 速度环整定模型（RTL 控制律的逐窗口整数复刻）。

用途：在跑 125 MHz RTL 仿真之前，先用 Python 快速扫描 KP/KI/速度窗口，
确认 SET_VEL(50) 能稳定到 ~50 mm/s，并检查 ±50 差速原地转。

复刻的 RTL 行为（与 src/ 下文件逐项对应）：
  * speed_measurement_and_velocity_filter：窗口 delta 累加，FILTER_LENGTH=4 滑动平均
    （向 0 截断），窗口输出 counts/窗口；
  * t4_wheel_speed_loop：mm/s -> counts/窗口 的 Q24 常数倒数（四舍五入）；
  * dual_speed_pi_pid_controller：增量式 PI，control += (KP*de + KI*e) >> GAIN_SHIFT，
    带条件积分（饱和且误差向外时去掉 I 项）与输出限幅；
  * pwm_and_startup_ramp：每 PWM 周期 duty 最多变化 RAMP_STEP=1，换向先回 0；
  * 被控对象：一阶电机 v'=(v_target-v)/TAU，v_target = GAIN*(|duty|-DEAD)（含静摩擦死区）。
"""
import math

CLK_HZ = 125_000_000
PWM_HZ = 20_000
PWM_PERIOD = CLK_HZ // PWM_HZ            # 6250 tick
PERIOD_S = 1.0 / PWM_HZ                  # 50 us
UM_PER_REV = 204203
COUNTS_PER_REV = 1456
UM_PER_COUNT = UM_PER_REV / COUNTS_PER_REV   # 140.25 um/count
DUTY_LIMIT = PWM_PERIOD // 2             # 3125 (50%)


def target_counts(v_mm_s, window_ms, counts_per_rev=COUNTS_PER_REV, um_per_rev=UM_PER_REV):
    """RTL: TARGET_RECIP_Q24 = round(2^24*COUNTS_PER_REV*UM_PER_WINDOW_PER_MMPS/UM_PER_REV)."""
    um_per_window = (CLK_HZ // 1000) * window_ms / CLK_HZ * 1_000_000  # um per window per mm/s
    # RTL integer: WINDOW_CYCLES*1000/CLK_HZ
    um_per_window = (CLK_HZ // 1000 * window_ms * 1000) // CLK_HZ
    recip = (2 ** 24 * counts_per_rev * um_per_window + um_per_rev // 2) // um_per_rev
    mag = abs(v_mm_s)
    prod = (mag * recip + 2 ** 23) >> 24
    prod = min(prod, 32767)
    return -prod if v_mm_s < 0 else prod


class Wheel:
    def __init__(self):
        self.v = 0.0            # true wheel speed mm/s
        self.dist_um = 0.0
        self.dut = 0
        self.direction = 0
        self.count_prev = 0
        self.samples = []
        self.speed_filt = 0
        self.control = 0
        self.e1 = 0
        self.e2 = 0

    def window_count(self):
        c = math.floor(self.dist_um / UM_PER_COUNT)
        delta = c - self.count_prev
        self.count_prev = c
        return delta


def run_scenario(target_l, target_r, windows, kp, ki, kd, window_ms,
                 wheel_gain=0.1222, wheel_dead=100, tau=0.05, log=False):
    wl, wr = Wheel(), Wheel()
    samples = []
    window_s = window_ms / 1000.0
    for w in range(windows):
        # ---- 一个速度窗口内：每 PWM 周期更新 duty 斜坡 + 被控对象
        cnt_l = wl.window_count()
        cnt_r = wr.window_count()
        for wheel, cnt in ((wl, cnt_l), (wr, cnt_r)):
            wheel.samples.append(cnt)
            if len(wheel.samples) > 4:
                wheel.samples.pop(0)
            ssum = sum(wheel.samples)
            # RTL 除法向 0 截断
            wheel.speed_filt = int(ssum / len(wheel.samples)) if ssum >= 0 else -int(-ssum / len(wheel.samples))
        # 目标换算用当前窗口参数
        tl = target_counts(target_l, window_ms)
        tr = target_counts(target_r, window_ms)
        # ---- PI 更新（每个窗口一拍，RTL control_tick）
        for wheel, tgt in ((wl, tl), (wr, tr)):
            e = tgt - wheel.speed_filt
            de = e - wheel.e1
            dd = e - 2 * wheel.e1 + wheel.e2
            p = kp * de
            i = ki * e
            d = kd * dd
            cand = wheel.control + (p + i + d)
            outward = (wheel.control >= DUTY_LIMIT and e > 0) or (wheel.control <= -DUTY_LIMIT and e < 0)
            if outward:
                cand = wheel.control + (p + d)
            wheel.control = max(-DUTY_LIMIT, min(DUTY_LIMIT, cand))
            wheel.e2 = wheel.e1
            wheel.e1 = e
        # ---- 400 个 PWM 周期：ramp + 被控对象 + 编码器距离
        for _ in range(PWM_HZ // 1000 * window_ms):
            for wheel, tgt in ((wl, wl.control), (wr, wr.control)):
                cmd = tgt
                mag = abs(cmd)
                cap = DUTY_LIMIT
                target_d = min(mag, cap)
                # pwm_and_startup_ramp 简化：换向先回 0，再按 RAMP_STEP=1 逼近
                if wheel.dut != 0 and target_d != 0 and (cmd < 0) != (wheel.direction == 1):
                    wheel.dut = max(0, wheel.dut - 1)
                    if wheel.dut == 0:
                        wheel.direction = 1 if cmd < 0 else 0
                else:
                    if wheel.dut == 0 and target_d != 0:
                        wheel.direction = 1 if cmd < 0 else 0
                    if wheel.dut < target_d:
                        wheel.dut = min(target_d, wheel.dut + 1)
                    elif wheel.dut > target_d:
                        wheel.dut = max(target_d, wheel.dut - 1)
                signed = -wheel.dut if wheel.direction else wheel.dut
                # 被控对象：一阶 + 静摩擦死区
                if abs(signed) > wheel_dead:
                    v_tgt = math.copysign(wheel_gain * (abs(signed) - wheel_dead), signed)
                else:
                    v_tgt = 0.0
                wheel.v += (v_tgt - wheel.v) * (PERIOD_S / tau)
                wheel.dist_um += wheel.v * PERIOD_S * 1000.0
        samples.append((w, tl, tr, wl.speed_filt, wr.speed_filt, wl.v, wr.v, wl.control, wr.control))
        if log:
            print("w=%2d tgt=%4d/%4d filt=%3d/%3d v=%7.2f/%7.2f ctrl=%5d/%5d"
                  % samples[-1])
    return samples


def avg(vals):
    return sum(vals) / len(vals)


def scenario_report(name, samples, skip, target_l, target_r):
    ls = [s[5] for s in samples[skip:]]
    rs = [s[6] for s in samples[skip:]]
    print("%-26s target=%4d/%4d  v_l=%7.2f mm/s  v_r=%7.2f mm/s  (last %d windows)"
          % (name, target_l, target_r, avg(ls), avg(rs), len(ls)))
    return avg(ls), avg(rs)


def turning_rate(v_l, v_r, track_mm):
    """yaw rate deg/s for in-place turn; dtheta/dt = (v_r - v_l)/track."""
    return (v_r - v_l) / track_mm * 57.2957795


if __name__ == "__main__":
    import sys
    kp = int(sys.argv[1]) if len(sys.argv) > 1 else 16
    ki = int(sys.argv[2]) if len(sys.argv) > 2 else 2
    window_ms = int(sys.argv[3]) if len(sys.argv) > 3 else 20
    kd = int(sys.argv[4]) if len(sys.argv) > 4 else 0
    print("KP=%d KI=%d KD=%d window=%d ms (DUTY_LIMIT=%d, RAMP_STEP=1/PWM period)"
          % (kp, ki, kd, window_ms, DUTY_LIMIT))
    cases = [
        ("forward 50 mm/s", 50, 50, 20, 10),
        ("forward 100 mm/s", 100, 100, 20, 12),
        ("forward 25 mm/s", 25, 25, 20, 12),
        ("reverse 50 mm/s", -50, -50, 20, 10),
        ("turn +50/-50", 50, -50, 20, 10),
        ("forward 300 mm/s", 300, 300, 30, 20),
        ("stop from 50", 0, 0, 8, 4),
    ]
    for name, tl, tr, nw, skip in cases:
        s = run_scenario(tl, tr, nw, kp, ki, kd, window_ms)
        vl, vr = scenario_report(name, s, skip, tl, tr)
        if name.startswith("turn"):
            print("    -> in-place yaw rate @240mm track = %.1f deg/s" % turning_rate(vl, vr, 240.0))
        # 最后 3 个窗口的波动
        if len(s) > skip + 3:
            tail = s[-3:]
            print("    tail v_l:", ["%.1f" % t[5] for t in tail], "ctrl:", [t[7] for t in tail])

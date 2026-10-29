# 90° 转向后故障锁存 — ModelSim 故障注入验证报告

版本：改动前基线（用户实际上板版本）
日期：2026-10-29
证据目录：`records/fault_injection/`
测试平台：`sim/tb_fault_turns.sv`（新增）；ModelSim SE-64 10.7

---

## 0. 结论速览

用**冻结的改动前基线**（git `0bd1a77`，其 `src/` 与 `3556192` 完全相同，见 §1）在 ModelSim 中复现了
“避障演示在 90° 转向完成后出现 FAULT 提示（OLED 显示 `FAULT RESET REQUIRED`）”的**全部 4 类锁存路径**，
每个场景都定位到具体信号、具体时刻与具体日志行：

| # | 机制（一句话） | 从注入到锁存 | 与用户描述的吻合度 |
|---|---|---|---|
| 1 | **转向后侧向 ToF 读到有效近距值 ≤85 mm（非 20）** → `obstacle_sensors.faults[4]` 立即锁存 | **+1 DUT-ms** | 最高：转向一结束立即故障，几何上必然发生、无需器件异常 |
| 2 | **转向后任一通道持续输出精确 20 mm（通信正常）** → 20 mm 被丢弃、`fresh` 永不置位 → WAIT_* barrier 的 `&post_turn` 永不满足 → 1000 ms 后 `route_fault=1 / reason=0x84` | **+999~1000 DUT-ms** | 高：转向完成后约 1 秒出现；两个转向场景均可触发 |
| 3 | **瞬时 I2C NACK（20 ms）** → `faults[0]` 永久锁存；通道通信会自恢复但故障位不清 | **+1 DUT-ms** | 中：随机、间歇；与“频繁但不定时”一致 |
| 4 | **IMU 健康检查错误（150 ms）** → `imu_bad` 永久锁存 | **+2 DUT-ms** | 中低：需 SPI/电源在转动时受扰 |
| 5 | **编码器持续计数** → `motion fault 5`（WAIT_STILL 停稳超时 1000 ms） | **+1000 DUT-ms** | 低：需 >1 s 持续“未停稳” |

**关键设计缺陷（4 条路径共同点）**：

1. 一旦 `fault` 成立，`system_dispatch` 立即 `auto_run<=0`；而 `obstacle_route` 的复位是
   `rst_n && auto_run`，所以 route 的 `route_fault` 与 `reason`（0x84/0x80…）**在同一时刻被清零**，
   板级 OLED/语音只能看到“FAULT RESET REQUIRED”，**丢失了具体故障原因**。
2. `fault_latched` 只有 `rst_n`（K0 退出/软复位）能清除；上述 4 类瞬时条件一旦发生就**必须 K0 才能继续**，
   不会自动恢复，与本轮“瞬时故障可恢复、真实故障才安全停车”的目标直接冲突。
3. 负向对照证明：同样的 20 mm 只持续 100 ms（< barrier 1000 ms）时**不产生任何故障**，路线在 barrier 通过后
   继续（state 8→23）。所以机制 2 是“等超时判故障”，不是 20 mm 本身触发传感器故障。
4. **修复版本对照（最终冻结源码，10/10 场景）**：在同样的注入下**没有出现任何 fault**
   （`records/fault_injection/fixed_final/` 中 0 条 `EVIDENCE FAULT_T0`）：20 mm 场景改为 barrier 重试/暂停
   （B 场景约 4 s 后进入 BLOCKED 暂停，`auto_run=0`、`fault_led=0`，属设计预期）；C 通道自恢复后路线继续；
   D/E 瞬态不再锁存；G_NEAR 不再产生近距离故障；H 只用“前向 + 相关侧向”lane 判 barrier，未被要求的对侧 lane
   的 20 mm 不再阻塞。**XSHUT glitch 自愈已修复并验证生效**：注入 20 mm 的通道
   `glitch_recover_req == driver_recover == tof_resets > 0`（A_FRONT 33/33/33、A_SIDE 33/33/33、I_HOLD 82/82/82、
   B 124/124/124、H_TURN2 左通道 33、CTRL 3/3/3），且注入结束（恢复 2000）后 **6–17 DUT-ms** 内 barrier 即通过、
   路线继续。详见 §7。

---

## 1. 验证对象与方法

### 1.1 基线版本（用户实际运行版本）

- 仓库基线：`git 0bd1a77 chore: record pre-change baseline for the obstacle-robustness round`
  （对应上一提交 `3556192`，`records/baseline_20261029/BASELINE.md` 记录的“改动前基线”）。
- 已核对：`git diff --name-only 3556192 0bd1a77 -- src` 输出为空，即两个提交的 `src/` 逐字节相同。
- **注意**：本轮修复开始后工作区 `src/` 被并发修改（11:53–11:56 修改了 `obstacle_sensors.v` /
  `obstacle_route.v` / `system_motion.v` / `robot_system_top.v`；期间一度无法编译）。
  为避免“半成品 RTL”污染证据，本报告的全部结果使用
  `git archive HEAD src` 导出的冻结副本编译：`records/fault_injection/baseline_src/`（只读复制，未改 `src/`）。

### 1.2 仿真环境

- ModelSim SE-64 10.7：`D:/app/modesilm/win64/vsim.exe`。
- DUT 参数与已通过的 `sim/tb_system_obstacle.sv` 完全一致（`CLK_HZ=1000000` 等加速参数）：
  **1 DUT ms = 1000 时钟周期 = 20 µs 仿真时间**，1 DUT s = 20 ms 仿真时间。
- 复用已有通过套件的：`sim/models/vl53l0x_model.sv`（协议级 VL53L0X 模型，`ranges` 可在任意时刻改）、
  电机/编码器物理 plant 模型、IMU660RA SPI 从机模型、ASR 反馈帧解析器。
- 新增：`sim/tb_fault_turns.sv`（单场景一次仿真，`+SCENARIO=<n>` 选择），
  `sim/run_fault_turns*.do`（编译 baseline 源码 + 跑指定场景）。

### 1.3 每个场景的公共驱动流程

1. 上电等待 READY、`tof_valid=7`；
2. 复用通过套件路径：SETTINGS 打开 VOICE ON → 返回主菜单 → 进入 `OBSTACLE AVOID` 面板 → **K2 启动**；
3. 路线正常进入 CRUISE(2)，前向测距切到 **130 mm** 触发 BRAKE_SELECT(3) → WAIT_OUT(5) → **TURN_OUT(4) 左转 90°**；
4. 转向完成瞬间路线进入 **WAIT_PARALLEL(8)**（或第二个转向的 WAIT_BACK(13)），在此处按场景注入；
5. 监视第一个 `dut.control.fault=1` 的时钟，打印全部锁存源（`EVIDENCE FAULT_T0` 行）。

### 1.4 运行命令（在仓库根目录执行）

```
& 'D:/app/modesilm/win64/vsim.exe' -c -l records/fault_injection/<log> -do sim/run_fault_turns<后缀>.do
```

例：`vsim.exe -c -l records/fault_injection/scenario_a_front.log -do sim/run_fault_turns.do`

---

## 2. baseline 结果总表（冻结的改动前 RTL，即用户实际运行的版本）

| 场景 | 注入条件 | 注入时刻<br>(DUT ms) | 首次故障<br>(DUT ms) | Δ | 首个故障源（锁存位） | route state / reason | 电机 | 能否自动恢复 | 日志（行号） |
|---|---|---|---|---|---|---|---|---|---|
| **A_FRONT** | 转向完成后 FRONT 通道 = 精确 20 mm，保持 1.2 s | 3710 | 4709 | +999 | `route_fault=1`，reason **0x84**；sensor_fault=0，fault_bits=0x0000 | 19(FAULT)→0 / 0x84 | 停 | 否，需 K0 | `scenario_a_front.log:153-154` |
| **A_SIDE** | 转向完成后 RIGHT 侧向通道 = 精确 20 mm，保持 1.2 s | 3710 | 4709 | +999 | `route_fault=1`，reason **0x84**；sensor_fault=0 | 19(FAULT)→0 / 0x84 | 停 | 否，需 K0 | `scenario_a_side.log:153-154` |
| **B** | 转向完成后 FRONT 通道 = 20 mm，**永久** | 3710 | 4709 | +999 | `route_fault=1`，reason **0x84**；sensor_fault=0 | 19(FAULT)→0 / 0x84 | 停 | 否；持续 20 mm 也不会再变化 | `scenario_b.log:153-154` |
| **C** | 转向完成后 RIGHT 通道 I2C **NACK 20 ms**（present=0），随后恢复 | 3710 | 3711 | **+1** | `sensor_fault=1`，fault_bits=**0x0004**，ch2 `faults=0x01`（faults[0] 坏样本）；`tof_error[2]=0x01` | 8 / 0x02（故障输入路径，route 随即被复位） | 停 | 通道通信自动恢复（+13 ms 恢复 fresh），但 `fault_latched` 保持 → 需 K0 | `scenario_c.log:152,158` |
| **D** | 转动过程中 IMU 健康检查错误 150 ms（`model_error=1`） | 326 | 328 | **+2** | `imu_bad=1`，`imu_error=0x07`，`sensor_error=0x01`；sensor_fault=0 | 4 / 0x02 | 停 | IMU 会重新初始化，但 `fault_latched` 保持 → 需 K0 | `scenario_d.log:150` |
| **E** | 转向完成后的 WAIT_STILL 期间持续注入编码器计数 1.2 s | 3661 | 4661 | +1000 | **`motion_fault=0x05`**（`motion_fault_code=05`，`stop_elapsed=1000`） | 4 / 0x02 | 停 | 否，需 K0 | `scenario_e.log:150` |
| **CTRL**（负向对照） | 转向完成后 FRONT = 20 mm，仅 100 ms | 3710 | **无故障** | — | 无；恢复后 barrier 立即通过 | 8→23（继续） | 不停 | — | `scenario_ctrl.log:161,167` |
| **G_NEAR** | 转向完成后 RIGHT 侧向通道 = **有效 60 mm（非 20，≤85 mm）** | 3710 | 3711 | **+1** | `sensor_fault=1`，fault_bits=**0x4000**（bit12+2 = ch2 **faults[4]** 近距保护），ch2 `faults=0x10` | 8 / 0x02 | 停 | 否，需 K0 | `scenario_g_near.log:152,160` |
| **I_HOLD** | 转向完成后 FRONT = 20 mm，通信正常，共保持 **2 s** | 3710 | 4709 | +999 | `route_fault=1`，reason **0x84**；`hold20=1`、`fresh=0`、`post_turn=0`（保持到 2010 ms age 仍相同） | 19(FAULT)→0 / 0x84 | 停 | 否，需 K0 | `scenario_i_hold.log:153,159` |
| **H_TURN2** | 左转+横移+**右转**完成后（WAIT_BACK=13）LEFT 通道 = 20 mm，1.2 s | 18630 | 19629 | +999 | `route_fault=1`，reason **0x84**；sensor_fault=0，fault_bits=0x0000 | 19(FAULT)→0 / 0x84 | 停 | 否，需 K0 | `scenario_h_turn2.log:164-165` |

> Δ 的测量基准是“路线进入转向后 barrier 状态的时刻”（TB 标记点）；实际 barrier 超时阈值为 **1000 DUT-ms**，
> 标记点比 route 进入 barrier 晚约 1 DUT-ms，故观测 Δ=999。E 场景的 motion 停稳超时阈值同为 1000 DUT-ms。
> 全部 10 个场景均 `Errors: 0, Warnings: 0`，PASS 行见各自日志 `RESULT scenario=n PASS`。

`fault_bits` 位映射（`obstacle_sensors.v` 第 43-44 行）：

| fault_bits 位 | 通道 | 内部位 | 含义 |
|---|---|---|---|
| `[0..2]` → `[g]` | ch0/ch1/ch2 | `faults[0]` | 坏样本（I2C/NACK/超时/状态异常） |
| `[3..5]` → `[3+g]` | ch0/ch1/ch2 | `faults[1]` | 通信过期（`comm_age >= 250 ms`） |
| `[6..8]` → `[6+g]` | ch0/ch1/ch2 | `faults[2]` | 有效数据过期（`age >= 250 ms`） |
| `[12..14]` → `[12+g]` | ch0/ch1/ch2 | `faults[4]` | 近距保护（有效距离 ≤ 85 mm） |

---

## 3. baseline 每个场景的证明日志（原文摘录）

### A_FRONT（精确 20 mm / 前向 / 1.2 s → barrier 0x84）

```
MAIN route=8 auto=1 waiting=0 reason=02 dut_ms=3710
EVENT INJECT_EXACT20 ch=0 mm=20 hold_ms=1200 expect_fault=1 route_state=8 dut_ms=3710
MAIN route=19 auto=1 waiting=0 reason=84 dut_ms=4709
EVIDENCE FAULT_T0 dt_from_mark_ms=999 ... route_state=19 route_fault=1 route_reason=84
  sensor_fault=0 fault_bits=0000 ch0_faults=00 ch1_faults=00 ch2_faults=00
  post_turn=110 fresh=011 ambiguous=100 reused=100 distances=07d0/07d0/0190 tof_raw_age=1007/9/9
EVIDENCE ROUTE_FAULT state=19 reason=84 route_fault=1 dut_ms=4709
```

解读：`post_turn=110` → 前向通道（ch0）`fresh=0`；`reused=100` → 该通道的 `hold20` 为 1（复用历史正常值
2000=0x07D0，`distances` 仍是 07d0，`tof_raw_age=1007` 说明原始样本早已过期）；另外两通道 fresh=1。
`&post_turn=0` → WAIT_PARALLEL 的 `elapsed` 到 1000 DUT-ms → `route_fault=1, reason=0x84`。
`sensor_fault=0 / fault_bits=0` 证明**不是传感器故障位触发**，而是路线 barrier 超时。

### A_SIDE / B / I_HOLD（同一机制的侧向/永久/2 s 变体）

```
scenario_a_side.log:153  post_turn=011 fresh=110 ambiguous=001 reused=001 ... route_reason=84
scenario_b.log:153       post_turn=110 fresh=011 ... route_reason=84（之后持续 20 mm，故障状态不再恢复）
scenario_i_hold.log:159  EVIDENCE HOLD20_TOTAL ch=0 total_hold_ms=3000 hold20=1 fresh=0 post_turn=0
                         ambiguous=1 accepted=2000 age=2010 comm_age=5 error=00 faults=00 fault=1 latch=1
```

解读：20 mm 期间通道通信完全正常（`comm_age=5`，`error=00`），`hold20` 持续复用旧值 2000，
但 `fresh` 永远为 0（20 mm 不能建立转向后新鲜样本）→ `post_turn` 永远为 0 → barrier 必然超时。
这直接把“20 mm 不可作为转向后新鲜样本”的策略与 barrier 0x84 关联起来（I_HOLD 保持 2 s 仍如此）。

### C（瞬时 I2C NACK → `faults[0]` 永久锁存，通道可自恢复）

```
EVENT INJECT_NACK ch=2 present=0 hold_ms=20 route_state=8 dut_ms=3710
EVIDENCE FAULT_T0 dt_from_mark_ms=1 ... sensor_fault=1 fault_bits=0004 ch2_faults=01
  route_state=8 route_fault=0 route_reason=02 tof_error=00/00/01
EVENT NACK_RESTORE present=111 dut_ms=3714
EVIDENCE CHANNEL_RECOVERY ch=2 recovered=1 error=00 comm_age=0 fresh=1 good=1 hold20=0 faults=01 dut_ms=3723
SNAPSHOT ... fault_latched=1 sensor_fault=1 fault_bits=0004 ... motion_fault=01 ... status=06 fault_events=1
```

解读：20 ms 的 NACK 在 +1 DUT-ms 内锁存 ch2 `faults[0]`（fault_bits bit2=0x0004）。13 DUT-ms 后该通道
已重新通信正常（`error=00, fresh=1`），但 `faults[0]` 是**只置位不清零**的寄存器，`fault_latched` 保持。
`motion_fault=01` 是外部故障输入引起的连锁值，不是首因。

### D（IMU 健康检查错误 → `imu_bad`）

```
EVENT INJECT_IMU model_error=1 hold_ms=150 motion_mode=6 motion_phase=1 route_state=4 dut_ms=326
EVIDENCE FAULT_T0 dt_from_mark_ms=2 ... imu_bad=1 imu_error=07 sensor_error=01 ... ctrl.fault=1
EVIDENCE IMU_STATE imu_ready=0 imu_error=07 calibrating=0 attitude_valid=0 dut_ms=431
```

解读：IMU 周期健康检查读到错误寄存器非 0 → `error_code=7`、`ready=0` → `imu_bad` → 锁存。
注入点在转动过程中（mode=6 ROTATE, phase=1 ACCEL），2 DUT-ms 内就锁存（健康检查周期 20 DUT-ms）。

### E（编码器持续计数 → motion fault 5）

```
EVENT INJECT_ENCODER phase=4 mode=8 stop_elapsed=0 route_state=4 dut_ms=3661
EVIDENCE FAULT_T0 dt_from_mark_ms=1000 ... motion_fault=05 motion_fault_code=05
  motion_phase=0 motion_stop_elapsed=1000 route_state=4
```

解读：在转向命令的 WAIT_STILL 阶段注入持续正交计数（左轮），运动 FSM 收到的速度始终非 0，
`still_l` 连续 3 窗口条件永不满足 → `stop_elapsed=1000` → `detected_fault=5` → `motion_fault=0x05`。

### CTRL（负向对照：100 ms 的 20 mm 不触发故障）

```
EVENT INJECT_EXACT20 ch=0 mm=20 hold_ms=100 ... route_state=8 dut_ms=3710
EVENT NO_FAULT_WHILE_HELD ch=0 held_ms=100 route_state=8
EVENT RESTORE ch=0 mm=2000 dut_ms=3813
MAIN route=23 auto=1 waiting=0 reason=02 dut_ms=3820
SNAPSHOT ctrl.fault=0 fault_latched=0 ... route_state=23 ... fresh=111 ... fault_events=0
PASS scenario CTRL: 100 ms exact-20 mm pulse after turn -> NO fault, route continued (state=23)
```

解读：恢复 2000 mm 后下一发正常样本即置 fresh，`&post_turn` 满足，barrier 放行并进入 SAMPLE_SHIFT(23)。
说明 A/B/I 的 FAULT 来自“barrier 被 20 mm 卡住 1000 ms”，不是 20 mm 本身。

### G_NEAR（有效近距 ≤85 mm → `faults[4]`）

```
EVENT INJECT_NEAR ch=2 mm=60 hold_ms=500 route_state=8 dut_ms=3710
EVIDENCE FAULT_T0 dt_from_mark_ms=1 ... sensor_fault=1 fault_bits=4000 ch2_faults=10
  route_state=8 route_fault=0 route_reason=02 distances=07d0/07d0/003c
```

解读：60 mm（0x003C）是**完全有效、非 20 mm** 的测距值（`tof_error=00`），但 `<=85 mm` 命中
“近距保护”分支 → ch2 `faults[4]=1`（fault_bits bit14=0x4000）→ `sensor_fault` → +1 DUT-ms 锁存。
**这一路径不需要任何器件异常或 20 mm 伪值，只需要转向后侧向传感器离障碍物 <85 mm。**

### H_TURN2（第二个 90° 转向完成后同样触发 0x84）

```
EVENT SECOND_TURN_DONE state=13 dut_ms=18630 actual_shift=652652
EVENT INJECT_EXACT20_TURN2 ch=1 mm=20 hold_ms=1200 route_state=13 dut_ms=18630
EVIDENCE FAULT_T0 dt_from_mark_ms=999 ... route_state=19 route_fault=1 route_reason=84
  post_turn=101 fresh=101 ambiguous=010 reused=010 ... tof_raw_age=2/1001/2
```

解读：完整走过“左转 → 横移(实际 652 mm) → 右转 90°”后，在 WAIT_BACK(13) barrier 注入 LEFT 通道
20 mm，同样 1000 DUT-ms 后 `route_fault=1 / reason=0x84`。**用户描述的第二种场景（左转+前进固定距离+
右转完成）与第一种是同一个 barrier 机制。**

---

## 4. 锁存路径信号链（baseline 源码行号）

### 4.1 机制 2：barrier 超时 → `route_fault=1, reason=0x84`（A/B/I/H）

```
obstacle_sensors.v:
  27  accept_now = done && complete_ok && mm!=GLITCH_MM;      // GLITCH_MM=20 被丢弃
  30  hold20 = have && good && ambiguous && error==0 && comm_age<STALE_MS;
  39  post_turn[g] = fresh;
  75-83  if(mm==GLITCH_MM) ambiguous<=1;                     // 20mm 只置 ambiguous
         else accepted<=mm; fresh<=1;                         //    永不置 fresh
obstacle_route.v:
  34  barrier = WAIT_OUT|WAIT_PARALLEL|WAIT_BACK|WAIT_HOME|20|SAMPLE_SHIFT
  102-103  if(... || (barrier && elapsed>=BARRIER_TIMEOUT_MS)) begin
             advance(FAULT); route_fault<=1; reason<=0x84; end   // BARRIER_TIMEOUT_MS=1000
  123/130/168/195/201  各 WAIT_* 状态：if(!refresh && &post_turn) ...
system_motion.v:
  100 fault_now = sensor_fault || route_fault || imu_bad || enc_fault_l || enc_fault_r
                 || tx_overflow || motion_fault!=0;
  101-103 fault_latched 置位；fault = fault_now || fault_latched;
system_dispatch.v:
  33  stop_event = exit_cmd || pause_cmd || fault;
  120-125 stop_event -> auto_run<=0（此时 avoid_mode 仍为 1）
tb6612/system_motion: fault -> control_enable=0 -> 电机停；status=8'h06 -> ASR 事件 type3 code6（FAULT 提示）
```

### 4.2 机制 3：I2C 坏样本 → `faults[0]`（C）

```
tof_i2c.v      : 地址 ACK 位读到 NACK -> error=1（bit_error）
tof_driver.v   : WAIT_IO 中 io_error!=0 -> error<=io_error, sample_valid=0, done=1
obstacle_sensors.v:
  26  complete_ok = ok && id==8'hEE && mask==5'h1F && err==0;   // 失败样本 -> 0
  72  if(done) if(!complete_ok) begin good<=0; if(monitor_enable) faults[0]<=1; end
  90  assign fault = |fault_bits;      // fault_bits[g]=faults[0]
```
通道自恢复：`tof_driver` 下一次 trigger 看到 `error!=0` → `pull_reset` → RESET/BOOT/ISSUE 重新读取，
通信可恢复；但 `faults[0]` 是 `reg`，源码中**没有任何清零路径**，只有 `rst_n` 复位。
`monitor_enable = auto_run && !route_cancel`（`system_motion.v:169`），所以该故障只在避障自动运行期间锁存。

### 4.3 机制 1：近距保护 → `faults[4]`（G_NEAR）

```
obstacle_sensors.v:
  67  if(have && good && accepted<=(g==0?front_near_mm:SIDE_NEAR_MM) && !accept_now) faults[4]<=1;
  82  if(monitor_enable && mm<=(g==0?front_near_mm:SIDE_NEAR_MM)) faults[4]<=1;
  // front_near_mm=85, SIDE_NEAR_MM=85
```
任何通道“有效（非20）距离 ≤85 mm”在自动模式下都会锁存近距故障。这与避障路线自己的几何逻辑冲突：
`obstacle_route.v` 的 `SAMPLE_SHIFT`/`WAIT_*` 明确接受 86–300 mm 的侧向箱体距离（`if(side_mm>AVOID_MM && stable_side)`），
但 ≤85 mm 会先一步把故障锁存，路线根本走不到自己的 `BLOCKED`/重试逻辑。

### 4.4 机制 4：IMU 健康失败 → `imu_bad`（D）

```
system_motion.v:64-66
  imu_bad = imu_error!=0 || sensor_error!=0 || (ready_seen && !imu_ready) ||
            (imu_ready && !sample_valid && sample_age>=IMU_TIMEOUT-1) ||
            (yaw_seen && !yaw_valid && yaw_age>=IMU_TIMEOUT-1);
imu660ra_driver.v: 健康检查 ST_HEALTH_ERROR 读 0x02 错误寄存器非 0 -> ready<=0, error_code<=7
```

### 4.5 机制 5：WAIT_STILL 停稳超时 → `motion fault 5`（E）

```
motion_control_and_mode_fsm.v:
  148  if(phase==WAIT_STILL && stop_elapsed>=STOP_TIMEOUT_MS) detected_fault=5;   // 1000 ms
  168-174 fault_code<=detected_fault -> mode=FAULT -> motion_fault=0x05
```

### 4.6 故障原因的“黑洞”

`fault` 成立后 `auto_run` 立即为 0，而 `obstacle_route` 的复位端是 `rst_n && auto_run`
（`system_motion.v:161`）。因此 route 的 `route_fault`/`reason` 会在下一个时钟被异步复位清零，
外部只能看到 `fault_latched=1`、`led_fault=1`、`status=0x06`，**看不到 0x84/0x80 等具体原因**。
本 TB 的 `EVIDENCE FAULT_T0` 之所以能捕获 0x84，是因为它在 route 复位前的同一个时钟沿采样。
上板时如果不在故障瞬间把 `route.reason`/`fault_bits` 快照到可读寄存器，就无法区分具体机制。

---

## 5. 真实板级原因排序（按可能性）

> 说明：本仿真只能证明“哪些条件会锁存 FAULT”，不能直接断言板上实际是哪一个。
> 但结合用户描述（**两个转向场景都高频出现、转向完成后立即/不久出现**）与设计代码，建议按以下顺序排查。
> 上板区分方法：在故障发生后（K0 前）读取面板三路距离快照或增加 fault 快照寄存器；
> 或者观察“转向结束到 FAULT 出现的延时”——立即出现≈#1/#3，约 1 秒≈#2。

### 第 1 位：转向后侧向 ToF 有效近距 ≤85 mm（faults[4]）— 场景 G_NEAR

- **为什么最可能**：避障时车在障碍物前 130 mm 停车后原地转向，侧向传感器正对箱体/墙面；
  只要几何间隙 <85 mm（箱体较宽、贴得较近、制动过冲、传感器安装偏前/偏侧）就会发生。
  不需要任何器件异常、不需要 20 mm 伪值、不依赖随机性，**每次贴得近都会发生**，与“经常出现”吻合。
  用户两个转向场景（左转；左转+横移+右转）都会在转向后让某个侧向传感器贴近障碍物。
- **证明日志**：`scenario_g_near.log:152`（`sensor_fault=1 fault_bits=4000 ch2_faults=10`，
  `distances=07d0/07d0/003c`，+1 DUT-ms 锁存）。
- **上板确认**：故障瞬间面板右/左侧距离是否 ≤0055（85 mm）。

### 第 2 位：转向后某通道持续精确 20 mm → barrier 1000 ms 超时（0x84）— 场景 A/A_SIDE/B/I/H

- **为什么很可能**：代码/文档本身已经记录了“20 mm 不能建立转向后新鲜样本”这一策略
  （`OBSTACLE_BOARD_TEST.md:58`，“正常通信中的 20 可复用历史正常距离，但不能建立首次正常值或转向后新鲜样本”），
  说明设计者**已经在板子上见过转向后的 20 mm 样本**。VL53L0X 在目标近于最小测量距离（约 20–30 mm）
  或强反射/贴面时可能返回固定的 20 mm 级读数；转向后侧向传感器贴面正好满足这一物理条件。
  一旦出现且持续 ≥1000 ms，就必然 0x84。延迟约 1 s，用户可能感知为“转向完成后不久”。
- **证明日志**：`scenario_a_front.log:153-154`、`scenario_h_turn2.log:164-165`
  （`post_turn=110 fresh=011 reused=100`，`sensor_fault=0 fault_bits=0000`，`route_reason=84`）。
- **上板确认**：故障前如果能看到某通道距离显示 0014（20 mm）且之后 ~1 s 才出现 FAULT。

### 第 3 位：瞬时 I2C NACK/坏样本 → faults[0] 永久锁存 — 场景 C

- **为什么可能**：转向是整车线束/连接器受力最大的动作（3 路 ToF 软排线、杜邦线、电源地弹跳），
  任何一次 NACK/超时都足以永久锁存（+1 ms 内），且**与几何位置无关**——这可以解释“有时能跑完、
  有时频繁出现”的无规律部分，也直接对应用户本轮要求“稳定并行采集传感器不报错”。
- **证明日志**：`scenario_c.log:152`（`sensor_fault=1 fault_bits=0004 ch2_faults=01`）、
  `:158`（通道 13 ms 后恢复通信，但 faults[0] 仍是 01）。
- **上板确认**：故障不随障碍物位置变化、K0 后同一位置重跑可能不复现 → 更像随机通信问题；
  可在故障后读 `tof_errors`（本 TB 中 `tof_error[2]=01`）。

### 第 4 位：IMU 健康检查错误 → imu_bad — 场景 D

- **可能性中低**：电机 PWM 大电流与 IMU SPI 共用板级电源/地时，转向（堵转电流大）可能干扰 SPI 读回，
  健康检查失败即锁存。但 IMU 是连续 5 ms 采样、20 ms 健康检查，若噪声只在转向时出现，
  故障应出现在**转动过程中/刚结束**；若板上能读到 imu_error 码（4/5/6/7）可直接确认。
- **证明日志**：`scenario_d.log:150`（`imu_bad=1 imu_error=07 sensor_error=01`，+2 DUT-ms）。

### 第 5 位：编码器噪声 → motion fault 5 — 场景 E

- **可能性低**：要求 WAIT_STILL 期间连续 ≥1 s 速度非 0（轮子打滑、被障碍物顶住来回抖动、编码器受扰）。
  如果用户观察到转向后车体被障碍物顶住/抖动，则可能；否则基本可排除。
- **证明日志**：`scenario_e.log:150`（`motion_fault=05 motion_fault_code=05 stop_elapsed=1000`）。

---

## 6. 修复建议（供 RTL 修复轮参考）

1. **barrier 不应以 `&post_turn`（三路同时 fresh）作为唯一放行条件**：
   改为“相关通道 fresh 或已知最近值 + 通道级超时重试/降级”，超时后进入暂停/重试（可 K2 继续），
   而不是直接 `FAULT` 锁存；至少 `WAIT_*` 的 0x84 不应锁存到 `fault_latched`。
2. **保留“20 mm 不算 fresh”的安全策略，但让它不再致死**：通道持续 20 mm 时，barrier 应把它视为
   “该方向已知极近”的几何信息，交给 `SAMPLE_SHIFT`/`BLOCKED` 逻辑判断，或允许 2-of-3 fresh 放行。
3. **瞬时故障 vs 持久故障分离**：`faults[0]`（以及 `faults[1]/[2]`、`imu_bad`）改为“连续 N 次/持续 X ms
   才锁存”，并在通信恢复后自动清除/重新武装；只有真实持久故障才保持安全停车。
4. **近距保护 `faults[4]` 在自动避障路线期间不应锁存**：≤85 mm 是绕障时的正常几何（路线自己有
   `BLOCKED`/`SAMPLE_SHIFT` 处理），应只对非避障运动或“同一距离持续恶化”这类条件生效。
5. **保留故障诊断信息**：fault 锁存时把 `route.reason`、`route_fault`、`fault_bits`、`imu_error`、
   `motion_fault` 快照到 K0 前可读的寄存器/调试帧（现在 `auto_run` 掉落后 reason 立即丢失，
   板上无法区分 0x84/0x80/…）。这样上板一次就能确认到底命中哪条路径。
6. **可恢复路径**：故障解除后允许自动恢复（或 K2 重试），满足“90° 转向后路线可重试继续”。
7. **XSHUT glitch 自愈的时序（最终冻结源码已修复并验证，见 §7.3）**：历史上 `obstacle_sensors` 的
   `recover_req` 只脉冲 1 拍，而 `tof_driver` 只在 `IDLE: if(trigger)` 里采样，而 `trigger` 下一拍才寄存，
   请求 100% 丢失（修复提交 `ef37e97` 快照证据：`glitch_recover_req=33` 但 `driver_recover=0`、`tof_resets=0`）。
   最终冻结源码已把 driver 改为 `IDLE: if(trigger || recover_req)`；本报告 §7.3 用
   `glitch_recover_req == driver_recover == tof_resets > 0` 验证其已生效。

---

## 7. fixed 版本对照（最终冻结源码）

### 7.1 方法

- 编译对象：**最终冻结的当前工作区 `src/`**（2026-10-29 12:25 最后一次修改后冻结；不是 git HEAD；
  baseline 的 `fault_lib` 与 `baseline_src/` 保持不动）。使用 `sim/run_fault_turns_fixed_compile.do`
  把最终 src 编到独立库 `fault_fixed_lib`（`sim/work_fault_fixed/`），不覆盖 baseline 库。
- TB 用 `+FIXED=1` 进入“观察模式”，最终判定行是 `FIXED_RUN_OBSERVED`；另用 `+NEWMENU=1` 适配最终菜单
  （DISPLAY 组删除：root item0=MANUAL(2)、item1=SETTINGS(4)、item2=OBSTACLE(5)）。
- 最终 fixed 日志在 **`records/fault_injection/fixed_final/`**（10 个场景，日志中 **0 条 `EVIDENCE FAULT_T0`**）；
  修复提交 `ef37e97` 的早期 fixed 日志保留在 `records/fault_injection/fixed/` 作历史对照。
- 场景注入与 baseline 完全相同（E 场景把编码器注入延长到 3.5 s，以覆盖新 `STOP_TIMEOUT_MS=3000`）。
- fixed 探针（编译宏 `FIXED_RTL`）：统计每通道 `glitch_recover_req`（obstacle_sensors 的恢复请求）、
  `driver_recover`（tof_driver 接受次数）、`tof_resets`（VL53L0X 模型观测到的 XSHUT 复位次数）。

### 7.2 baseline vs fixed_final 汇总表

| 场景 | baseline 首个故障源（Δ） | fixed_final 结果（`FIXED_RUN_OBSERVED`） | XSHUT 自愈<br>`req/driver_recover/tof_resets` | fixed 证明行 |
|---|---|---|---|---|
| **A_FRONT** | route 0x84（+999 ms） | `fault_seen=0`、`fault_latched=0`、`fault_bits=0000`；20 mm 期间 barrier 重试；恢复 2000 后 **+6 DUT-ms** barrier 通过（4913 restore→4919 到 23），路线继续到 **state=6(SHIFT)**，`auto_run=1` | ch0 **33/33/33** | `fixed_final_a_front.log:162`（:157,:160） |
| **A_SIDE** | route 0x84（+999 ms） | `fault_seen=0`；恢复 2000 后 **+6 DUT-ms** barrier 通过，路线继续到 **state=22(SHIFT_EXTRA)**，`auto_run=1` | ch2 **33/33/33** | `fixed_final_a_side.log:163` |
| **B**（永久 20 mm） | route 0x84（+999 ms） | `fault_seen=0`、`fault_latched=0`；4 次 barrier 重试后约 **+3999 ms** 进入 **BLOCKED(17) reason=0x85**、`auto_run=0`、route 复位（暂停，非故障），`status=20`、`fault_led=0`、`fault_events=0` | ch0 **124/124/124**（持续 20 mm 反复重初始化，仍按设计暂停） | `fixed_final_b.log:163`（:156 进入 BLOCKED） |
| **C**（NACK 20 ms） | sensor `faults[0]`（+1 ms） | `fault_seen=0`、`sensor_fault=0`；通道自恢复（`error=0`），路线继续到 **state=6(SHIFT)**，`auto_run=1` | 0/0/**64**（error 路径，非 glitch 路径） | `fixed_final_c.log:163` |
| **D**（IMU 150 ms） | `imu_bad`（+2 ms） | `fault_seen=0`；IMU 重初始化后恢复（观察点 `imu_error=00`），转动继续，`auto_run=1` | 0/0/0 | `fixed_final_d.log:159` |
| **E**（编码器抖动） | `motion_fault=5`（+1000 ms） | `fault_seen=0`；新 `STOP_TIMEOUT_MS=3000` 到点后 motion 以 **reason=7** 完成（WAIT_STILL 进入 3661→route=8 于 6661，恰好 +3000 ms），转向动作结束、路线继续到 23→6，`auto_run=1` | 0/0/0 | `fixed_final_e.log:161`（:153-156 为 3000 ms 证据） |
| **CTRL** | 无故障 | 无故障，路线继续到 state=6（回归对照保持） | ch0 **3/3/3** | `fixed_final_ctrl.log:164` |
| **G_NEAR**（侧向 60 mm） | sensor `faults[4]`（+1 ms） | `fault_seen=0`、`sensor_fault=0`、`fault_bits=0000`；60 mm 作为普通几何数据，barrier 正常放行（路线到 23/SAMPLE_SHIFT，`auto_run=1`） | 0/0/0（60 mm 不是 glitch） | `fixed_final_g_near.log:161` |
| **H_TURN2**（第二转向后左侧 20 mm） | route 0x84（+999 ms） | `fault_seen=0`；barrier 只要求 front+相关侧向 fresh，未用到的左侧 lane 20 mm 不再阻塞，注入后 **+17 DUT-ms** 即从 WAIT_BACK(13) 进入 **PASS(9)** | ch1 **33/33/33**（对侧 lane 自愈，不阻塞路线） | `fixed_final_h_turn2.log:170`（:167 进入 9） |
| **I_HOLD**（20 mm 2 s） | route 0x84（+999 ms） | `fault_seen=0`；3 s 内 `hold20=1 / fresh=0 / post_turn=0`（`accepted=2000, age=3011`）；恢复 2000 后 **+14 DUT-ms** barrier 通过（→23），路线继续到 **state=6(SHIFT)** | ch0 **82/82/82** | `fixed_final_i_hold.log:162`（:157,:160） |

结论：**最终冻结源码 10/10 场景均不再锁存 fault**；仅 B（持续 20 mm 超过 4 次 barrier 重试）按设计预期进入
BLOCKED 暂停（`auto_run=0`，需要 K2 重试继续），不是故障。

### 7.3 XSHUT glitch 自愈：从缺陷到修复验证

设计意图：`obstacle_sensors` 连续 4 个精确 20 mm 后请求 `tof_driver` 做一次 XSHUT 重新初始化，使持续 20 mm
的通道能自愈，避免它长期卡住转向后的 barrier。

1) **缺陷（修复提交 `ef37e97` 快照，12:12 编译）**：`recover_req` 只脉冲 1 拍，而 `tof_driver` 只在
   `IDLE: if(trigger)` 分支采样，`trigger` 下一拍才寄存 → 请求 100% 丢失。历史日志
   `records/fault_injection/fixed/fixed_a_front.log:163` 证据：
   `glitch_recover_req=33/0/0  driver_recover=0/0/0  tof_resets=0/0/0`（作为对照，C 场景 error 路径
   `tof_resets=0/0/64`，说明 resets 观测链正常）。
2) **修复（最终冻结源码）**：`tof_driver` 改为 `IDLE: if(trigger || recover_req) begin`（2026-10-29 12:21），
   `recover_req` 不再依赖 `trigger` 同拍出现。
3) **验证（`fixed_final/`，全部 20 mm 注入场景）**：
   - A_FRONT：ch0 `33/33/33`；A_SIDE：ch2 `33/33/33`；I_HOLD：ch0 `82/82/82`；
     B（4.5 s 持续 20 mm）：ch0 `124/124/124`；H_TURN2（对侧左 lane）：ch1 `33/33/33`；CTRL：ch0 `3/3/3`。
     即 **每一次 `recover_req` 都被 driver 接受并实际产生了 XSHUT 复位**（三个计数逐场景完全相等）。
   - 自愈后路线可继续：A_FRONT/A_SIDE 在注入结束（恢复 2000）后 **+6 DUT-ms** barrier 通过，
     I_HOLD **+14 DUT-ms** 通过，随后路线继续到 SAMPLE_SHIFT/SHIFT；H_TURN2 因 barrier 不需要对侧 lane，
     注入后 **+17 DUT-ms** 直接进入 PASS。
   - B（永久 20 mm）属设计预期：XSHUT 重初始化后传感器仍读到 20 mm，`fresh` 仍无法建立 → barrier 4 次重试后
     BLOCKED 暂停（`reason=0x85`、`auto_run=0`、`fault_led=0`），**不是 FAULT**，需要 K2 重试。
   - CTRL（100 ms 20 mm，≤4 个样本量级）也触发 3 次 XSHUT 复位，且未恢复 2000 时 barrier 仍通过；
     A/B/I 等在注入结束后立即恢复，证明“短 glitch 自愈 + barrier 重试”组合满足“90° 转向后路线可重试继续”。

**结论：XSHUT glitch 自愈机制在最终冻结源码中已生效**：`glitch_recover_req>0` 时 `driver_recover>0` 且
`tof_resets>0`；瞬时 20 mm 结束后 barrier 在 ≤1 s（实测 6–17 DUT-ms）内通过并继续，永久 20 mm 以 BLOCKED
暂停收场而非 FAULT。

---

## 8. 文件与复现

### 8.1 新增/修改文件（未改 `src/`、未改 `tb_system_obstacle.sv`）

| 文件 | 说明 |
|---|---|
| `sim/tb_fault_turns.sv` | 故障注入 TB（10 场景，`+SCENARIO=0..9`） |
| `sim/run_fault_turns.do` | 默认场景 A_FRONT |
| `sim/run_fault_turns_a_side.do` | 场景 A_SIDE |
| `sim/run_fault_turns_b.do` | 场景 B |
| `sim/run_fault_turns_c.do` | 场景 C |
| `sim/run_fault_turns_d.do` | 场景 D |
| `sim/run_fault_turns_e.do` | 场景 E |
| `sim/run_fault_turns_ctrl.do` | 场景 CTRL（负向对照） |
| `sim/run_fault_turns_g_near.do` | 场景 G_NEAR |
| `sim/run_fault_turns_h_turn2.do` | 场景 H_TURN2 |
| `sim/run_fault_turns_i_hold.do` | 场景 I_HOLD（Lead 追加要求） |
| `sim/run_fault_turns_baseline.do` | 编译冻结 baseline 源码到独立库 `fault_lib`（`sim/work_fault_baseline/`） |
| `sim/run_fault_turns_fixed_compile.do` | 编译当前工作区 `src/`（fixed）到独立库 `fault_fixed_lib`（`sim/work_fault_fixed/`），不覆盖 baseline 库 |
| `sim/run_fault_turns_fixed.do` | fixed 观察模式；`FAULT_SCENARIO` 环境变量选场景，TB 以 `+FIXED=1 +NEWMENU=1` 运行 |
| `records/fault_injection/baseline_src/` | `git archive HEAD src` 冻结副本（与 3556192 的 src 相同，只读） |
| `records/fault_injection/scenario_*.log` | baseline 每个场景的 ModelSim 日志（含 EVIDENCE/PASS 行） |
| `records/fault_injection/fixed_final/fixed_final_*.log` | **最终冻结源码** 10 个场景日志（含 `FIXED_RUN_OBSERVED` 行，§7 引用） |
| `records/fault_injection/fixed/fixed_*.log` | 修复提交 `ef37e97` 的早期 fixed 日志（历史对照，§7.3 缺陷证据） |
| `records/fault_injection/REPORT.md` | 本报告 |

### 8.2 复现全部场景（PowerShell 片段）

#### baseline（冻结的改动前源码）

```powershell
$runs = @(
 @{log='scenario_a_front.log'; do='sim/run_fault_turns.do'},
 @{log='scenario_a_side.log';  do='sim/run_fault_turns_a_side.do'},
 @{log='scenario_b.log';       do='sim/run_fault_turns_b.do'},
 @{log='scenario_c.log';       do='sim/run_fault_turns_c.do'},
 @{log='scenario_d.log';       do='sim/run_fault_turns_d.do'},
 @{log='scenario_e.log';       do='sim/run_fault_turns_e.do'},
 @{log='scenario_ctrl.log';    do='sim/run_fault_turns_ctrl.do'},
 @{log='scenario_g_near.log';  do='sim/run_fault_turns_g_near.do'},
 @{log='scenario_i_hold.log';  do='sim/run_fault_turns_i_hold.do'},
 @{log='scenario_h_turn2.log'; do='sim/run_fault_turns_h_turn2.do'}
)
foreach($j in $runs){
 & 'D:/app/modesilm/win64/vsim.exe' -c -l ('records/fault_injection/' + $j.log) -do $j.do
}
```

单场景耗时约 16–20 s，H_TURN2（完整绕障到第二个转向）约 80–95 s；baseline 全部 10 个场景合计约 4 min（wall clock），
fixed 全部 10 个场景合计约 4 min（实测 241 s）。所有场景结束为 `Errors: 0, Warnings: 0`；
baseline 日志以 `RESULT scenario=<n> PASS` 结束，fixed 日志以 `FIXED_RUN_OBSERVED ...` 结束。

#### fixed（最终冻结源码，观察模式）

```powershell
New-Item -ItemType Directory -Force -Path records/fault_injection/fixed_final | Out-Null
$runs = @(
 @{n=0; log='fixed_final_a_front.log'},
 @{n=1; log='fixed_final_a_side.log'},
 @{n=2; log='fixed_final_b.log'},
 @{n=3; log='fixed_final_c.log'},
 @{n=4; log='fixed_final_d.log'},
 @{n=5; log='fixed_final_e.log'},
 @{n=6; log='fixed_final_ctrl.log'},
 @{n=7; log='fixed_final_g_near.log'},
 @{n=8; log='fixed_final_h_turn2.log'},
 @{n=9; log='fixed_final_i_hold.log'}
)
foreach($r in $runs){
  $env:FAULT_SCENARIO = "$($r.n)"
  & 'D:/app/modesilm/win64/vsim.exe' -c -l ('records/fault_injection/fixed_final/' + $r.log) -do sim/run_fault_turns_fixed.do
}
```

单场景作用：`sim/run_fault_turns_fixed.do` 会先执行 `sim/run_fault_turns_fixed_compile.do`（编译当前 `src/`
到 `fault_fixed_lib`），再以 `+SCENARIO=<n> +FIXED=1 +NEWMENU=1` 运行 TB。fixed 日志中 `FAIL scenario ...`
行是“基线预期未再出现”的意思（观察模式有意保留基线判定文字），以 `FIXED_RUN_OBSERVED` 行为准。
`+NEWMENU=1` 用于最终菜单（DISPLAY 组已删除：root item0=MANUAL(2)、item1=SETTINGS(4)、item2=OBSTACLE(5)）；
baseline 运行不带该参数，保持旧菜单映射，因此 baseline 证据仍可原样复现。

### 8.3 结论边界

- 仿真中的三路距离是**注入的**（协议级 VL53L0X 模型直接改 `ranges`），未建模真实光学、反光、
  线束受力与电源噪声；因此本报告证明的是“哪些条件会锁存 FAULT、锁存到哪个信号、需要多久”，
  而不是“板上一定是哪个”。真实原因需要按 §5 的上板区分方法确认。
- 本 TB 的 20 mm 场景按设计中的 `GLITCH_MM=20` 精确注入；近距场景按路线代码中的
  `SIDE_NEAR_MM=85` 边界注入 60 mm。两者都来自设计代码本身的常量，不是任意假设。
- fixed 证据钉定到 2026-10-29 12:25 冻结的最终工作区 `src/`（`records/fault_injection/fixed_final/`）；
  若此后 `src/` 再改动，需按 §8.2 重新生成 fixed_final 日志并刷新 §7。baseline 证据钉定到
  `3556192`/`0bd1a77` 的冻结副本（`baseline_src/`），不受工作区改动影响。

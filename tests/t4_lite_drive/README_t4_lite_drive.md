# tests/t4_lite_drive —— 编码器里程 + ToF 停车边界（精简驱动工程）

从 `tests/eth_motion_test` 复制出来的独立测试工程：保留已经验证过的**千兆以太网 UDP
链路 v2**（SET_VEL / SET_LIMIT / STOP / CLEAR_FAULT / HEARTBEAT + 20 ms 遥测），
新增**双轮编码器里程**（遥测 `enc_l/enc_r` = 左右轮里程 mm）、**前向 VL53L0X 停车
边界**（标定后 <= 120 mm 切断电机）和 **KEY0 使能**。协议本身零改动。

不碰主工程，也不碰任何其它 test 目录。

## 1. 数据流 / 用法

```
上位机 (树莓派, 20 Hz) --UDP SET_VEL--> FPGA --TB6612--> 左右轮
       ^                                   |
       |--UDP 遥测 50 Hz (enc_l/enc_r=mm) <-+
                     前向 VL53L0X --(<=120 mm)--> tof_stop 锁存 -> 切电机
```

1. 上电，等待 `led_link` 亮（链路 OK）、`led_ready` 亮（无故障、按键使能、ToF 未停车）。
2. 上位机按 20 Hz 发 `SET_VEL`，按 50 Hz 收遥测；用 `enc_l/enc_r`（单位 mm）判断
   已走距离，到位后发 `STOP`。
3. 心跳 300 ms 断流 / `CMD_TIMEOUT` 500 ms / STOP 锁存 / 任一故障位都会自动切 STBY。
4. 前向 ToF 标定后距离 <= 120 mm 时锁存 `tof_stop`，电机立即切断；**只有复位 FPGA
   才会解除**（安全边界，不做自动恢复）。

## 2. 新增/改动文件（相对 eth_motion_test）

| 文件 | 说明 |
| --- | --- |
| `src/t4_lite_top.v` | 新顶层（由 `eth_motion_top.v` 扩展；文件内另含 `t4_div1000` 逐位除法） |
| `src/t4_lite_top.cst` | 新引脚约束（eth_motion_top.cst + KEY0/编码器/ToF/灯） |
| `build.tcl` | 顶层改为 `t4_lite_top`，加入编码器/ToF 源文件 |
| `src/eth_link_bridge.v` | eth_link_protocol 版本的本地副本，只加了 `enc_l/enc_r`（里程 mm）与 `speed_l_mm_s/speed_r_mm_s`（实测轮速）四个输入并接入 `u_core`（原版固定接 0）；协议/行为不变 |
| `src/dual_ab_quadrature_encoder.v` | 从 task4_navigation 复制 |
| `src/asr_feedback.v` | 从 task4_navigation 复制（`asr_distance`） |
| `src/task4_tof_driver.v` | 从 task4_navigation 复制（模块名 `tof_driver`） |
| `src/tof_i2c.v` | 从 task4_navigation 复制 |
| `src/tof_calibrate.v` | 从 task4_navigation 复制 |
| `src/dual_speed_pi_pid_controller.v` | 主工程 `src/motor/` 副本（速度环 PI/PID）；**本地为 125 MHz 做了 3 级流水 + 位宽裁剪，方程/端口/参数名不变** |
| `src/speed_measurement_and_velocity_filter.v` | 主工程 `src/motor/` 副本（窗口测速 + 4 点滑动平均）；本地把滑动平均拆成 2 级流水，数值语义不变 |
| `src/pwm_and_startup_ramp.v` | 主工程 `src/motor/` 副本（duty 斜坡）；本地把命令预处理单独打一拍，数值语义不变 |
| `src/t4_wheel_speed_loop.v` | **新增**：mm/s 目标 → counts/窗口 → PI → duty 的互联/换算（取代开环 `to_duty`） |
| `sim/tb_wheel_speed_loop.sv` | **新增**：速度闭环物理仿真（真实 125 MHz/20 kHz 时间基准 + 一阶电机/编码器对象模型） |
| `tools/loop_model.py` | **新增**：整定用 Python 整数模型（RTL 控制律逐窗口复刻，扫 KP/KI/窗口用） |

其余（`src/eth_motion_bridge.v`、`sim/*`、`tools/*`、`modelsim.ini`、README/TEST_PLAN）
为 eth_motion_test 原样拷贝；旧的上位机工具保持不动，由 Lead 另做替换。

## 3. 顶层参数

| 参数 | 值 | 说明 |
| --- | --- | --- |
| `UM_PER_REV` | 204203 | 65 mm 轮径周长（µm） |
| `COUNTS_PER_REV` | 1456 | 编码器线数 |
| `ENC_FILTER_CYCLES` | 125 | 1 µs 去抖 @125 MHz；`INVERT_L=1, INVERT_R=0`（与主工程实车一致） |
| `TOF_I2C_HZ` | 100 kHz | 软 I2C |
| `TOF_OFFSET_MM` | 60 | 前向实测偏置（读数 = 真实 + 60） |
| `TOF_STOP_MM` | 120 | 标定后停车边界 |
| ToF 驱动 | `RESET=1 ms`, `BOOT=20 ms`, `POLL=10 ms`, `MAX_POLLS=100`, `TIMEOUT=10 ms` | CLK_HZ=125 MHz 按比例 |
| ToF trigger | 60 ms 周期脉冲，busy 期间自动跳过 | |
| 轮速遥测 | 100 ms 窗口 `Δmm × 10` → `speed_l/right`（mm/s，10 mm/s 量化） | 供上位机判断电机是否被安全门切断 |
| 遥测周期 | 20 ms（50 Hz） | 与 eth_motion_test 相同 |
| PWM | 20 kHz，`MAX_V_MM_S=300` | |
| `SPEED_WINDOW_MS` | 20 | 速度环窗口（counts/窗口 采样周期）。50 mm/s = 7 counts = 49.1 mm/s |
| `SPEED_KP` / `SPEED_KI` / `SPEED_KD` | 16 / 4 / 0 | 增量式 PI；与主工程连续域等效（见 §9） |
| `SPEED_GAIN_SHIFT` | 0 | PI 增量右移位数 |
| `SPEED_DUTY_DIV` | 2 | PI/ramp duty 上限 = PWM 周期/2 = 3125 拍（50%） |
| `SPEED_RAMP_STEP` | 1 | duty 每 PWM 周期最多变化 1 拍（主工程同值） |

## 4. 引脚（新增部分，完整见 `src/t4_lite_top.cst`）

| 信号 | 引脚 | 说明 |
| --- | --- | --- |
| `key_enable_n` | A15 | KEY0，低有效，PULL_UP |
| `key_estop_n` | C15 | KEY2，低有效，PULL_UP |
| `encoder_a_l/b_l` | C17 / C18 | 输入下拉 |
| `encoder_a_r/b_r` | F14 / G14 | 输入下拉 |
| `tof_front_scl/sda` | G16 / G18 | LVCMOS33 DRIVE=8（板上外部上拉，不用内部 OPEN_DRAIN） |
| `tof_front_xshut_n` | F18 | 同上 |
| `led_link` | A9 | 链路存活（高亮） |
| `led_fault` | B9 | 故障 / 停车 / 按键按下（高亮） |
| `led_run` | C14 | 电机已使能（高亮） |
| `led_ready` | D14 | 可以接收运动命令（高亮） |

其余以太网 RGMII、TB6612、`clk_50m`、`reset_n` 引脚与 `eth_motion_test` 完全相同。

## 5. 安全（全部硬连线）

* 心跳 300 ms 超时 / 任一链路故障位 / STOP 锁存 / CMD 超时（500 ms）
  → `motors_enabled=0` → TB6612 `STBY=0`；
* `key_estop_n && key_enable_n && !tof_stop` 同时门控 `eth_motion_bridge` 的使能
  和 `tb6612_driver_interface` 的 `enable`；`emergency_stop` 还额外包含
  `!key_estop_n || !key_enable_n || tof_stop`；
* 复位未释放 → inhibit。

## 6. 构建 / 仿真

```powershell
cd tests/t4_lite_drive
& 'D:\Gowin\Gowin_V1.9.12_x64\IDE\bin\gw_sh.exe' build.tcl      # -> impl/pnr/t4_lite_top.fs
& 'D:\app\modesilm\win64\vsim.exe' -c -do sim/run_motion.do
& 'D:\app\modesilm\win64\vsim.exe' -c -do sim/run_motion_unit.do
& 'D:\app\modesilm\win64\vsim.exe' -c -do sim/run_motion_system.do
& 'D:\app\modesilm\win64\vsim.exe' -c -do sim/run_wheel_speed_loop.do   # 新增：速度闭环
# 或一次跑完全部四套：.\run_motion_sim.ps1
```

旧仿真 TB 覆盖 `eth_motion_bridge + tb6612`（15/24/27 项），保持不变（它们验证的是
安全门控/开环映射的旧行为）；新增 `tb_wheel_speed_loop` 覆盖闭环速度环（24 项）。

## 7. 已知限制 / 实现说明

* `tof_stop` 为**锁存**信号（按任务书要求），只有复位释放；若需要"离墙后自动恢复"，
  把 `t4_lite_top.v` 里的 `tof_stop` 改成电平寄存器即可。
* ToF 只做停车边界：没有超时/失效判定，读数保持最后一次有效值；传感器在线时每
  60 ms 更新一次。
* `enc_l/enc_r` 为 int32 毫米（`position_um / 1000`，向 0 截断），分辨率约 0.14 mm；
  用文件内 `t4_div1000`（32 拍逐位除法）实现，避免 125 MHz 组合除法关键路径。
* 遥测 `speed_l/right` 为 100 ms 窗口的轮速（mm/s，10 mm/s 量化），只用于上位机
  判断"电机是否被切断"，**不参与控制**（控制用独立的 20 ms 窗口 `speed_measurement_
  and_velocity_filter`）。
* 遥测的 pitch/roll 字段仍借用为 `dbg_a/dbg_b`；yaw 由 clk_50m 域 IMU 链给出（CDC
  toggle 握手）。
* 速度环窗口 20 ms 的反馈量化是 1 count = 7.0 mm/s（20 ms 内 1 count 对应的轮速），
  因此稳态实际轮速可能在目标 ±1 count 内（50 mm/s 目标实测 49.9；从高速阶跃降到
  低速时最多可停在 +1 count ≈ 54.4 mm/s 附近）。这是“counts/窗口”方案的量化极限，
  若要更准可换 40 ms 窗口或改用 asr_distance 的 µm 差分测速（见 §9 风险）。
* 编码器失效（无增量）时 PI 会积分到 50% duty 上限（约 1 s）；本工程未把
  `encoder_fault` 接入安全门（保持原安全逻辑不变），上板前应确认编码器线束。

## 8. 本次构建 / 仿真结果（2026-10-30 速度闭环版）

| 项 | 结果 |
| --- | --- |
| 构建 | `gw_sh build.tcl` → **Bitstream generation completed**（exit 0） |
| 位流 | `impl/pnr/t4_lite_top.fs`，SHA-256 `57BE714A7616FECC95164303C3E1CA8C5E3850AA1FC0E8C8C7C385779D0865B7`（`.bin` `489F1E06AC1B3DBBE9C1DC4487CBBAC082A250A35AE86F22AD99578F299EEB92`；`.fs` 每次构建含元数据会变，`.bin` 内容相同） |
| 构建日志 | `records/build_t4_lite_final.log` |
| 资源 | 7470 LUT(33%) / 4266 REG(19%) / 8 DSP / 6 BSRAM（GW5A-25A） |
| 时序（PnR 默认 100 MHz 口径，见 `impl/pnr/t4_lite_top.timing_paths`） | 最差 SETUP **+0.691 ns**（改动前 +1.147 ns），HOLD 最小 **+0.138 ns**；**无负 slack** |
| 仿真 | `run_motion_sim.ps1`：bridge PASS 15 项、unit PASS 24 项、system PASS 27 项、**speedloop PASS 24 项**（`records/tb_*.log`） |
| 速度闭环实测（仿真，真实 125 MHz/20 kHz 时间基准） | 见 §9.3 |

## 9. 速度闭环（2026-10-30：移植主工程 PI）

### 9.1 数据流

```
SET_VEL(mm/s)  --eth_link_bridge（协议/心跳/STOP，未改）--> eth_motion_bridge
               （限幅 ±MAX_V_MM_S=300 + motors_enabled 安全门，未改）
   --> t4_wheel_speed_loop：编码器增量 -> speed_measurement_and_velocity_filter
        （20 ms 窗口 counts/窗口）-> mm/s 目标换算 -> dual_speed_pi_pid_controller
        -> pwm_and_startup_ramp -> 有符号 duty
   --> tb6612_driver_interface（未改）--> TB6612
```

遥测字段、IMU 50 MHz 域 + yaw CDC、ToF/STOP/KEY2/链路故障逻辑全部未改动。
旧的 `eth_motion_bridge::to_duty`（duty = v*20）仍保留在文件内供旧仿真套件使用，
但顶层不再连接它的 `pwm_l/pwm_r`，综合时被裁掉。

### 9.2 控制律与参数

```
control[n+1] = clamp( control[n] + (KP*de + KI*e + KD*dd) >> GAIN_SHIFT, ±DUTY_LIMIT )
e = target_counts - filtered_counts；de = e[n]-e[n-1]；dd = e[n]-2e[n-1]+e[n-2]
（饱和且误差继续向外时去掉 I 项，与主工程一致）
```

| 参数 | 值 | 说明 |
| --- | --- | --- |
| 速度窗口 | **20 ms**（主工程 10 ms） | 50 mm/s = 7.13 counts，取整 7 = 49.1 mm/s（-1.8%）；10 ms 窗口会取整成 4 counts = 56.1（+12%） |
| KP / KI / KD | **16 / 4 / 0** | KP=16@20 ms 与主工程 KP=32@10 ms 连续域等效；KI 取 4 使 SET_VEL 阶跃 ~0.6 s 到 90%（主工程同值约 2 s），Python 模型扫描无振荡 |
| GAIN_SHIFT | 0 | 与主工程一致 |
| duty 上限 | PWM 周期/2 = **3125 拍（50%）** | 主工程 25%；50% 覆盖 MAX_V=300 mm/s（实测 ~2570 拍），同时给积分灌满留余量 |
| duty 斜坡 | 1 拍/PWM 周期 | 与主工程一致（0→455 拍 ≈ 22.8 ms） |
| 目标换算 | counts = round(v·146/1024) | 精确比例 0.14260 counts/(mm/s)/窗口，误差 -0.02% |
| 编码器/轮径 | 1456 counts/rev，204203 µm/rev | 与主工程/task4 一致 |

> 125 MHz 适配：主工程三个电机模块原版为 50 MHz 单拍实现，直接搬到 125 MHz 会产生
> SETUP 违规（实测最差 -3.42 ns）。本地副本保持模块名/端口/参数名/数值语义不变，
> 仅做流水拆分（PID 3 级、测速滑动平均 2 级、目标换算 3 级、ramp 命令预处理 1 拍）。

### 9.3 闭环仿真实测（`sim/tb_wheel_speed_loop.sv`，真实时间基准）

被控对象：一阶电机 + 编码器模型，按旧开环实测标定（duty≈1000 tick → 110 mm/s，
即 0.1222 mm/s/tick，含 100 tick 静摩擦死区，τ=50 ms）。

| 场景（`records/tb_wheel_speed_loop.log`） | 结果 |
| --- | --- |
| V2 SET_VEL(50,50) 前进 | **49.91 / 49.91 mm/s**（滤波 7 counts/窗口，duty≈508 拍；误差 0.2%） |
| V3 SET_VEL(100,100) 前进 | 97.06 / 97.06 mm/s（目标取整 14 counts = 98.2，-1.1%） |
| V4 原地转 (+50,-50) | **+54.3 / -48.9 mm/s，差速 103.1 mm/s**；@240 mm 轮距 = yaw **-24.6 °/s** |
| V5 SET_VEL(-50,-50) 后退 | -49.86 / -50.35 mm/s（方向翻转正确） |
| V6 SET_VEL(0,0) | duty 归零，轮速衰减到 ~-2 mm/s |
| V7 enable=0（安全门关断） | duty 立即 0、STBY=0 |
| V8 SET_VEL(300,300) 满量程 | 296.2 / 296.0 mm/s（duty 2544 拍，未顶到 3125 限幅） |
| 合计 | **24 项检查全 PASS** |

`run_motion_sim.ps1` 四套结果：bridge PASS（15 项）/ unit PASS（24 项）/ system
PASS（27 项）/ speedloop PASS（24 项）→ “All motion simulation suites passed.”

### 9.4 复现

```powershell
cd tests/t4_lite_drive
& 'D:\Gowin\Gowin_V1.9.12_x64\IDE\bin\gw_sh.exe' build.tcl
& 'D:\app\modesilm\win64\vsim.exe' -c -do sim/run_wheel_speed_loop.do
.\run_motion_sim.ps1       # 四套仿真
```



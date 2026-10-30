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
```

旧仿真 TB 只覆盖 `eth_motion_bridge + tb6612`（15/24/27 项），与改动无关，原样保留。

## 7. 已知限制 / 实现说明

* `tof_stop` 为**锁存**信号（按任务书要求），只有复位释放；若需要"离墙后自动恢复"，
  把 `t4_lite_top.v` 里的 `tof_stop` 改成电平寄存器即可。
* ToF 只做停车边界：没有超时/失效判定，读数保持最后一次有效值；传感器在线时每
  60 ms 更新一次。
* `enc_l/enc_r` 为 int32 毫米（`position_um / 1000`，向 0 截断），分辨率约 0.14 mm；
  用文件内 `t4_div1000`（32 拍逐位除法）实现，避免 125 MHz 组合除法关键路径。
* 遥测 `speed_l/right` 为 100 ms 窗口的轮速（mm/s，10 mm/s 量化），只用于上位机
  判断"电机是否被切断"，不参与控制。
* 遥测的 IMU 字段（yaw/pitch/roll）仍为 0（本工程不接 IMU、不做速度环）。

## 8. 本次构建 / 仿真结果（2026-10-30）

| 项 | 结果 |
| --- | --- |
| 构建 | `gw_sh build.tcl` → **Bitstream generation completed**（exit 0） |
| 位流 | `impl/pnr/t4_lite_top.fs`，SHA-256 `DA2C37A1AC7E27338A439E4BBD2FCF0DF69C6D283AACAFFA880B285E8A74E657` |
| 构建日志 | `records/build_t4_lite.log` |
| 资源 | 3461 LUT(16%) / 2057 REG(9%) / 6 DSP / 2 BSRAM（GW5A-25A） |
| 仿真 | `sim/run_motion.do` PASS 15 项；`sim/run_motion_unit.do` PASS 24 项；`sim/run_motion_system.do` PASS 27 项（`records/tb_bridge.log`、`tb_unit.log`、`tb_system.log`） |


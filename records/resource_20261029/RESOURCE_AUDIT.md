# Embodied_RPACS 资源审计与安全优化方案（RESOURCE_AUDIT）

- **记录编号**：resource_20261029（task-2）
- **审计对象**：`D:\Gowin\Project\Contest\Embodied_RPACS\Embodied_RPACS\Embodied_RPACS` 主工程 `robot_system_top`
- **器件 / 工具 / 约束**：Gowin GW5A-25A（GW5A-LV25UG324C2/I1），Gowin EDA V1.9.12 64-bit，`build.tcl` `-global_freq 50`
- **审计性质**：**只读**。未修改 `src/`、`build.tcl`、`README.md`、`impl/`，未运行综合/PnR/`gw_sh`，未改动 git 状态。本文所有文件写入仅限 `records/resource_20261029/`。
- **数据时间点**：2026-10-29 11:37–11:49（Asia/Shanghai）。审计期间 Lead 的基线重建于 11:40:35 完成，前后两次 PnR 报告**逐项一致**（见 §2.2）。
- **图例**：**【实测】**＝直接读自 `impl/gwsynthesis/Embodied_RPACS_syn_rsc.xml` 或 `impl/pnr/Embodied_RPACS.rpt.txt` 的数字；**【估算】**＝由源码结构/模块实测值推导、需重综合后才能确认的收益。

---

## 0. 结论速览（TL;DR）

| 结论 | 内容 |
| --- | --- |
| 真实基线 | **Logic 17661/23040（77%）＝14866 LUT + 2795 ALU；Register 6859/23685（29%）；CLS 9858/11520（86%）；BSRAM 9/56；DSP 8.5/28；I/O 44/222** |
| 真正瓶颈 | **CLS 86%**，不是 Logic 77%。CLS 只剩 **1662 个**（≈3.0k logic 当量），工程上建议只动用 700–1100 个以保证布线质量。 |
| 53% 的来源 | `12036/53%` 是 02:06 一次**中间构建**的数字：该次构建 OLED 文本通路是死的（`font_byte` 无驱动），综合把 `system_pages`、`tof_page`、`attitude_angle_to_digits_pipeline` 整块 **swept**（有 PnR 警告为证）。**不能作为余量/合入依据**。 |
| 无需功能裁剪的收益（Tier A） | 约 **1.2–1.4k LUT / 0.65–0.70k FF**：ToF 三通道 FSM 共享、左右测速共享、PWM 斜坡共享、PI 共享、ASR FIFO 深度减半、按键计数器位宽。 |
| 需用户确认的功能裁剪（Tier B） | 约 **3.0–4.0k LUT / 0.8–1.1k FF**（B1–B3 主体；另含可选 B4/B5）：DISPLAY 组 5 个诊断页（~1.1–1.6k）、IMU 只保留 yaw（~1.5–1.9k）、USB 事件日志（~0.42k）、菜单下一项预览（~0.2–0.28k）。 |
| 结构性重构（Tier C，中高风险） | 约 **1.2–1.7k LUT**：pages 字面量文本改 pROM/字符流、`display_text` 改 SDPB、ToF I2C 位引擎共享。 |
| Tier A+B 全部落地后（估算） | Logic ≈ **12.3–13.5k（53–59%）**，CLS ≈ **6.9–7.5k（60–65%）**，可腾出约 2.4–3.0k CLS —— 后续合入 uart_bridge 才有现实空间（现状仅余 1662 CLS）。 |
| 时序 | **Fmax 53.020 MHz**（目标 50 MHz，余量 +6.0%）；Setup/Hold 违例 0，TNS 0/0；最关键路径在 **IMU 姿态陀螺积分 wrap 链**（slack 1.140 ns），第二在 **速度换算乘法链**（slack 1.995 ns）。 |

---

## 1. 证据与数据来源（全部为只读读取）

| 用途 | 文件 | 时间戳 / 说明 |
| --- | --- | --- |
| 当前 PnR 资源+引脚 | `impl/pnr/Embodied_RPACS.rpt.txt` | `Created Time: Thu Oct 29 11:40:35 2026`；审计开始时读到的是 04:16:34 版本，数值相同 |
| 当前时序 | `impl/pnr/Embodied_RPACS_tr_content.html`、`impl/pnr/Embodied_RPACS.timing_paths` | 11:40:35/36；`.rpt.txt` **不含**时序，Fmax/TNS 只能从这两个文件读 |
| 逐模块资源 | `impl/gwsynthesis/Embodied_RPACS_syn_rsc.xml` | 11:39:34，与 04:15 版本数值一致 |
| 图/综合日志 | `impl/gwsynthesis/Embodied_RPACS.log`、`impl/pnr/Embodied_RPACS.log` | 11:39:34 / 11:40:36，本次构建无 swept/black-box 警告 |
| 综合网表（交叉核对用） | `impl/gwsynthesis/Embodied_RPACS.vg` | 11:39:33 |
| 构建入口 | `build.tcl`、`Embodied_RPACS.gprj`、`impl/Embodied_RPACS_process_config.json` | 50 MHz、top=robot_system_top、GowinSyn、timing-driven |
| 行为/接口 | `src/robot_system_top.v`、`src/**/*.v`、`README.md`、`PIN_ASSIGNMENT.md` | 本轮逐模块通读 |
| 历史构建证据 | `records/obstacle_20261025/{baseline_rebuild,build_menu_trim,build_opt2,build_opt3,build_final}.log`、`records/main_20261023/*` | 用于 §9 差异溯源 |
| 仿真/验收入口 | `tools/run_system_sim.ps1`（7 套）、`tools/run_legacy_regression.ps1`、`tools/check_system_reports.ps1`、`verify_project.tcl` | 见 §7.2 |
| uart_bridge 对照 | `tests/uart_bridge/impl/pnr/uart_bridge*.rpt.txt`、`tests/uart_bridge/README.md`、`docs/INTEGRATION_GUIDE.md` | 用于 §9.3 合入余量修正 |

> 说明：`impl/pnr/Embodied_RPACS.rpt.txt` 只有 PnR 消息、资源汇总、I/O bank/时钟/引脚表，**没有 timing 章节**；任务书里“从 rpt.txt 查 Fmax/TNS”这一点以 §10 的 `tr_content.html` 为准。

---

## 2. 当前总体资源基线

### 2.1 官方 PnR 汇总（`impl/pnr/Embodied_RPACS.rpt.txt`，11:40:35）【实测】

| 资源 | 占用 | 利用率 | 剩余 | 备注 |
| --- | --- | --- | --- | --- |
| **Logic**（LUT+ALU+ROM16） | **17661/23040** | **77%** | 5379 | 14866 LUT + 2795 ALU，0 ROM16 |
| —— SSRAM(RAM16) | 0 | — | — | 未用分布式 RAM |
| **Register** | **6859/23685** | **29%** | 16826 | 6843 logic FF + 16 I/O FF；无 latch |
| **CLS** | **9858/11520** | **86%** | **1662** | **真正的紧张资源** |
| I/O Port | 44/222 | 20% | 178 | 输入 17 / 输出 21 / 双向 6 |
| I/O Buf | 37 | — | — | 10 in + 21 out + 6 inout |
| **BSRAM** | **9/56** | 17% | 47 | 4 SDPB + 5 pROM（综合 XML 记 7 个逻辑块，见 §4 注） |
| **DSP** | **8.5/28** | 31% | 19.5 | MULTALU27X18×4 + MULT27X36×2 + MULT12X12×1 |
| 时钟 | PRIMARY 3/8，LW 8/8 | — | — | LW 100%，新增全局时钟会受限 |
| 全局频率 | 50 MHz 约束 | — | — | 见 §10 |

### 2.2 与审计开始时（04:16 基线）的一致性【实测】

| 构建 | Created Time | Logic | LUT | ALU | Register | CLS | BSRAM | DSP | 来源 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 优化前（历史） | 2026-10-28 21:56 前后 | 20334（89%） | — | — | — | 10728（估算） | — | — | README/PLAN 引用；未见 rpt 原文 |
| 中间坏构建（**无效**） | 2026-10-29 02:06:09 | **12036（53%）** | — | — | 5138 | 7525 | — | — | 仅见 `tests/uart_bridge` 文档转述（§9） |
| 04:16 基线 | 2026-10-29 04:16:34 | 17661（77%） | 14866 | 2795 | 6859 | 9858 | 9 | 8.5 | 审计开始时读取的 `Embodied_RPACS.rpt.txt` |
| **11:40 重建（当前）** | 2026-10-29 11:40:35 | **17661（77%）** | **14866** | **2795** | **6859** | **9858** | **9** | **8.5** | 当前 `Embodied_RPACS.rpt.txt` |

- 两次 rpt 文件大小仅差 1 字节（71284 → 71283，即时间戳差异），**资源数字完全一致**；11:39 的 `Embodied_RPACS_syn_rsc.xml` 与 04:15 版本的前 73 行内容相同。
- 结论：**当前报告数字稳定、可信**；后续任何优化都应以 `17661 / 6859 / 9858 CLS` 为基线做 A/B 对比。

### 2.3 资源余量的正确解读（重要）

- Logic 余量 5379 是“器件 LUT 位”口径，但**版图资源是 CLS**。当前密度 17661 logic / 9858 CLS ≈ **1.79 logic/CLS**。
- 若按理论余量：1662 CLS ≈ **2.9–3.0k logic 当量**。
- 工程建议：PnR 在 >95% CLS 时布通率和时序风险明显上升。按 ≤93%/95% CLS 设限：
  - ≤93% → 可新增约 **740 CLS（≈1.3k logic）**；
  - ≤95% → 可新增约 **1086 CLS（≈1.9k logic）**。
- 即：**新增功能前必须先做优化**，不能简单按“Logic 还有 23%”规划。README 中“预留约 2700 LUT”只是 LUT 口径；按 CLS 口径实际可用更少。

---

## 3. 子系统资源分桶（按 Logic=LUT+ALU 计）

以综合 XML 的 `T_Lut / T_Alu / T_Register`（含子模块）汇总，全部为【实测】：

| 子系统 | Logic (LUT+ALU) | 占比 | Register | 其中主要模块 |
| --- | --- | --- | --- | --- |
| **IMU（驱动 + 姿态 + unwrap）** | **4356** | **24.7%** | 1793 | imu660ra_driver 935 / imu_attitude_fixed 3272（atan 843 + sqrt 152 + divider 178 + bias 135 + 自身 1964）/ asr_yaw_unwrap 149 |
| **UI / OLED 显示层** | **3968** | **22.5%** | 966 | system_pages 2422（含 ad 254）/ tof_page 595 / system_oled_text 770（含 i2c 54）/ keys 124 / menu 57 |
| **运动 / 路线 / 控制链** | **3874** | **21.9%** | 1213 | motion FSM 1864 / obstacle_route 1399 / control 顶层胶合 493 / obstacle_command_buffer 22 / asr_distance×2 96 |
| **编码器 / 测速 / PI / 斜坡 / 电机驱动** | **2066** | **11.7%** | 1038 | 测速×2 755 / PI×2 513 / 斜坡 423 / TB6612 181 / 编码器×2 194 |
| **ToF 三通道** | **1836** | **10.4%** | 981 | obstacle_sensors 自身 432 + 3×tof_driver（含 bus/bits）469/466/469 |
| **ASR 收/发协议** | **774** | **4.4%** | 381 | asr_packet_tx 559 / asr_uart_rx 215 |
| **模式调度 dispatch** | **287** | **1.6%** | 138 | system_dispatch |
| **USB 事件日志** | **390** | **2.2%** | 198 | system_event_logger（含 serial 78）+ 2 逻辑 BSRAM |
| **顶层胶合** | **106** | **0.6%** | 151 | robot_system_top 自有 |
| **合计** | **17657** | 100% | **6859** | 与 PnR 17661 相差 4（技术映射重新分配 46 LUT↔50 ALU） |

> 结论：前三个子系统合计 **69%**；若要显著腾资源，只能从 **UI 诊断页 / IMU roll-pitch / 运动路线** 三者之一动刀——其中前两者含“显示/诊断性质”的可裁剪逻辑，运动路线是竞赛核心不建议动。

---

## 4. 逐模块资源表（`Embodied_RPACS_syn_rsc.xml`，按 Logic 降序）【实测】

列说明：`own` = 本模块自身原语；`T_*` = 含子模块总计；`Logic` = `T_Lut + T_Alu`；DSP 按 Gowin 口径（MULTALU27X18=1、MULT27X36=2、MULT12X12=0.5）。

| 层级路径 | 模块 | own LUT | own ALU | own REG | T_LUT | T_ALU | T_REG | Logic | BSRAM | DSP |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `robot_system_top` | 顶层 | 44 | 62 | 151 | 14912 | 2745 | 6859 | 17657 | 7 | 8.5 |
| `control` | system_motion | 488 | 5 | 234 | 8766 | 2591 | 4563 | **11357** | 4 | 8.0 |
| `control/attitude` | imu_attitude_fixed | 1524 | 440 | 874 | 2593 | 679 | 1298 | **3272** | 0 | 4.0 |
| `pages` | system_pages | 2168 | 0 | 201 | 2422 | 0 | 253 | **2422** | 0 | 0 |
| `control/motion` | motion_control_and_mode_fsm | 1338 | 526 | 488 | 1338 | 526 | 488 | **1864** | 0 | 0 |
| `sensors` | obstacle_sensors | 432 | 0 | 282 | 1836 | 0 | 981 | **1836** | 0 | 0 |
| `control/route` | obstacle_route | 1013 | 386 | 354 | 1013 | 386 | 354 | **1399** | 0 | 0 |
| `control/imu` | imu660ra_driver | 845 | 0 | 366 | 935 | 0 | 429 | **935** | 4 | 0 |
| `control/attitude/atan_unit` | imu_attitude_atan2 | 652 | 191 | 134 | 652 | 191 | 134 | **843** | 0 | 0 |
| `display` | system_oled_text | 693 | 23 | 586 | 747 | 23 | 622 | **770** | 1 | 0.5 |
| `tof_display` | tof_page | 595 | 0 | 0 | 595 | 0 | 0 | **595** | 0 | 0 |
| `control/packets` | asr_packet_tx | 449 | 26 | 206 | 533 | 26 | 252 | **559** | 0 | 0 |
| `sensors/channels[0].driver` | tof_driver | 212 | 0 | 86 | 469 | 0 | 233 | **469** | 0 | 0 |
| `sensors/channels[2].driver` | tof_driver | 211 | 0 | 86 | 469 | 0 | 233 | **469** | 0 | 0 |
| `sensors/channels[1].driver` | tof_driver | 211 | 0 | 86 | 466 | 0 | 233 | **466** | 0 | 0 |
| `control/ramp` | pwm_and_startup_ramp | 25 | 0 | 16 | 359 | 64 | 39 | **423** | 0 | 0 |
| `logger` | system_event_logger | 243 | 69 | 153 | 321 | 69 | 198 | **390** | 2 | 0 |
| `control/measure_r` | speed_measurement_and_velocity_filter | 242 | 137 | 345 | 242 | 137 | 345 | **379** | 0 | 0 |
| `control/measure_l` | speed_measurement_and_velocity_filter | 239 | 137 | 350 | 239 | 137 | 350 | **376** | 0 | 0 |
| `control/dispatch` | system_dispatch | 272 | 15 | 138 | 272 | 15 | 138 | **287** | 0 | 0 |
| `sensors/channels[2].driver/bus` | tof_i2c_reg | 65 | 0 | 48 | 258 | 0 | 147 | **258** | 0 | 0 |
| `control/pi/right_channel` | speed_pi_pid_channel | 98 | 159 | 29 | 98 | 159 | 29 | **257** | 0 | 0 |
| `sensors/channels[0].driver/bus` | tof_i2c_reg | 65 | 0 | 48 | 257 | 0 | 147 | **257** | 0 | 0 |
| `control/pi/left_channel` | speed_pi_pid_channel | 97 | 159 | 29 | 97 | 159 | 29 | **256** | 0 | 0 |
| `sensors/channels[1].driver/bus` | tof_i2c_reg | 63 | 0 | 48 | 255 | 0 | 147 | **255** | 0 | 0 |
| `pages/ad` | attitude_angle_to_digits_pipeline | 254 | 0 | 52 | 254 | 0 | 52 | **254** | 0 | 0 |
| `control/receiver` | asr_uart_rx | 199 | 16 | 129 | 199 | 16 | 129 | **215** | 0 | 0 |
| `control/ramp/left_channel` | pwm_startup_ramp_channel | 168 | 32 | 12 | 168 | 32 | 12 | **200** | 0 | 0 |
| `control/ramp/right_channel` | pwm_startup_ramp_channel | 166 | 32 | 11 | 166 | 32 | 11 | **198** | 0 | 0 |
| `sensors/channels[2].driver/bus/bits` | tof_i2c_bits | 193 | 0 | 99 | 193 | 0 | 99 | **193** | 0 | 0 |
| `sensors/channels[0].driver/bus/bits` | tof_i2c_bits | 192 | 0 | 99 | 192 | 0 | 99 | **192** | 0 | 0 |
| `sensors/channels[1].driver/bus/bits` | tof_i2c_bits | 192 | 0 | 99 | 192 | 0 | 99 | **192** | 0 | 0 |
| `control/driver` | tb6612_driver_interface | 30 | 0 | 50 | 181 | 0 | 80 | **181** | 0 | 0 |
| `control/attitude/divider_unit` | imu_attitude_divider | 178 | 0 | 111 | 178 | 0 | 111 | **178** | 0 | 0 |
| `control/attitude/sqrt_unit` | imu_attitude_sqrt | 104 | 48 | 108 | 104 | 48 | 108 | **152** | 0 | 0 |
| `control/unwrap` | asr_yaw_unwrap | 85 | 64 | 66 | 85 | 64 | 66 | **149** | 0 | 0 |
| `control/attitude/bias_divider` | imu_attitude_bias_divider | 135 | 0 | 71 | 135 | 0 | 71 | **135** | 0 | 0 |
| `keys` | system_keys | 124 | 0 | 69 | 124 | 0 | 69 | **124** | 0 | 0 |
| `control/enc/left_channel` | dual_ab_encoder_channel | 34 | 63 | 84 | 34 | 63 | 84 | **97** | 0 | 0 |
| `control/enc/right_channel` | dual_ab_encoder_channel | 34 | 63 | 82 | 34 | 63 | 82 | **97** | 0 | 0 |
| `control/imu/spi_master` | imu660ra_spi_master | 90 | 0 | 63 | 90 | 0 | 63 | **90** | 0 | 0 |
| `control/packets/sender` | uart_tx | 84 | 0 | 46 | 84 | 0 | 46 | **84** | 0 | 0 |
| `logger/serial` | uart_tx | 78 | 0 | 45 | 78 | 0 | 45 | **78** | 0 | 0 |
| `control/driver/left_channel` | tb6612_driver_channel | 76 | 0 | 15 | 76 | 0 | 15 | **76** | 0 | 0 |
| `control/driver/right_channel` | tb6612_driver_channel | 75 | 0 | 15 | 75 | 0 | 15 | **75** | 0 | 0 |
| `menu` | system_menu | 57 | 0 | 22 | 57 | 0 | 22 | **57** | 0 | 0 |
| `display/i2c_writer` | oled_i2c_write_byte | 54 | 0 | 36 | 54 | 0 | 36 | **54** | 0 | 0 |
| `control/distance_l` | asr_distance | 2 | 46 | 46 | 2 | 46 | 46 | **48** | 0 | 0 |
| `control/distance_r` | asr_distance | 2 | 46 | 46 | 2 | 46 | 46 | **48** | 0 | 0 |
| `control/buffer` | obstacle_command_buffer | 22 | 0 | 45 | 22 | 0 | 45 | **22** | 0 | 0 |
| `control/imu/config_rom` | imu660ra_config_rom | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 4 | 0 |

> **BSRAM 口径**：综合 XML 记 7 个逻辑块（config_rom 4 + logger debug FIFO 2 + 字体 pROM 1）；PnR 报 9 个物理块（SDPB 4 + pROM 5）。最可能的解释是 logger 的 32×64 debug FIFO 因位宽被拆成 4 个 SDPB（综合侧记 2）。以 PnR 的 9/56 为准。
> **DSP 口径**：attitude 4×MULTALU27X18（gx/gy/gz×1250 与加速度平方）+ control 2×MULT27X36（左右速度换算 32×32 乘）+ display 1×MULT12X12（无实际乘法语义，综合映射）。
> **重要发现（寄存器≠源码位宽）**：源码 `tof_snapshot[671:0]` 与 `display_text[671:0]` 在 50 MHz 时序驱动综合下被**寄存重定时/常量位折叠**：网表中不存在 672-bit 快照寄存器（顶层仅 151 个 FF），文本寄存器只保留变化位。因此**不要按源码位宽估算 FF 收益**；改动前必须重综合确认。

### 4.1 与网表原语清单的交叉核对

用 `tools/netlist_resource_report.py` 思路对 `impl/gwsynthesis/Embodied_RPACS.vg` 做全原语统计（本次审计用只读脚本，见 §11）：

- 网表原语总计：LUT4 9314 + LUT3 3840 + LUT2 1691 + MUX2_LUT5/6/7 305 + INV 67 + **ALU 2745** = 17962（网表级，PnR 后 17661）+ DFFCE 6493 + DFFRE 221 + DFFPE 145 = **6859 FF** + SDPB 2（网表内可见的 debug FIFO，其余 BSRAM 由 PnR 从 ROM/RAM 推断）。
- 逐模块 roll-up 与 XML 的 `T_Lut + T_Alu` 在 `pages(2422)`、`motion(1864)`、`route(1399)`、`atan(843)` 等模块**完全一致**；`system_motion` 11633 vs 11357 的差异来自网表内未实例化的参数副本定义，属统计口径差异。
- 结论：综合 XML 是可信的逐模块成本来源，本文收益估算以其为准。

---

## 5. 冗余 / 未使用 / 可复用清单

### 5.1 已确认冗余但**已被综合裁剪、0 资源成本**的代码（只影响可读性/综合时间）

| 项 | 位置 | 现状 | 建议 |
| --- | --- | --- | --- |
| `tof_three` 模块 | `src/tof/tof_driver.v:106-168` | 未实例化（主工程用 `obstacle_sensors` 内 3×`tof_driver`），综合无成本 | 可删，但 `tests/vl53l0x_three` 历史工程可能参考同文件；**删除前确认测试依赖** |
| `obstacle_turn_trace` / `obstacle_command_trace` | `src/obstacle/obstacle_command.v:24-82` | 主工程未实例化；**`tests/obstacle_avoidance` 的 tb 仍在引用** | **不建议删**（0 收益，会破坏测试工程） |
| `imu660ra_driver` 诊断输出（`chip_id/init_status/power_conf_readback/init_ctrl_readback/sample_sequence`） | `src/imu/imu660ra_driver.v` | 主工程接空 → 已裁剪；`sim/tb_imu660ra_driver.v` 使用 | 保留端口 |
| `imu_attitude_fixed` 的 `gyro_bias_x/y/z`、`system_oled_text` 的 `frame_done` | 同上 | 接空 → 已裁剪 | 保留 |
| `src/imu/imu_attitude_oled.v`、`imu_uart_telemetry.v`、`closed_loop_uart_telemetry.v`、`encoder_uart_telemetry.v`、`uart_heartbeat_watchdog.v`、`chassis_*.v`、`power_and_hardware_safety.v`、`safety_protection_and_fault_arbitration.v`、`robot_*_test_top.v` | `src/` | 不在 `build.tcl`，仅测试工程使用 | **保持现状**（不要加入主构建） |

> 净收益：**0 LUT**。仅建议在“清理轮”统一处理，不要和优化混做。

### 5.2 结构重复（同构多份，可共享）

| 重复结构 | 份数 | 单份实测 | 合计实测 | 可共享性 |
| --- | --- | --- | --- | --- |
| `tof_driver` 通道 FSM（3 通道并行） | 3 | own 212/211/211 LUT + 86 FF | **~634 LUT / 258 FF**（driver 自身） | 高：可串行共享 1 份 |
| `tof_i2c_reg` + `tof_i2c_bits` 总线引擎 | 3 | own 65/65/63 + 192/192/193 LUT | **~771 LUT / 441 FF** | 中：位引擎可共享但需引脚 mux/仲裁 |
| `speed_measurement_and_velocity_filter` | 2 | 239+137 ALU / 242+137 ALU | **755 logic / 695 FF** | 高：10 ms 窗口内 2 拍分时 |
| `speed_pi_pid_channel` | 2 | 97/98 LUT + 159/159 ALU | **513 logic / 58 FF** | 高：1 ms tick 内 2 拍分时 |
| `pwm_startup_ramp_channel` | 2 | 168/166 LUT + 32/32 ALU | **398 logic / 23 FF** | 高：PWM 周期 2500 clk 内 2 拍分时 |
| `dual_ab_encoder_channel` | 2 | 34 LUT + 63 ALU | 194 logic / 166 FF | 低：本身很小，共享收益~50 LUT（不建议） |
| `asr_distance` | 2 | 2 LUT + 46 ALU | 96 logic / 92 FF | 低：小数乘法常量，共享不值得 |
| `tb6612_driver_channel` | 2 | 76/75 LUT | 151 logic / 30 FF | 低：死区状态机，共享复杂（不建议） |
| `asr_packet_tx` FIFO 深度 16 | 1 | own 449 LUT + 206 FF | — | 可缩到 8（最坏单拍突发 4 条） |
| `attitude_angle_to_digits_pipeline` | 1（已从 3 份共享为 1 份，README 记录） | 254 LUT | — | 已优化到位 |
| OLED 字面量字符串 mux（`label()` ×2 处） | 2 | 估算 200–350 LUT/处 | 估算 0.4–0.7k | 中：可改 pROM 或减少一处 |

### 5.3 顶层“已布线但逻辑未使用”的输入（0 逻辑开销，不建议动）

综合日志 `CV0016` 明确：`uart_rx`、`imu_int1`、`imu_int2`、`imu_gnd_reserved`、`tof_front/left/right_gpio1` 共 **7 个顶层输入未参与逻辑**。它们已在 `robot_system_top.cst` 约束并上板布线，保留约束即可（`uart_rx` 未用是设计意图：USB 只输出事件日志）。删除端口/约束不会带来逻辑收益，反而破坏板级兼容与 `tools/check_system_reports.ps1` 的 44 引脚验收，**不要动**。

---

## 6. 优化候选与定量收益

> 标注规则：**【实测】**=被删/被替换模块在 XML 中的 `T_Lut/T_Alu/T_Register` 原值；**【估算】**=源码结构推导。所有最终收益须以重综合报告复核（±10% 误差）。

### 6.1 Tier A —— 不裁剪功能的结构优化（推荐先做）

| # | 优化项 | 机制 | 收益 | 风险 | 用户可见变化 |
| --- | --- | --- | --- | --- | --- |
| A1 | **ToF 三通道共享 1 套驱动 FSM**（保留 3 个 I2C 总线引擎与 3 组结果寄存器） | 3 个 `tof_driver` 的 init/poll/read 序列状态机完全相同；改为 1 份 FSM + 通道索引轮转，3 个 `tof_i2c_reg` 各自独立，XSHUT 分时释放 | **实测可回收 0.42k LUT + 0.17k FF**（2×212 LUT + 2×86 FF）；扣除通道选择 glue（估算 20–50 LUT）→ **净 ~0.37–0.40k LUT / 0.17k FF** | 中：每通道刷新周期约 ×3（估算单次 30–40 ms → 全轮 100–120 ms），避障依赖的样本新鲜度下降；route 超时（10 s/1 s）仍有大余量 | 无（避障反应可能略慢，需上板确认） |
| A2 | **左右测速/滤波共享**（保留左右历史样本与位置积分） | 同一 10 ms 窗口内串行处理 L→R（每窗口多 1–2 拍） | **实测 0.38k logic + 0.35k FF**（一个实例 239 LUT+137 ALU+350 FF）；净估 **0.34–0.37k** | 中低：需保证两侧 `speed_valid` 在 PI tick 前都更新（相邻两拍即可），`control_tick=valid_l&&valid_r` 逻辑相应调整 | 无 |
| A3 | **左右 PWM 斜坡共享算术引擎** | tick 后 2 拍分别处理左右；duty/direction 寄存器仍各 1 份 | **实测 0.20k logic**（一个 channel 168 LUT+32 ALU）；净估 **0.17–0.19k** | 低：PWM 周期 2500 clk，2 拍裕量极大 | 无 |
| A4 | **左右 PI 共享算术引擎** | 控制节拍内 2 拍分时；左右状态/积分寄存器保留 | **实测 0.256k logic**（一个 channel 97 LUT+159 ALU）；净估 **0.23–0.26k** | 中：需复核饱和/条件积分逻辑时序，闭环仿真对比 | 无（延迟 1 拍） |
| A5 | **ASR 发送 FIFO 深度 16→8** | 最坏单拍突发 4 条（pair 2 + event 1 + aux 1），深度 8 仍留 2× 余量；UART 每 6 字节约 0.52 ms 排空 1 条 | **估算 0.10–0.15k LUT + 0.06–0.10k FF**（FIFO 位宽 16→8 的一半） | 低：仿真验证最坏突发不溢出（溢出会锁存 fault） | 无 |
| A6 | `system_keys` 去抖计数器 32→19 bit | 去抖 500000 周期只需 19 位 | **估算 ~10 LUT + 39 FF** | 极低 | 无 |
| A7 | 删除 `tof_three`（未实例化）等死代码 | 见 §5.1 | **0 LUT**（已被裁剪），只改善可读性/综合时间 | 无 | 无 |
| ~~A8~~ | ~~删除顶层 `tof_snapshot` 672-bit 快照~~ | **审计发现它已被综合重定时吸收（顶层仅 151 FF），收益为 0** | 0 | — | 不作为优化项 |

**Tier A 小计（估算）：≈ 1.2–1.4k LUT、0.65–0.70k FF。**
注意：A1/A2/A3/A4 每一项都要改 RTL、跑 7 套仿真并重新上板；建议一次只做 1–2 项并单独出报告，否则无法归因。

### 6.2 Tier B —— 需要用户确认的功能裁剪（收益最大）

| # | 候选 | 机制 / 涉及文件 | 收益 | 风险 | 失去什么 |
| --- | --- | --- | --- | --- | --- |
| B1 | **删除 DISPLAY 组 5 个诊断页**（SYSTEM STATUS / IMU ATTITUDE / TOF DISTANCE / ENCODER-SPEED / VOICE-MOTION） | 删 `system_pages.v` 中 `group_id==1` 的分页分支；删 `robot_system_top.v` 的 `tof_page`/`tof_snapshot`/`ad` 引用；菜单 `system_menu.v` 的 DISPLAY 子项缩为 0 | **实测 0.85k LUT**（`tof_page 595` + `ad 254`）**【实测】** + pages 内页内容/文本选择 mux **估算 0.3–0.8k** → **合计 ≈1.1–1.6k LUT**；FF 约 0.1–0.2k | 低（纯 UI，无控制依赖） | 5 个现场诊断显示页（仍可用 USB 事件日志/上位机看部分状态）；`system_oled` 等仿真期望文本需更新 |
| B2 | **IMU 只保留 yaw（陀螺积分），删除 roll/pitch 加速度校正** | `imu_attitude_fixed.v` 删 atan/sqrt/divider 与加速度平方/误差/融合分支；保留陀螺零偏校准（可只留 gz）与 yaw 积分；`system_pages.v` 姿态页随 B1 一起删 | **实测子单元 1.17k**（atan 843 + sqrt 152 + divider 178）**【实测】** + attitude 自身组合逻辑 **估算 0.4–0.7k** → **≈1.5–1.9k LUT**；FF 0.45–0.65k；DSP −1 | 中（功能取舍；RTL 重写需重跑 `attitude_pipeline`/`imu_attitude` 仿真） | roll/pitch 角（当前**仅用于 OLED 显示**，不参与任何运动控制）；未来倾角/坡道补偿能力；若将来要合入 uart_bridge 的 IMU 遥测变体（roll/pitch/yaw 上报）则**不能删** |
| B3 | **删除 USB 事件日志**（`system_event_logger` + `debug_state` 64-bit 遥测） | 删 `system_event_logger` 实例、`debug_state/debug_valid/event_errors` 组装；`uart_tx` 顶层端口与 `.cst` 的 `uart_tx=V8` 一并处理（或保留引脚空置） | **实测 0.39k logic + 198 FF + 2 逻辑 BSRAM【实测】** + motion 侧 debug 组装 **估算 30–60 LUT** → **≈0.42–0.45k LUT / 0.26k FF / 2 BSRAM** | 低（无控制依赖），但需同步改 `check_system_reports.ps1` 的引脚期望 | 上电心跳 `D 0000...` 行、S/E 错误事件行（README 的板级验收证据依赖它） |
| B4 | **菜单删除“下一项预览”第三行** | `system_pages.v` 删 `r2=label(group_id,next_item)`（保留 `current_label`） | **估算 0.2–0.28k LUT**（一个 168-bit×7-bit 字面量 mux 实例） | 极低 | 菜单第三行不再预览下一项，按键操作不变 |
| B5 | 若只删 debug D 行、保留 S/E 事件行 | 删 `debug_state/debug_fifo/debug_previous` 相关逻辑，保留 `pending` 事件上报 | **估算 0.15–0.25k LUT + 2 BSRAM**（logger 的 debug FIFO 是 BSRAM 大头） | 低 | 失去 64-bit 状态快照 D 行，保留事件行 |

**Tier B 小计（估算）：B1–B3 主体 ≈ 3.0–4.0k LUT / 0.8–1.1k FF / 2–4 BSRAM；再加可选 B4/B5 约 +0.2–0.5k LUT。**

### 6.3 Tier C —— 结构性重构（收益中等、风险偏高，非必要不做）

| # | 候选 | 机制 | 收益 | 风险 |
| --- | --- | --- | --- | --- |
| C1 | `pages` 字面量文本改字符 ROM / 字符流 | 把 45 条 21 字符标签表放进 1–2 个 pROM（BSRAM 有 47 空闲），OLED 扫描时逐字符取数，消除 672-bit 并行 mux；动态数字页仍用 LUT | **估算 0.6–1.0k LUT**，+1–2 BSRAM | 中高：改 `system_pages`↔`system_oled_text` 接口与扫描时序，需重写 `system_oled` 仿真 |
| C2 | `display_text` 672-bit 寄存器 → SDPB 帧缓存 | 每帧把 text 写入 84×8 RAM，扫描时顺序读，消除 672:1 字符 mux | **估算 0.25–0.35k LUT**，+1 SDPB | 中：写帧/读扫描时序与 clear 流程要重排 |
| C3 | ToF I2C 位引擎 3→1（引脚 mux + 仲裁） | 在 A1 基础上再共享 `tof_i2c_bits` | **估算再省 0.38k LUT**（2×192） | 中高：三总线并发→串行，SDA/SCL 三态 mux 与仲裁易出错 |
| C4 | 三通道 ToF 全串行（A1+C3 组合） | 单引擎轮询三路 | ≈0.8k LUT | 高：采样率再降，需重新标定避障参数 |

**Tier C 小计：≈1.2–1.7k LUT。**

### 6.4 收益汇总与落地后预测（全部为估算）

| 方案 | LUT/Logic 节省 | 剩余 Logic | 剩余 CLS（按 1.79 logic/CLS 折算） | 说明 |
| --- | --- | --- | --- | --- |
| 现状 | — | 17661（77%） | 9858（86%） | — |
| 仅 Tier A | ≈1.2–1.4k | ≈16.3–16.5k（71%） | ≈9.1–9.2k（79%） | 不改行为，最安全 |
| Tier A + B（推荐路径） | ≈4.2–5.4k | ≈12.3–13.5k（53–59%） | ≈6.8–7.5k（59–65%） | 需用户确认 B 项 |
| Tier A + B + C | ≈5.4–7.1k | ≈10.6–12.3k（46–53%） | ≈5.9–6.9k（51–60%） | C 项无必要，仅作储备 |

> 预测是线性折算，实际 CLS 与逻辑混合方式有关，**必须以重综合+PnR 报告复核**。Tier A+B 落地后，CLS 余量可从 1662 提升到 ~4.2k，届时合入 uart_bridge 完整链路（881 CLS）才比较从容。

---

## 7. 优先级优化计划

### 7.1 排序总表（按“收益×低风险”排序）

| 优先级 | 项 | 类型 | 估算收益 | 风险 | 是否改用户可见行为 | 建议批次 |
| --- | --- | --- | --- | --- | --- | --- |
| P1 | B1 DISPLAY 诊断页裁剪 | 功能裁剪 | **1.1–1.6k LUT** | 低 | 是（少 5 个诊断页） | 等用户确认后第 1 批 |
| P2 | B2 IMU 仅 yaw | 功能裁剪 | **1.5–1.9k LUT** | 中 | 是（无 roll/pitch） | 等用户确认后第 1 批（与 B1 一起做，因为姿态页本身就属于 B1） |
| P3 | A1 ToF FSM 共享 | 结构共享 | **0.37–0.42k LUT** | 中 | 否（采样略慢） | 第 2 批（单独 A/B 对比） |
| P4 | A2 测速共享 | 结构共享 | **0.34–0.37k LUT** | 中低 | 否 | 第 2 批 |
| P5 | B3 删除 USB 事件日志 | 功能裁剪 | **0.42–0.45k LUT + 2 BSRAM** | 低 | 是（无 USB 日志） | 等用户确认 |
| P6 | A4 PI 共享 / A3 斜坡共享 | 结构共享 | **0.40–0.45k LUT（合计）** | 中低 | 否 | 第 3 批 |
| P7 | A5 FIFO 16→8；A6 keys 位宽 | 参数级 | **0.11–0.16k LUT** | 低 | 否 | 随任一批次顺手做 |
| P8 | B4 菜单预览删除 | 功能裁剪 | **0.2–0.28k LUT** | 极低 | 是（菜单少一行） | 等用户确认 |
| P9 | C1 pages 文本 ROM 化 | 重构 | 0.6–1.0k LUT + BSRAM | 中高 | 否（若实现正确） | 最后手段 |
| P10 | C2 display_text 改 RAM | 重构 | 0.25–0.35k LUT | 中 | 否 | 最后手段 |
| P11 | C3 ToF I2C 位引擎共享 | 重构 | 0.38k LUT | 中高 | 否 | 只在需要极限压缩时 |

### 7.2 每一项的验证方法（统一要求）

1. **RTL 级**：`./tools/run_system_sim.ps1`（7 套：`system`、`system_oled`、`system_50mhz`、`system_dispatch`、`live_speed`、`system_obstacle`、`attitude_pipeline`），要求每套日志尾行为 `Errors: 0, Warnings: 0`。
2. **单元/历史回归**（涉及电机/测速/PI/斜坡/按键时）：`./tools/run_legacy_regression.ps1`（imu_attitude、imu_attitude_oled、speed_prefetch、ground_tests、closed_loop_test、dual_closed_loop_test、right_closed_loop_test、motion、tb6612、motion_invalid）。
3. **构建**（Lead 执行）：`& 'D:/Gowin/Gowin_V1.9.12_x64/IDE/bin/gw_sh.exe' build.tcl`，必须 **0 error、无新增 swept/black-box 警告**。
4. **报告验收**：`./tools/check_system_reports.ps1`（44 引脚、电气属性、10 个 open-drain、CPU/SSPI GPIO、单 owner 模块、零 setup/hold 违例、位流新鲜）。
5. **资源对比**：新 `impl/pnr/Embodied_RPACS.rpt.txt` + `_syn_rsc.xml` 与本文基线逐项 diff；记录 Logic/CLS/FF/BSRAM/DSP 的实测差值（不要相信估算）。
6. **时序**：`impl/pnr/Embodied_RPACS_tr_content.html` 中 `Numbers of Setup/Hold Violated Endpoints = 0`、TNS=0、`Actual Fmax ≥ 50 MHz`；**若 Fmax 跌破 50 或 slack < 0.5 ns 则回退**。
7. **上板**（行为相关项）：USB 每秒心跳、OLED 页面、语音命令、避障绕行、K0 急停；至少覆盖 `records/obstacle_20261025/board_smoke_final.log` 的同等项。

### 7.3 实施顺序建议

1. **先做 0 风险清理**：A7 死代码/文档（0 收益，但让代码可读）、重跑一次基线确认 17661/9858。
2. **等用户回答 §8 问题** → 若同意 B1/B2，一次做完“诊断页 + IMU 仅 yaw”，预期 −2.6～3.5k LUT；必须重写 `system_oled`/`attitude_pipeline` 仿真期望。
3. **每轮只加 1 个 Tier A 项**（A1 → A2 → A4/A3 → A5/A6），每轮独立构建、记录实测收益，避免收益归因混乱。
4. **B3（删日志）** 放最后做，因为它会改变板级验收证据形态；如果 Lead 还需要 USB 日志做验收，则保留 B5（只删 D 行）。
5. Tier C 只在 Tier A+B 仍不够时启动。

---

## 8. 待用户确认的“可删除功能”清单（可直接粘贴给用户）

> 建议 Lead 用下面 4 个问题向用户确认（每项都给出代价与失去的功能）：

1. **是否删除 DISPLAY 菜单组里的 5 个诊断页**（SYSTEM STATUS / IMU ATTITUDE / TOF DISTANCE / ENCODER·SPEED / VOICE·MOTION）？
   - **节省**：约 **1.1–1.6k LUT**（其中 `tof_page` 595、姿态数字转换 254 为实测；其余为页内容估算）。
   - **失去**：现场 OLED 诊断显示（状态码、三路 ToF 五位毫米、编码器计数/速度、姿态角）。不影响按键/手动/设置/避障面板。
2. **是否把 IMU 姿态简化为“只算 yaw”**（删除 roll/pitch 加速度融合，保留陀螺零偏校准与 yaw 积分）？
   - **节省**：约 **1.5–1.9k LUT**（其中 atan 843 + sqrt 152 + divider 178 为实测），少 1 个 DSP。
   - **失去**：roll/pitch 角（目前只用于 OLED 显示，**不参与任何运动控制**；90° 转向只用 yaw，不受影响），以及未来的倾角补偿能力。
   - **注意**：若计划以后把 `tests/uart_bridge` 的 IMU 遥测变体（上报 roll/pitch/yaw）合入主工程，则**不建议删**。
3. **是否删除 USB 事件日志**（`system_event_logger` 与 64-bit 调试快照）？
   - **节省**：约 **0.42–0.45k LUT、198+ FF、2 个 BSRAM 块**。
   - **失去**：上电心跳 `D 0000...` 行与 S/E 错误事件行（目前 README 的板级验收证据依赖它）。若只希望省一半，可只删 D 行、保留 S/E 事件行（约 0.15–0.25k LUT + 2 BSRAM）。
4. **是否删除主菜单的“下一项预览”行**（菜单第三行不再显示下一项名称）？
   - **节省**：约 **0.2–0.28k LUT**（估算）。
   - **失去**：菜单预览提示；按键操作与功能不变。

---

## 9. 12036/53% 与 17661/77% 的差异溯源（结论 + 证据链）

### 9.1 证据链（按时间顺序）

| 时间 | 证据 | 内容 |
| --- | --- | --- |
| 10-28 21:52 | commit `0e8b98f` | “baseline snapshot before main-project resource optimization”，README 记优化前 **20334（89%）** |
| 10-29 02:06:09 | `records/obstacle_20261025/build_opt3.log`（第 178 行时间戳） | 该次 PnR 完成；日志内 **3 条 `WARN (NL0002) : The module ... is swept in optimizing`**：`system_pages`(robot_system_top.v:113)、`attitude_angle_to_digits_pipeline`(system_pages.v:75)、`tof_page`(robot_system_top.v:105) |
| 10-29 02:09:38 | commit `09f2766` message | “perf: cut main project logic from 89% to 53% and raise Fmax to 66.7 MHz”；同时提交 `build_opt3.log` 与 `src/ui/oled_font.hex` 等 |
| 10-29 02:35:43 | commit `c66be42`（tests/uart_bridge 文档） | 引用“`impl/pnr/Embodied_RPACS.rpt.txt`，2026-10-29 02:06”得 **Logic 12036（53%）、Register 5138、CLS 7525**，并记录“基线变化 20334/89% → 19664/86% → 12036/53%” |
| 10-29 04:16:34 | `records/obstacle_20261025/build_final.log` | 重新构建：**无 swept 警告**；P nR 报告 **17661/6859/9858** |
| 10-29 04:32:04 | commit `f22c6e0` | “fix: green all seven suites and verified final bitstream on board”，更新 README 为 **17661/77%**；`git diff 09f2766 HEAD -- src/` 仅 3 个文件 +32/−15 行 |
| 10-29 11:40:35 | 本次审计期间 Lead 重建 | 与 04:16 **逐项一致**：17661/6859/9858/9/8.5，Fmax 53.020，0 违例 |

### 9.2 根因（可复现的代码级解释）

`09f2766` 在把 OLED 字体改成 768×8 ROM 时，`src/ui/system_oled_text.v` 中的 `font_byte` **只在复位分支被赋值 0，没有任何其他驱动**：

```verilog
// 09f2766 当时的代码（git show 09f2766:src/ui/system_oled_text.v）
reg [7:0] font_byte;
...
font_byte <= 0;            // 仅复位赋值 → 运行期恒为 0
...
byte_data = font_byte;     // 点阵数据恒 0
```

`font_byte` 恒 0 后，`screen_character`→`font_addr`→字体 ROM 读取整条链变成“常数”，于是综合判定以下输出无人使用并**整块裁剪（sweep）**：

- `system_pages`（2422 logic）：菜单/页面文本生成全部消失（其输出 `text` 只喂给已死的字模链）→ 连带 `pages/ad`（254）一起消失；
- `tof_page`（595）：TOF DISTANCE 页文本只喂 `pages.text` → 消失；
- `display` 的字符扫描/字模读取部分：只剩 I2C 状态机（`font_byte` 恒 0 时无字模可读）。

**因此 02:06 那次构建报告的 12036/5138/7525 是一个“OLED 文本不显示”的非功能网表**（寄存器差 1721 ≈ pages 253 + ad 52 + display 大半 + 顶层文本链被重定时后的余量；Logic 差 5625 中 pages+tof_page+display 文本链即可解释大部分）。之后 `f22c6e0` 补上了同步读：

```verilog
// 现在（HEAD / 11:40 构建）：字体 ROM 同步读恢复数据通路
always @(posedge clk or negedge rst_n)
if (!rst_n) font_byte <= 0;
else font_byte <= font_rom[font_addr];
```

重建后 3 条 swept 警告消失，数字回到 **17661/77%**，README 也随 `f22c6e0` 更新为 77%。

### 9.3 结论与影响

1. **17661/77% / 6859 / 9858 CLS 是唯一有效的功能完整基线**；`12036/53%`（以及随之的 Fmax 66.7 MHz）来自**功能不完整的中间网表，不能用于余量规划或合入评估**。
2. 该错误数字已扩散到 3 处文档：`tests/uart_bridge/README.md` §6、`tests/uart_bridge/docs/INTEGRATION_GUIDE.md` §4、`tests/uart_bridge/records/2026-10-29_*`；**建议 Lead 让相关文档 Owner 修正**（本次审计只读，未改）。
3. **对 uart_bridge 合入评估的影响（重要）**：
   - 文档原结论“主工程 CLS 余 3995，完整链路 858 CLS 可宽松并入，合入后余 3137”**不成立**。
   - 真实余量只有 **1662 CLS**。按 `tests/uart_bridge/impl/pnr/*.rpt.txt` 实测：完整版 1174 Logic / 881 CLS、精简版 1110 / 830、IMU 变体 5430 / 3671：
     - 合入完整版：9858 + 881 = **10739/11520（93.2%）**——能放但很紧，布线/时序风险高，且还要加适配器（文档估 ~50 CLS）；
     - 合入精简版：10688（92.8%），同样紧；
     - 合入 **IMU 变体：13529 > 11520，装不下**。
   - 因此：**合入前先执行 §7 的 Tier A（≈1.3k LUT，约 0.7k CLS）甚至 Tier B**，把 CLS 压到 ≤80% 再合，才是低风险路径。

---

## 10. 时序（Fmax）与后续扩展余量

### 10.1 时序现状（`impl/pnr/Embodied_RPACS_tr_content.html`，11:40:35）【实测】

| 项目 | 值 |
| --- | --- |
| 约束时钟 | `clk_50m` 20.000 ns / 50.000 MHz（`src/robot_system_top.sdc`） |
| **Actual Fmax** | **53.020 MHz**（Logic Level 32） |
| Setup 违例端点 | **0** |
| Hold 违例端点 | **0** |
| **TNS（Setup / Hold）** | **0.000 / 0.000**（0 端点） |
| 最差 Setup slack | **+1.140 ns**（即 20 − 18.86 ns；Fmax 余量约 +6.0%） |
| 最差 Hold slack | **+0.183 ns** |
| 分析路径数 / 端点 | 20626 / 21148 |
| 全局时钟 | PRIMARY 3/8、LW 8/8（**LW 已用满**）、GCLK_PIN 7/40 |

### 10.2 关键路径（决定 Fmax 的前几条）

| # | Slack | 起点 → 终点 | 说明 |
| --- | --- | --- | --- |
| 1 | 1.140 ns | `control/attitude/gyro_numerator_x_4_s0/Q` → `control/attitude/roll_mdeg_22_s0/D` | **IMU 陀螺积分**：`gyro_numerator_x / 4096` → `step_x` → `wrap_angle(roll_mdeg+step_x)`（32-bit 加/比较/回绕） |
| 2 | 1.995 ns | `control/target_um_q_l_3_s0/Q` → `control/target_l_14_s0/D` | **速度换算乘法链**：`convert_speed()` 的 32×32 乘法（MULT27X36→DSP）+ 移位/饱和 |
| 3 | 2.072 ns | `control/attitude/gyro_numerator_x_4_s0/Q` → `control/attitude/roll_mdeg_19_s0/D` | 同 1 的其它位 |
| 4 | 2.263 ns | `control/target_um_q_l_3_s0/Q` → `control/target_l_15_s0/D` | 同 2 的其它位 |
| 8 | 2.753 ns | `control/target_um_q_r_1_s0/Q` → `control/target_r_14_s0/D` | 右轮同类路径 |

### 10.3 对未来功能的余量评估与风险

| 维度 | 现状 | 未来新增逻辑的风险 |
| --- | --- | --- |
| **Fmax 余量** | +6.0%（18.86 ns vs 20 ns） | 若在 **attitude 更新链**或 **convert_speed 乘法链**上新增加法/比较/选择，每多一级 32-bit 组合逻辑约 1–2 ns，**很可能直接跌破 50 MHz**。新增逻辑应避开这两条路径（或加流水寄存器）。 |
| **CLS 余量** | 14.4%（1662） | 真正的瓶颈；见 §2.3。 |
| **全局时钟（LW）** | LW 8/8 **已满** | 若新功能要新增低扇出时钟/使能网络，不能再用 LW，需改用普通布线或合并使能。 |
| **PRIMARY** | 3/8 | 还有 5 个，够用。 |
| **BSRAM / DSP / IO** | 9/56、8.5/28、44/222 | 都不紧张；**BSRAM 是未来做数据显示/缓存的低成本方向**（Tier C1 用 1–2 块即可）。 |
| PnR 压力 | CLS 86% | 继续增加 >90% 时布线拥塞/时序收敛风险上升，建议设定“新功能落地后 CLS ≤90%，优化后 ≤85%”的门槛。 |

---

## 11. 附录

### 11.1 复现命令（只读，不触发构建）

```powershell
# 资源汇总（GbK/UTF-8 均可）
Select-String -Path impl/pnr/Embodied_RPACS.rpt.txt -Pattern 'Logic|Register|CLS|BSRAM|DSP|Created Time'
# 逐模块资源（XML 很小，可直接阅读）
Get-Content impl/gwsynthesis/Embodied_RPACS_syn_rsc.xml
# 时序（rpt.txt 不含时序，必须看这两个）
Select-String -Path impl/pnr/Embodied_RPACS_tr_content.html -Pattern 'Actual Fmax|Violated Endpoints|Endpoints TNS'
Get-Content impl/pnr/Embodied_RPACS.timing_paths -TotalCount 60
# 历史 swept 警告（§9 证据）
Select-String -Path records/obstacle_20261025/build_opt3.log -Pattern 'swept|WARN'
Select-String -Path records/obstacle_20261025/build_final.log -Pattern 'swept|WARN'
```

### 11.2 审计方法说明

- 逐模块数字来自 `syn_rsc.xml` 的 `own`/`T_*` 字段（综合后、PnR 前）；用自写只读脚本（`records/resource_20261029/tmp_xml_table.py`，已在交付前删除）展开层级并校验总和 = `T_Lut 14912 + T_Alu 2745 = 17657`，与 PnR 17661 差 4。
- 用另一只读脚本（`tmp_vg_census.py`，同样已删除）统计 `.vg` 网表原语，交叉验证逐模块排名与 FF 总数（6859）。
- **收益口径**：Tier A/B 的“实测可回收”= 对应模块在 XML 中的总值（删除/共享后理论消失的上限）；净收益 = 实测 − glue 估算。**没有任何一项经过实际重综合验证**，实施后必须用新报告核对。

### 11.3 明确不可信的/不要引用的数字

- ❌ `12036 / 53% / CLS 7525 / Fmax 66.7 MHz`（02:06 中间坏构建，§9）。
- ❌ “CLS 余 3995 / 合入 uart_bridge 后仍余 3137”（基于上面坏基线）。
- ❌ 用 `impl/pnr/Embodied_RPACS.rpt.txt` 查 Fmax/TNS（该文件没有 timing 内容）。
- ⚠️ README“预留约 2700 LUT”只是 LUT 口径；按 CLS 口径可用余量更小（见 §2.3）。

---

*本记录由 resource-auditor 于 2026-10-29 完成；全程只读，未参与构建。如需把本文任何估算变成结论，请让 Lead 在实施后用新的 `Embodied_RPACS.rpt.txt` / `_syn_rsc.xml` 复核。*

# Embodied_RPACS 避障演示稳定性修复与资源优化技术报告

- 日期：2026-10-29
- 工程：`Embodied_RPACS` 主工程（ACG525 / GW5A-LV25UG324C2/I1，50 MHz，Gowin 1.9.12）
- 改动前基线提交：`0bd1a77`（其 `src/` 与用户实际运行的 `3556192` 逐字节相同）
- 故障修复提交：`ef37e97`
- 资源优化提交：见文末“提交记录”（本报告与最终构建一并提交）
- 最终位流：`impl/pnr/Embodied_RPACS.fs`（保存工程 `verify_project.tcl` 构建）
  - 大小 6,389,412 字节
  - SHA256 `D3E910465561F096BF32A0B40357AE054553521DA5C7822350B2C7AB7BEED76F`
  - （`build.tcl` 构建同一份源码得到 13257/58% 相同资源，位流 SHA 为
    `A46B70FA7B371BAA45F415ECE5F0A1DD5D1222DAB687FA2F1D43DEF3D9E84B6D`，两者均通过验收检查）
- 证据目录：`records/robustness_20261029/`、`records/fault_injection/`、`records/resource_20261029/`

## 1. 摘要

用户报告：避障演示运行到一半容易报故障，尤其“转 90° 之后停下报错”——遇障左转完成、
或左转前进固定距离后右转完成时，OLED 避障面板出现 `FAULT RESET REQUIRED`，必须按 K0
复位；同时要求稳定并行采集三路传感器、简化复杂逻辑、优化板级资源为后续扩展留余量。

本轮工作分三部分：

1. **故障复现**：把改动前用户的真实源码冻结点（`3556192`/`0bd1a77`）导出为只读基线，
   在 ModelSim 中用 10 个故障注入场景复现了全部锁存路径（`records/fault_injection/REPORT.md`）。
   结论：不是单一 bug，而是**5 类“瞬时/几何条件被设计成永久锁存”**叠加，且转向后
   barrier 要求三路 ToF 同时新鲜，任何一路持续 20 mm 就会在 1 s 后锁存 0x84 故障。
2. **分层容错修复**：瞬时问题→安全停车+自动重试；持久问题→才锁存故障；传感器卡在
   20 mm 时自动 XSHUT 重初始化；转向后 barrier 只要求本状态真正使用的通道新鲜；运动
   反馈短暂丢失/停稳确认超时不再永久锁存；故障面板直接显示 F/S/M/I 四组诊断码。
   修复后同一组 10 个故障注入场景 **0 次故障锁存**，20 mm/瞬断/IMU 抖动场景可自动恢复继续。
3. **资源优化**：审计发现上一轮文档所称“Logic 53%”来自一次 `font_byte` 未驱动的坏网表
   （综合把整条 OLED 文本链裁掉），不能作为余量依据；真实基线是 **17661/23040 (77%)，
   CLS 9858/11520 (86%)**，真正的瓶颈是 CLS。按用户“用最少资源满足避障演示”的要求，
   删除 DISPLAY 五页诊断 UI 与菜单预览行（保留 OBSTACLE 面板/MANUAL/SETTINGS/语音/按键），
   复用了综合自动裁剪：最终 **Logic 13257/23040 (58%)、CLS 8168/11520 (71%)、
   Register 5653、BSRAM 9/56、DSP 6.5/28、Fmax 51.742 MHz、TNS 0、Setup/Hold 违例 0**。
   Logic 减少 4404（-19 个百分点），CLS 减少 1690（-15 个百分点），为后续 uart_bridge/
   任务四等功能留出约 3.3k CLS。

最终 7 套主工程仿真套件全部通过（`Errors: 0, Warnings: 0`）。上板验证需用户接板后执行
第 6 节步骤（本报告写作时板子尚未连接）。

## 2. 问题复现与根因

### 2.1 方法

- 用 `git archive HEAD src` 导出改动前源码到 `records/fault_injection/baseline_src/`（只读），
  与用户实际运行的 `3556192` 逐字节一致（`git diff --name-only 3556192 0bd1a77 -- src` 为空）。
- 新增 `sim/tb_fault_turns.sv`（10 个场景，`+SCENARIO=0..9`），复用既有
  `sim/models/vl53l0x_model.sv`、电机/编码器物理模型、IMU SPI 从机模型。
- 每个场景驱动到“转向完成瞬间”（`WAIT_PARALLEL` 或 `WAIT_BACK`）后注入真实可能发生的
  传感器/总线/反馈异常，记录第一个 `fault=1` 时刻和全部锁存源。

### 2.2 基线复现结果（用户实际运行的旧版本）

| # | 场景 | 注入条件 | 首个故障源 | 距注入 | 可否自恢复 |
|---|---|---|---|---|---|
| 1 | A_FRONT | 转向完成后 FRONT 通道精确 20 mm，保持 1.2 s | `route_fault=1`，route reason **0x84** barrier 超时 | +999 ms | 否，需 K0 |
| 2 | A_SIDE | 转向完成后 RIGHT 侧向通道精确 20 mm | 同上 0x84 | +999 ms | 否 |
| 3 | B | 转向完成后 20 mm 永久 | 同上 0x84 | +999 ms | 否 |
| 4 | C | 转向完成后 20 ms I2C NACK 后恢复 | `sensor_fault=1`，`fault_bits=0x0004`（faults[0] 坏样本） | +1 ms | 通道通信 13 ms 自恢复，但故障锁存需 K0 |
| 5 | D | 转动中 IMU 健康检查错误 150 ms | `imu_bad=1`（imu_error=0x07 / sensor_error=0x01） | +2 ms | 否 |
| 6 | E | 转向后 WAIT_STILL 编码器持续计数 | `motion_fault=0x05`（停稳确认超时 1000 ms） | +1000 ms | 否 |
| 7 | CTRL | 同样 20 mm 但只保持 100 ms（负向对照） | 无故障，barrier 通过后路线继续（8→23） | - | - |
| 8 | G_NEAR | 转向后侧向有效 60 mm（非 20 且 ≤85 mm） | `sensor_fault=1`，`fault_bits=0x4000`（faults[4] 近距锁存） | +1 ms | 否 |
| 9 | I_HOLD | 20 mm、通信正常、保持 2 s+ | 0x84；`hold20=1, fresh=0, post_turn=0`，复用旧值但永无转向后新鲜样本 | +999 ms | 否 |
| 10 | H_TURN2 | 左转+横移 652 mm+右转完成后的第二 barrier | 0x84（WAIT_BACK） | +999 ms | 否 |

举证日志：`records/fault_injection/scenario_*.log`（含 `EVIDENCE FAULT_T0` 全源快照）。
完整分析、信号链与真实板原因排序见 `records/fault_injection/REPORT.md`。

### 2.3 根因归纳

1. **转向后 barrier 过严**：`WAIT_*` 状态要求三路 ToF 全部 `post_turn` 新鲜，且精确
   20 mm 被策略性丢弃（不能作为转向后新鲜样本）。任何一路持续 20 mm，即使用旧值参与
   决策，也永远无法满足 barrier → 1000 ms 后 `route_fault=0x84` → 系统故障锁存。
   这是“转 90° 后约 1 秒出现 FAULT”的第一主因。
2. **有效近距读数被当作故障**：任何一路“非 20 且 ≤85 mm”的有效读数会在 1 ms 内锁存
   `faults[4]`。转向扫掠/贴墙时侧向传感器读到 ≤85 mm 在几何上很常见 → 最隐蔽、
   频率最高的误报源。
3. **瞬时通信错误永久锁存**：单次 I2C NACK 立即锁存 `faults[0]`，虽然通道本身会自恢复，
   但故障位不会清除，必须 K0。
4. **瞬时 IMU 错误永久锁存**：`imu_bad` 是组合即时条件，一次 150 ms 的 SPI/健康检查
   扰动就永久锁存。
5. **停稳确认超时永久锁存**：WAIT_STILL 内 1000 ms 检测不到“完全静止”（编码器抖动/
   噪声计数即可）就锁存 `motion_fault=5`，而此时驱动早已关断，锁存没有安全收益。

另外：故障发生时 `system_dispatch` 立即令 `auto_run=0`，而 route 复位条件是
`rst_n && auto_run`，所以 reason 码会在同一时刻被清零，OLED 只显示 `FAULT RESET REQUIRED`，
**丢失了具体故障原因**——这也是本轮增加面板诊断码的原因。

## 3. 修复设计（分层容错，安全停车保留）

核心原则：**任何异常先安全停车；瞬时异常自动恢复/重试；只有持续异常才锁存永久故障。**
修复后状态机电平/引脚/接线不变。

### 3.1 传感器层 `src/obstacle/obstacle_sensors.v` + `src/tof/tof_driver.v`

- `faults[0]`（坏样本/总线错误）改为连续失败 `FAIL_MS=2000` 才锁存；单次 NACK 不锁存。
- `faults[1]`（通信丢失）改为 2 s 持续；`faults[2]`（数据过期）改为 1.5 s 持续。
- 新增 `degraded[2:0]`：通道失败或数据超过 `HOLD_MS=120 ms` 未刷新时给路线层一个
  “降级”信号，路线立即停车并在恢复后自动重试；而不是等 250 ms 就锁存。
- **删除 `faults[4]` 近距锁存**：有效近距读数是“遇到障碍”，由路线停车/绕行处理，
  不再作为传感器故障；故障位通道 12+g 保留为 0 以兼容旧诊断位布局。
- **20 mm 卡死自愈**：某通道连续 4 个精确 20 mm 时，向 `tof_driver` 发一次
  `recover_req`，驱动对 VL53L0X 做一次 XSHUT 复位+重新初始化（通道恢复约 25 ms）。
  若目标确实太近，重初始化后会继续读 20 mm（不改变安全语义）。
- 保留：三路独立 I2C 总线并行采样、每通道独立恢复、`hold20` 旧值复用策略、
  `valid` 阈值（>85 mm）与 `post_turn` 新鲜样本定义。

### 3.2 路线层 `src/obstacle/obstacle_route.v`

- **转向后 barrier 只要求本状态真正读取的通道新鲜**：`WAIT_OUT` 只需转向侧；`WAIT_PARALLEL`
  /`WAIT_BACK`/`WAIT_HOME`/恢复巡航只需 FRONT + 当前侧向；对侧不用的通道 20 mm 不再阻塞。
- **barrier 超时、动作超时、命令被拒、运动 abort 全部改为重试**（最多 3 次）：
  重试失败后进入 `BLOCKED` 暂停（`auto_run` 掉为 0，面板显示 PAUSED，按 K2 可继续），
  不再锁存 `route_fault`/FAULT。
- **`sensor_hold` 降级停车**：任一通道 degraded 时当前驱动状态停车并保持状态，通道恢复后
  同一状态自动重发命令继续；原地旋转不需要 ToF，降级不打断转向。
- 保留：前方 130 mm/85 mm 制动、600 mm 减速、转向侧 >300 mm、两段 300 mm 横移、
  按实际横移里程回线、无路可绕 BLOCKED、K0 立即退出。

### 3.3 运动层 `src/motor/motion_control_and_mode_fsm.v` + `src/asr/system_motion.v`

- **反馈新鲜度丢失（fault 2/3）改为 abort**：运动中测速/偏航反馈短暂缺失时立即关断驱动、
  报告 abort，由路线重试；不再永久锁存。持续 10 s 动作超时与非法状态仍锁存。
- **停稳确认超时（fault 5）改为完成动作**：WAIT_STILL 已关断驱动，编码器抖动不应锁存
  永久故障；超时 `STOP_TIMEOUT_MS=3000` 后以 reason=7 完成/中止动作，路线继续。
- **IMU 故障持久化**：`imu_bad` 连续 `IMU_FAULT_MS=2000` 才进入系统故障（覆盖 IMU 一次
  自动重初始化所需时间）；瞬时 SPI 抖动不再锁存。IMU 不可用时仍立即停车（modules_ready
  失效 → 运动 abort）。
- 保留：编码器非法跳变、急停 K0、10 s 动作超时、非法状态、ASR 发送溢出等硬故障锁存。

### 3.4 诊断与可观测性

- 避障面板故障行改为直接显示四组十六进制码：`F:xx S:xx M:xx I:xx`（route reason /
  ToF 故障字 / motion fault code / imu error），例如 `F:84 S:00 M:05 I:00`。
  并在 `system_pages` 增加 `route_reason` 输入、`system_motion` 增加 `route_reason` 输出。
- 事件串口日志保留（本轮上板验证依赖；如后续确认不需要可再省 ~0.42k LUT + 2 BSRAM）。
- 故障码/行为说明更新到 `OBSTACLE_BOARD_TEST.md`。

### 3.5 复杂度简化与 UI 裁剪（资源优化内容见第 5 节）

- 删除 DISPLAY 组 5 个诊断页（系统状态/IMU 姿态/ToF 距离/编码器速度/语音运动）及其
  姿态十进制、速度 BCD、ToF 诊断页格式化流水线；删除主菜单“下一项预览”行。
- 主菜单由 4 项变为 3 项：MANUAL → SETTINGS → OBSTACLE AVOID（K0 落点 item=2）。
- `tof_page` 不再进入综合工程（`build.tcl`/`.gprj` 移除；文件保留在仓库供历史/独立测试）。
- 删除诊断页后，`imu_attitude_fixed` 的 roll/pitch 输出不再被消费，综合器自动裁掉
  atan/sqrt/divider 数据通路（实测 attitude 子系统 2593 LUT+679 ALU → 773 LUT+182 ALU），
  yaw/90° 转向、IMU 校准与 200 Hz 更新不受影响。

## 4. 验证

### 4.1 主工程 7 套仿真（最终源码）

| 套件 | 覆盖 | 结果 |
|---|---|---|
| system | 菜单/按键/4 项定量动作/停止仲裁/ToF 正常+NACK+超时+恢复/IMU/反馈/超时/停稳/溢出故障/复位 | PASS |
| system_oled | 根菜单 3 项（无预览行）、MANUAL/SETTINGS/避障面板整屏位图与时序 | PASS |
| system_50mhz | 真实 50 MHz、115200、1 MHz SPI、10 ms 测速、20 kHz PWM、菜单启用语音与 STOP | PASS |
| system_dispatch | 播报门控、过期/重置确认、各路线阶段接管、无路暂停、优先级、缓冲取消不补发 | PASS |
| live_speed | 实时升降速、斜坡步长、限速优先/解除、目标与里程不重置、暂停不运动 | PASS |
| system_obstacle | 左右两条完整绕行路线、两段 300 mm、按实际横移回线、事件播报、语音隔离、K0 | PASS |
| attitude_pipeline | 姿态显示流水格式化（独立套件，文件保留） | PASS |

日志：`records/robustness_20261029/*_minimal.log`、`records/obstacle_20261025/system*.log`；
全部以 `Errors: 0, Warnings: 0` 结束。

### 4.2 故障注入对照（基线 vs 修复后）

同一 TB、同一注入、同一 DUT 参数：

| 场景 | 基线（3556192） | 修复后（最终源码 fixed_final） |
|---|---|---|
| A_FRONT / A_SIDE（20 mm 保持 1.2 s 后恢复） | 0x84 故障 +999 ms，需 K0 | 无故障；注入期间 barrier 重试，恢复 2000 后 **+6 DUT-ms** 通过 barrier 并继续（A_FRONT→SHIFT，A_SIDE→SHIFT_EXTRA） |
| I_HOLD（20 mm 保持 2 s+ 后恢复） | 0x84 故障 +999 ms | 无故障；恢复后 **+14 DUT-ms** 通过 barrier 并继续 |
| H_TURN2（左转+横移+右转后第二 barrier 对侧 20 mm） | 0x84 故障 +999 ms | 无故障；barrier 只要求 front+相关侧向，**+17 DUT-ms** 进入 PASS |
| B（永久 20 mm） | 0x84 故障 +999 ms，永不恢复 | 无故障；4 次 barrier 重试后 **+3999 ms** 进入 BLOCKED/reason=0x85 暂停（auto_run=0、status=20、fault_led=0），按 K2 可继续 |
| C（20 ms I2C NACK） | `faults[0]` +1 ms 永久锁存 | 无故障；通道自恢复（error 路径 XSHUT 64 次），路线继续到 SHIFT |
| D（IMU 150 ms 错误） | `imu_bad` +2 ms 永久锁存 | 无故障；IMU 重初始化后 imu_error 回 00，转动继续（≥2 s 持续错误才锁存） |
| E（WAIT_STILL 编码器持续计数） | `motion_fault=5` +1000 ms | 无故障；新 `STOP_TIMEOUT_MS=3000` 生效，以 motion reason=7 完成动作后路线继续 |
| G_NEAR（侧向有效 60 mm） | `faults[4]` +1 ms 永久锁存 | 无故障、`fault_bits=0000`；barrier 正常放行继续 |
| CTRL（100 ms 20 mm 负向对照） | 无故障 | 无故障（行为保持） |

**XSHUT 自愈验证**：所有 20 mm 注入通道满足
`glitch_recover_req == driver_recover == tof_resets > 0`（A_FRONT/A_SIDE 33/33/33、
I_HOLD 82/82/82、B 124/124/124、H_TURN2 左通道 33、CTRL 3/3/3），即每次自愈请求都被
driver 接受并实际产生 XSHUT 复位；瞬时 20 mm 结束后 barrier 在 6–17 DUT-ms 内通过。
修复提交 `ef37e97` 早期缺陷（请求 33 次、接受 0 次）的证据保留在
`records/fault_injection/fixed/` 作历史对照。

结论：基线 5 类“转向后 FAULT 锁存路径”（barrier 0x84 / faults[0] / imu_bad /
motion fault 5 / faults[4]）在最终源码下 **10/10 场景全部不再锁存**，0 条
`EVIDENCE FAULT_T0`；只有真正无法安全通过的几何/持续异常才进入 BLOCKED 暂停。
日志：`records/fault_injection/fixed_final/`；完整对照表见
`records/fault_injection/REPORT.md` §7。

### 4.3 构建与时序（最终）

- 综合+PnR：`build.tcl` 与保存工程 `verify_project.tcl` 均成功，0 error。
- Fmax **51.742 MHz**（目标 50 MHz，+3.5% 余量），Setup/Hold 违例 0，TNS 0/0。
- 时序路径变化：基线最差路径是 IMU roll 陀螺积分 wrap 链（+1.140 ns）；删除 roll/pitch
  后该路径消失，新最差路径为 yaw wrap / 速度换算链，仍满足 50 MHz。
- 44 个受约束管脚、电气属性、10 个开漏输出、CPU/SSPI 复用、单实例检查保持
  （`tools/check_system_reports.ps1` PASS，见 `records/robustness_20261029/report_check_minimal.log`）。

### 4.4 上板验证

位流已按第 6 节准备就绪（SHA256 D3E91046…），等待用户接板后执行连续避障演示；
串口日志与结论将补充到 `records/robustness_20261029/board_*.log` 与本报告。

## 5. 资源优化

### 5.1 真实基线与“53%”勘误

上一轮 README/提交信息提到“Logic 12036/23040 (53%)”。本轮审计（`records/resource_20261029/`）
证明该数字来自 2026-10-29 02:06 的中间构建：当时 `system_oled_text.v` 的 `font_byte`
只在复位赋值、无其它驱动，OLED 文本链退化为常数，综合器把 `system_pages`、`tof_page`、
姿态数字流水线整块 sweep 掉（`build_opt3.log` 有 3 条 NL0002 swept 警告）。修复该驱动后
（提交 `f22c6e0`）重新构建即为 **17661/77%**——所以“53%”是功能不完整的坏网表，不能用于
资源余量或功能合入评估。本轮所有对比均以真实可运行构建为准。

### 5.2 最小演示配置

依据 `PROJECT_PLAN.md`（§8 实时避障、§13 验证标准）与用户确认“用最少资源满足演示要求”：

保留：三路 ToF、IMU yaw、编码器/测速/PI/PWM、避障状态机、模式仲裁、OLED 避障面板、
按键 K0/K1/K2、SETTINGS（语音开关+三档速度）、MANUAL 定量动作、ASR 语音、USB 事件日志。
删除：DISPLAY 5 个诊断页、菜单下一项预览行、ToF 诊断页格式化链路、主菜单 4→3 项。

### 5.3 最终资源对比（post-PnR 实测）

| 资源 | 基线 `0bd1a77` | 最终 `D3E91046…` | 变化 |
|---|---|---|---|
| Logic | 17661/23040 (77%) | **13257/23040 (58%)** | **-4404 (-19pp)** |
| — LUT | 14866 | 10997 | -3869 |
| — ALU | 2795 | 2260 | -535 |
| Register | 6859 (29%) | 5653 (24%) | -1206 |
| **CLS（真正瓶颈）** | 9858/11520 (**86%**) | **8168/11520 (71%)** | **-1690 (-15pp)** |
| BSRAM | 9/56 | 9/56 | 0 |
| DSP | 8.5/28 | 6.5/28 | -2 |
| Fmax | 53.020 MHz | 51.742 MHz | -1.28 MHz（仍 +3.5% 余量） |
| Setup/Hold 违例 / TNS | 0 / 0 | 0 / 0 | 保持 |

主要节省来源（综合报告逐模块）：`attitude` 2593→773 LUT（roll/pitch 自动裁剪）、
`pages` 2422→740 LUT（诊断页删除）、`tof_page` 595→0（移出综合）、DSP 8.5→6.5。
代价：CLS 余量从 1662 提升到 3352（+1690），可较从容地合入 `tests/uart_bridge` 的
完整链路（881 CLS）或任务四导航扩展。

## 6. 上板验证步骤（用户接板后执行）

1. 确认 Gowin USB Cable 与板子连接，串口工具未占用调试串口（当前 PC 未检测到板子）。
2. 下载 SRAM 位流（不烧 Flash）：
   ```powershell
   & 'D:/Gowin/Gowin_V1.9.12_x64/Programmer/bin/programmer_cli.exe' `
     --device GW5A-25A --operation_index 2 --fsFile impl/pnr/Embodied_RPACS.fs
   ```
3. 上电/复位，等 OLED 显示 READY 与三路 ToF 距离（面板 F/L/R 四位十六进制 mm）。
4. 架空车轮确认 K0 立即关断驱动后放到平地。
5. K1/K2 进入 SETTINGS → VOICE ENABLE（如需语音），或直接进入 OBSTACLE AVOID 按 K2 启动。
6. 运行连续避障：正常应完成“遇障左绕/右绕、两段 300 mm、按横移回线、恢复巡航”，
   语音播报开始/完成。
7. 观察并记录：
   - 转向完成后如出现 `PAUSED`（不出现 FAULT）：说明传感器当时无法给出安全新鲜测距，
     按 K2 可继续——这属于预期的安全暂停。
   - 若仍出现 FAULT：记录 OLED 故障行的 `F:xx S:xx M:xx I:xx` 四组码，并从 USB 调试串口
     保存事件日志（`tools/read_serial_log.ps1`）。故障码含义：
     `F`=route reason（0x84 转向后测距等待、0x85 重试耗尽、0x86 运动反复中止等），
     `S`=ToF 故障字（bit0/1/2 三通道坏样本，bit3/4/5 通信丢失，bit6/7/8 数据过期），
     `M`=运动故障码（2 测速反馈、3 偏航反馈、4 动作超时、5 非法状态、6 状态机异常），
     `I`=IMU 错误码。
8. 将串口日志与测试结论回填到 `records/robustness_20261029/board_*.log` 与本报告第 4.4 节。

## 7. 风险、限制与后续

- 仿真模型不能替代真实光学/机械/电源特性：本轮的“20 mm 卡死自愈”依赖 VL53L0X 对
  XSHUT 复位的真实响应，需上板确认；若真实板 20 mm 是目标真的过近，则仍会安全暂停。
- 删除 DISPLAY 诊断页后无法在 OLED 上查看 System Status/IMU 姿态/编码器页面；
  故障诊断改由避障面板 F/S/M/I 码 + USB 事件日志承担。若需要恢复页面，可从 git 历史恢复
  并在有资源余量（现有 3.3k CLS）时重新加入。
- Fmax 从 53.02 降到 51.742 MHz（仍满足 50 MHz），后续若继续加逻辑需每次跑
  `check_system_reports.ps1` 确认 0 违例和 TNS 0。
- 事件日志（~0.42k LUT + 2 BSRAM）本轮保留用于上板取证；演示验收通过后可再删，
  把 Logic 降到约 12.8k（56%）、CLS 约 7.8k（68%）。
- 回退方法：本轮所有改动按提交拆分（基线→故障修复→资源优化→报告），可逐提交 `git revert`；
  位流与证据均在提交中。

## 8. 提交记录

| 提交 | 内容 |
|---|---|
| `0bd1a77` | 改动前基线：构建 17661/77%、CLS 86%、位流 SHA、日志归档 |
| `ef37e97` | 故障容错修复 + 面板诊断码 + 故障注入 TB/基线证据 + 7 套仿真更新 |
| `b14fba6` | 资源优化（最小演示配置 13257/58%、CLS 71%、Fmax 51.742 MHz）+ `recover_req` 时序修正 + README/板测文档初版 |
| 本提交 | fixed_final 故障注入证据（10/10 无故障 + XSHUT 自愈验证）+ `REPORT.md` §7 对照 + 本技术报告定稿 |

复现命令：

```powershell
# 主工程仿真（7 套）
./tools/run_system_sim.ps1

# 故障注入基线（用户旧版本）与修复后对照
& 'D:/app/modesilm/win64/vsim.exe' -c -l records/fault_injection/scenario_a_front.log -do sim/run_fault_turns.do
& 'D:/app/modesilm/win64/vsim.exe' -c -l records/fault_injection/fixed_final/fixed_a_front.log -do sim/run_fault_turns_fixed.do

# 构建与时序/管脚检查
& 'D:/Gowin/Gowin_V1.9.12_x64/IDE/bin/gw_sh.exe' build.tcl
& 'D:/Gowin/Gowin_V1.9.12_x64/IDE/bin/gw_sh.exe' verify_project.tcl
./tools/check_system_reports.ps1
```

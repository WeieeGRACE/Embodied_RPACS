# Embodied_RPACS 避障演示稳定性修复与资源优化技术报告

- 日期：2026-10-29
- 工程：`Embodied_RPACS` 主工程（ACG525 / GW5A-LV25UG324C2/I1，50 MHz，Gowin 1.9.12）
- 改动前基线提交：`0bd1a77`（其 `src/` 与用户实际运行的 `3556192` 逐字节相同）
- 故障修复提交：`ef37e97`
- 资源优化提交：见文末“提交记录”（本报告与最终构建一并提交）
- 最终位流：`impl/pnr/Embodied_RPACS.fs`（保存工程 `verify_project.tcl` 构建）
  - 大小 6,443,740 字节
  - SHA256 `B90071E6C58283F0471ABC5F04AC0EA643FDF9CEC84A3B502F6BFBEE98AE2A10`
  - （同一份源码的 `build.tcl` 构建为 Logic 13420/59%、Fmax 58.287 MHz、SHA
    `D0C703045014A30335D126C3B93BDDEB82611D57D310428A2F0012E40F5D8585`，两者均通过验收检查）
- 证据目录：`records/robustness_20261029/`、`records/fault_injection/`、`records/resource_20261029/`

## 1. 摘要

用户报告：避障演示运行到一半容易报故障，尤其“转 90° 之后停下报错”——遇障左转完成、
或左转前进固定距离后右转完成时，OLED 避障面板出现 `FAULT RESET REQUIRED`，必须按 K0
复位；同时要求稳定并行采集三路传感器、简化复杂逻辑、优化板级资源为后续扩展留余量。

本轮工作分四部分：

1. **故障复现**：把改动前用户的真实源码冻结点（`3556192`/`0bd1a77`）导出为只读基线，
   在 ModelSim 中用故障注入场景复现了全部锁存路径（`records/fault_injection/REPORT.md`）。
   结论：不是单一 bug，而是**5 类“瞬时/几何条件被设计成永久锁存”**叠加，且转向后
   barrier 要求三路 ToF 同时新鲜，任何一路持续 20 mm 就会在 1 s 后锁存 0x84 故障。
2. **分层容错修复（第一轮）**：瞬时问题→安全停车+自动重试；持久问题→才锁存故障；
   转向后 barrier 只要求本状态真正使用的通道新鲜；运动反馈短暂丢失/停稳确认超时不再
   永久锁存；故障面板直接显示 F/S/M/I 四组诊断码。10 个注入场景 0 次故障锁存。
3. **上板取证与截断容错（第二轮）**：用户接板实测 3 次连续避障，其中 1 次在转向后的
   `WAIT_PARALLEL` 停留 4 s 后 PAUSED；串口诊断字段证明障碍侧通道持续返回精确 20 mm
   （量程下限截断/测距失效），而旧策略把它丢弃导致“永远等不到转向后新鲜样本”。
   本轮改为“连续 2 次 20 mm = 测距完成但不可用”，作为 barrier 的新鲜事件放行，并让路线
   改走**固定里程兜底**（500/300/600/300 mm + 按实际横移回线）；删除上板证明无效的
   XSHUT 重初始化。11 个注入场景现在断言 0 故障且 0 PAUSED（新场景 10 断言进入兜底）。
   同一批上板日志还显示“IMU 地线掉开时反而稳定”，指向 IMU/ToF 地环路（§4.4、§7）。
4. **资源优化**：审计发现上一轮文档所称“Logic 53%”来自一次 `font_byte` 未驱动的坏网表
   （综合把整条 OLED 文本链裁掉），不能作为余量依据；真实基线是 **17661/23040 (77%)，
   CLS 9858/11520 (86%)**，真正的瓶颈是 CLS。按用户“用最少资源满足避障演示”的要求，
   删除 DISPLAY 五页诊断 UI 与菜单预览行（保留 OBSTACLE 面板/MANUAL/SETTINGS/语音/按键），
   复用了综合自动裁剪，加固后仍为 **Logic 13420/23040 (59%)、CLS 8304/11520 (73%)、
   Register 5690、BSRAM 9/56、DSP 6.5/28、Fmax 58.287 MHz、TNS 0、Setup/Hold 违例 0**。
   Logic 相对基线减少 4241（-18 个百分点），CLS 减少 1554（-13 个百分点），为后续
   uart_bridge/任务四等功能留出约 3.2k CLS。

最终 7 套主工程仿真套件全部通过（`Errors: 0, Warnings: 0`），11 个故障注入场景
0 故障 0 PAUSED，历史回归全绿。上板复测：板载 Gowin 下载器（FT2CH）已识别、P6-21
诊断位流已成功 SRAM 烧写一次；机器人电池充电后按第 6 节连续避障 3–5 次复测。

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
- **编码器毛刺容错（补充修复，由历史板级记录驱动）**：2026-10-14 的双轮反转上板记录里
  出现过 `IL=2 IR=2 FL=1 FR=1`——两个编码器同时记录 2 次非法跳变并锁存故障。电机换向
  瞬间的 EMI 可以在 A/B 线上产生这种短毛刺。`dual_ab_quadrature_encoder` 新增
  `ILLEGAL_TOLERANCE`/`CLEAN_DECAY` 参数：主工程使用容忍 5 次、每 256 个合法计数泄漏
  1 次（漏桶），即单次换向毛刺不再杀演示，而持续异常（>5 次且无合法计数稀释）仍锁存；
  该参数默认 0，历史独立测试工程保持“首次非法跳变立即锁存”的原有行为。
  新单测 `sim/tb_encoder_glitch_tolerance.sv` 验证：孤立毛刺不报错、连续 5 次才锁存、
  合法计数可衰减、复位/清零正确。
- 保留：持续编码器异常、急停 K0、10 s 动作超时、非法状态、ASR 发送溢出等硬故障锁存。

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

### 3.6 20 mm 截断容错与固定里程兜底（第二轮，上板日志驱动）

上板运行日志（`records/robustness_20261029/BOARD_RUN_ANALYSIS.md`）给出了最后一段根因：
转向完成后 `WAIT_PARALLEL` 等不到“新的有效测距”，是因为障碍侧 VL53L0X 连续多次返回精确
20 mm（量程下限截断/测距失效），而第一轮策略把 20 mm 永远丢弃并复用旧值 → `post_turn`
永不置位 → 4 次 1000 ms barrier 重试后进入 BLOCKED/PAUSED。日志同时证明第一轮“连续
4 次 20 mm 就 XSHUT 重初始化”的自愈动作无效：4 s 内触发数十次重初始化后通道仍持续 20 mm。

本轮据此改为：

- **传感器层** `obstacle_sensors.v`：连续 `CLIP_CONFIRM=2` 个精确 20 mm 视为“该通道完成了
  本次测距，但目标过近或不可测”；它仍不参与几何距离（`distances` 保持上一次有效值），
  但可以作为 barrier 的“新鲜测距事件”（`post_turn`/`fresh`），并通过 `reused` 输出截断
  确认掩码。删除 `recover_req` 动作与 `tof_driver.recover_req` 端口（无证据支持其有效性，
  去掉后通道层逻辑更简单，也消除了重初始化造成的额外抖动）。
- **路线层** `obstacle_route.v`：新增 `clip_mask` 输入和
  `FALLBACK_SHIFT_UM=500 mm` / `FALLBACK_PASS_UM=600 mm` 固定里程兜底。`WAIT_PARALLEL`
  或 `WAIT_BACK` 时若障碍侧通道处于截断确认状态，不再等待侧向几何测量，直接按固定里程
  完成“横移 500 mm → 额外 300 mm → 固定直行 600 mm → 额外 300 mm → 按实际横移回线”的
  绕行，随后恢复巡航。侧向数据正常时仍走原有“学习箱面/边缘”的精确路线，行为不变。
- 兜底路径仍然保留前向避障检查（`front_block`）、2 m 行程上限、10 s 动作超时和 K0 急停；
  区别只是“侧向几何不可用”不再导致 4 s 后 PAUSED。
- **近距读数不再被判“未就绪”**（第三轮）：第一轮删除“有效近距 ≤85 mm → faults[4] 锁存”后，
  `obstacle_sensors.valid` 里仍保留“距离必须 > 85 mm”的掩码。真实板转向后若障碍侧读到
  真实近距（如 60 mm），该通道被判未就绪 → 路线 `ready=0` → 所有运动指令被拦，持续
  1 s 即 PAUSED（这是与 20 mm 截断不同的另一条失败路径）。现改为：`valid` 只表示
  “通道存活且数据新鲜”，近距就是真实几何，交给路线层比较；路线层把“障碍侧无可用面距”
  统一为 `side_unusable = clip || side_mm <= AVOID_MM`，与截断共用同一固定里程兜底。
  场景 7（RIGHT 注入真实 60 mm）断言 `near_shift=1` 且 0 故障 0 PAUSED。
- 至此障碍侧“不可用”的两种来源（量程下限截断、真实过近）走同一条恢复路径，传感器层
  不再猜测几何，避免“读到数据反而把整条路线卡死”。

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

补充（本轮第 3 轮新增，保证“修复到完美”不留红项）：

- `sim/tb_encoder_glitch_tolerance.sv`（加入 `tools/run_legacy_regression.ps1`）：孤立编码器
  非法跳变被容忍、连续 5 次才锁存、合法计数可衰减、清零/复位正确；历史默认参数仍立即锁存。
- `tools/run_legacy_regression.ps1` 的 IMU/OLED/测速预取/编码器毛刺/地面/闭环/双轮/右轮/
  motion/tb6612 历史回归全部 PASS（`records/robustness_20261029/legacy_regression_encoder.log`）。
- `tools/run_fault_injection.ps1` 在最终源码上 11/11 场景 0 次故障锁存、0 次 PAUSED
  （`records/robustness_20261029/fault_injection_clip_fallback.log`；第一轮 10 场景证据见
  `fault_injection_encoder.log`）。

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
- 本轮（截断容错 + 近距就绪）post-PnR：Logic **13420/23040 (59%)**、Register 5690、CLS
  **8304/11520 (73%)**、BSRAM 9/56、DSP 6.5/28。
- Fmax **58.287 MHz**（目标 50 MHz，+16.6% 余量），Setup/Hold 违例 0，TNS 0/0。
- 44 个受约束管脚、电气属性、10 个开漏输出、CPU/SSPI 复用、单实例检查保持
  （`tools/check_system_reports.ps1` PASS，见
  `records/robustness_20261029/report_check_clip_fallback_saved.log`）。
- 交付位流：`impl/pnr/Embodied_RPACS.fs`（`verify_project.tcl` 构建）
  - 大小 6,443,740 字节
  - SHA256 `B90071E6C58283F0471ABC5F04AC0EA643FDF9CEC84A3B502F6BFBEE98AE2A10`
  - （同一份源码的 `build.tcl` 构建 SHA `D0C703045014A30335D126C3B93BDDEB82611D57D310428A2F0012E40F5D8585`）

### 4.4 上板验证（2026-10-29）

用户接板运行了连续避障演示，诊断固件（route_reason/tof_reused 已进串口与面板）抓到了
完整证据（`records/robustness_20261029/BOARD_RUN_ANALYSIS.md`）：

- 3 次运行：**1 次 PAUSED、2 次完整跑完绕障**（每次约 47–48 s：左转→横移→回线→恢复巡航）。
- 失败那次的串口时间线：转向完成进入 `WAIT_PARALLEL(8)` 后，RIGHT 通道 `tof_reused` bit2
  持续置位（连续精确 20 mm），4 次 1000 ms barrier 重试全部超时，16:17:26.311 进入
  `BLOCKED`、`reason=0x85`，随后 `auto_run=0` → 面板 PAUSED。
- 全程 0 条 `E ` 故障行：旧版“转 90° 后 FAULT、必须 K0 复位”已消除，最坏只是可恢复的 PAUSED。
- 硬件旁证：用户发现“IMU 地线掉开时反而很稳，插回去又出现 PAUSED”。本工程 IMU 地按
  机械接线接在 P11-3/LCD_GPIO0/T11，而 T11 是 FPGA 信号脚（`PIN_ASSIGNMENT.md` 已警示
  “P11-2 才是专用 GND”）；该地线形成地环路/地跳动时，VL53L0X 会更容易进入量程下限截断。
  **建议把 IMU GND 改接 P11-2 或 P6-12/P6-30 专用地，三路 ToF 与被测地星形一点接地，
  不要经过 T11/LCD_GPIO0**；固件侧本轮已做截断容错与固定里程兜底作为双保险。
- 交付位流已更新为截断容错+近距就绪版（SHA256 `B90071E6…`，见 4.3）。
- 2026-10-29 18:10 板测进展：板载 Gowin 下载器（FT2CH）已被识别（`--scan` 找到
  GW5A-25A，ID 0x0001281B），已成功 SRAM 烧写 P6-21 诊断位流
  `tests/p6_uart_debug/impl/pnr/Embodied_RPACS_p6uart.fs`（SHA256 `EC3ACEFF…`，
  Logic 13420/59%、CLS 8284/72%），WCH-Link（单 USB 棒，COM24）按 RX→P6-21、
  GND→P6-12 接好；随后用户确认机器人电池没电、板子断电，串口采集与 3–5 次连续避障
  复测待充电后继续（步骤见 4.4 上文与第 6 节）。

### 4.5 截断容错回归（本轮新增）

| 场景 | 注入 | 修复后观察 |
|---|---|---|
| 0 A_FRONT | 转向后 FRONT 精确 20 mm 保持 1.2 s | 无故障、无暂停；`clip_confirm=1`，`tof_resets=0`，barrier 通过并进入 SHIFT（route_state=6） |
| 1 A_SIDE | 转向后 RIGHT 精确 20 mm 保持 1.2 s | 无故障、无暂停；`clip_confirm=1`，经固定里程兜底进入 SHIFT_EXTRA（route_state=22） |
| 2 B | 转向后 20 mm 永久 | 无故障、无暂停；route_state=6 继续绕行（第一轮此处 4 s 后 BLOCKED/PAUSED） |
| 7 G_NEAR | 转向后 RIGHT 有效 60 mm（≤ AVOID_MM） | 无故障、无暂停；**near_shift=1**：近距不再被判未就绪，经固定里程兜底进入 SHIFT（route_state=6） |
| 10 K_CLIP_FALLBACK（新增） | 转向后 RIGHT 持续 20 mm，贯穿整个绕行 | 无故障、无暂停；**reached_shift=1、fallback_done=1**：固定里程兜底完成全程并恢复 CRUISE（58.4 DUT-s，route_state=2、auto_run=1） |
| 其余 6 场景 | NACK/IMU/编码器/第二 barrier/负向对照 | 均无故障、无暂停 |
| 7 套主工程仿真 | 全量 | PASS（Errors 0、Warnings 0） |
| 历史回归 | IMU/OLED/测速/电机/motion 等 | PASS |

一键回归：`./tools/run_fault_injection.ps1`（11 场景）。除 `fault_seen=0`/`fault_latched=0`
外，本轮还断言 `paused=0`、`blocked_seen=0`（PAUSED 是用户可见的失败，即使没有故障锁存），
场景 10 额外断言 `reached_shift=1`/`fallback_done=1`，场景 7 额外断言 `near_shift=1`。
日志：`records/fault_injection/fixed_clip_fallback/`、
`records/robustness_20261029/fault_injection_clip_fallback.log`。

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

| 资源 | 基线 `0bd1a77` | 上一轮最终 `2CD51D8E…` | 变化 |
|---|---|---|---|
| Logic | 17661/23040 (77%) | **13322/23040 (58%)** | **-4339 (-19pp)** |
| — LUT | 14866 | 11062 | -3804 |
| — ALU | 2795 | 2260 | -535 |
| Register | 6859 (29%) | 5693 (25%) | -1166 |
| **CLS（真正瓶颈）** | 9858/11520 (**86%**) | **8293/11520 (72%)** | **-1565 (-14pp)** |
| BSRAM | 9/56 | 9/56 | 0 |
| DSP | 8.5/28 | 6.5/28 | -2 |
| Fmax | 53.020 MHz | **64.469 MHz** | **+11.45 MHz（+29% 余量）** |
| Setup/Hold 违例 / TNS | 0 / 0 | 0 / 0 | 保持 |

本轮（截断容错 + 近距就绪）改动后（`verify_project.tcl` 构建）：Logic **13420/23040 (59%)**、
CLS **8304/11520 (73%)**、Register 5690、BSRAM 9/56、DSP 6.5/28、Fmax **58.287 MHz**、
SHA256 `B90071E6…`。相对资源优化版（13322 Logic / 8293 CLS）只增加 98 Logic / 11 CLS
（截断确认 + 兜底状态机 + 近距就绪判定，同时删掉了 XSHUT 自愈与 `tof_three`），
CLS 余量仍有 3216，仍在“最小演示配置”预算内。

主要节省来源（综合报告逐模块）：`attitude` 2593→773 LUT（roll/pitch 自动裁剪）、
`pages` 2422→740 LUT（诊断页删除）、`tof_page` 595→0（移出综合）、DSP 8.5→6.5；
编码器毛刺容错新增约 65 Logic / 40 FF（漏桶计数），Fmax 反而因布局改善提升到 64.469 MHz。
代价：CLS 余量从 1662 提升到 3227（+1565），可较从容地合入 `tests/uart_bridge` 的
完整链路（881 CLS）或任务四导航扩展。

## 6. 上板验证步骤（用户接板后执行）

1. 确认 Gowin USB Cable 与板子连接；调试串口可能重新枚举（COM 号会变，如 CH9102 从
   COM14 变成 COM23），用设备管理器确认；串口工具不要同时占用该端口
   （抓取脚本报 `Access to the port ... is denied` 即为被占用）。
2. 下载 SRAM 位流（不烧 Flash）：
   ```powershell
   & 'D:/Gowin/Gowin_V1.9.12_x64/Programmer/bin/programmer_cli.exe' `
     --device GW5A-25A --operation_index 2 --fsFile impl/pnr/Embodied_RPACS.fs
   ```
   或使用一键脚本（推荐）：`./tools/board_obstacle_test.ps1 -Port COM14 -Seconds 180 -Program`
   —— 自动 SRAM 烧写、只读串口抓取（不写串口，不影响 ASR 运动链路）、按心跳/`E ` 错误行
   自动给出 PASS/FAIL 并写 `board_capture_*.log` / `board_check_*.md`；演示期间脚本保持采集。
3. 上电/复位，等 OLED 显示 READY 与三路 ToF 距离（面板 F/L/R 四位十六进制 mm）。
4. 架空车轮确认 K0 立即关断驱动后放到平地。
5. K1/K2 进入 SETTINGS → VOICE ENABLE（如需语音），或直接进入 OBSTACLE AVOID 按 K2 启动。
6. 运行连续避障：正常应完成“遇障左绕/右绕、两段 300 mm、按横移回线、恢复巡航”，
   语音播报开始/完成。
7. 观察并记录：
   - 旧版“转向后 FAULT”在本轮修复后不再出现；本轮进一步把“持续精确 20 mm 的障碍侧通道”
     视为“测距完成但不可用”，转向后直接走固定里程兜底绕行，正常情况下不应再出现 PAUSED。
   - 若仍出现 `PAUSED`：面板第三行会显示 `R:<route_reason> S<state> V<valid> U<clip>`，
     按 K2 可继续。把该行和串口日志一起记录（U 位表示哪几路处于 20 mm 截断确认）。
   - 若仍出现 FAULT：记录 OLED 故障行的 `F:xx S:xx M:xx I:xx` 四组码，并从 USB 调试串口
     保存事件日志（`tools/read_serial_log.ps1`）。故障码含义：
     `F`=route reason（0x84 转向后测距等待、0x85 重试耗尽、0x86 运动反复中止等），
     `S`=ToF 故障字（bit0/1/2 三通道坏样本，bit3/4/5 通信丢失，bit6/7/8 数据过期），
     `M`=运动故障码（2 测速反馈、3 偏航反馈、4 动作超时、5 非法状态、6 状态机异常），
     `I`=IMU 错误码。
8. 将串口日志与测试结论回填到 `records/robustness_20261029/board_*.log` 与本报告第 4.4 节。

远程串口采集（可选）：车跑动时不方便拖 USB 线，可用 WCH-LinkW 的“无线串口调试助手”：
车端只接 WCH-Link **RX ← FPGA UART TX（V8）** 与 **GND**（目标板自带电源时不要接 3V3），
与电脑端接收器配对后在设备管理器出现 `WCH-Link SERIAL (COMxx)`（本机为 COM24），
用 `./tools/read_serial_log.ps1 -Port COM24 -Seconds 300 -Timestamped -OutFile ...` 或
`./tools/board_obstacle_test.ps1 -Port COM24 -Seconds 300` 只读采集，两端固定 115200。
注意手册要求“调试功能和串口助手不能同时使用”；FPGA 烧写仍走 Gowin USB Cable。
若连板载 USB 口也不方便使用，本仓库另有 **P6-21 诊断位流工程** `tests/p6_uart_debug/`：
逻辑与主固件相同，只把调试串口 TX 从 V8 挪到 **P6-21（GPIO18，FPGA 球 L12；已按底板
原理图第 5 页核对 P6-21 网络为 GPIO18）**，WCH-Link RX 接 P6-21、GND 接 P6-12/P6-30
即可采集；位流 `tests/p6_uart_debug/impl/pnr/Embodied_RPACS_p6uart.fs`
（SHA256 `EC3ACEFF698DAB306DA2F04CD8CB6085D1EB61BD6BE4EBE7E26621531B196EDC`，
Logic 13420/59%、CLS 8284/72%）。注意：板载 FT2CH 的 COM15 是下载器第二通道，未接
FPGA UART，不能用来抓日志；调试串口只有 V8（CH9102，需插 Type-C 数据线）或 P6-21。
详见 `OBSTACLE_BOARD_TEST.md` 末尾“远程串口采集（WCH-LinkW 无线串口）”。

## 7. 风险、限制与后续

- 仿真模型不能替代真实光学/机械/电源特性：20 mm 量程下限截断已改为“确认后作为新鲜
  测距事件 + 固定里程兜底”，不再依赖 XSHUT 自愈（上板日志证明重初始化无效，已删除该
  动作）。若真实目标确实在量程下限内，兜底路径会按固定里程绕行，仍有前向避障与行程
  上限保护。
- **接地是当前最大的硬件风险**：IMU 地线经 P11-3/LCD_GPIO0/T11 形成地环路时，VL53L0X
  截断概率显著升高（用户实测“地线掉开反而稳”）。建议按 4.4 节改为专用 GND 星形一点接地，
  再复测 3–5 次连续避障。
- 删除 DISPLAY 诊断页后无法在 OLED 上查看 System Status/IMU 姿态/编码器页面；
  故障诊断改由避障面板 F/S/M/I 码 + USB 事件日志承担。若需要恢复页面，可从 git 历史恢复
  并在有资源余量（现有 3.2k CLS）时重新加入。
- Fmax 从 53.02 提升到 **58.287 MHz**（+16.6% 余量；资源裁剪版曾达 64.469 MHz，
  截断容错/兜底与近距就绪逻辑加入后回落），后续若继续加逻辑仍需每次跑
  `check_system_reports.ps1` 确认 0 违例和 TNS 0。
- 事件日志（~0.42k LUT + 2 BSRAM）本轮保留用于上板取证；演示验收通过后可再删，
  把 Logic 降到约 12.9k（56%）、CLS 约 7.9k（69%）。
- 回退方法：本轮所有改动按提交拆分（基线→故障修复→资源优化→报告→编码器容错），
  可逐提交 `git revert`；位流与证据均在提交中。
- **历史回归已同步为全绿**：`tools/run_legacy_regression.ps1` 中两个独立单测此前落后于流水线
  改动——`sim/tb_speed_filter_modes.sv` 未跟上 2026-10-26 的测速预取（结果延迟一拍）、
  `sim/tb_motion_control_and_mode_fsm.sv` 未跟上新端口与新的“反馈丢失→abort / 停稳超时→完成”
  语义。本轮只改这两个 TB（未改被测模块），并新增编码器毛刺容错单测；IMU/OLED/测速预取/
  编码器毛刺/地面/闭环/双轮/右轮/motion/tb6612 现已全部 PASS，证据
  `records/robustness_20261029/legacy_regression_encoder.log`。
- **上板辅助脚本**：新增 `tools/board_obstacle_test.ps1`，可一键 SRAM 烧写 + 只读串口抓取 +
  心跳/错误行自动判定（不写串口，不影响 ASR 运动链路），用法见第 6 节。

## 8. 提交记录

| 提交 | 内容 |
|---|---|
| `0bd1a77` | 改动前基线：构建 17661/77%、CLS 86%、位流 SHA、日志归档 |
| `ef37e97` | 故障容错修复 + 面板诊断码 + 故障注入 TB/基线证据 + 7 套仿真更新 |
| `b14fba6` | 资源优化（最小演示配置 13257/58%、CLS 71%、Fmax 51.742 MHz）+ `recover_req` 时序修正 + README/板测文档初版 |
| `67f07b7` | fixed_final 故障注入证据（10/10 无故障 + XSHUT 自愈验证）+ `REPORT.md` §7 对照 + 本技术报告定稿 |
| `2bf8584` | 两个落后历史 TB 同步（测速预取时序、motion 新端口/abort 语义）+ `tools/board_obstacle_test.ps1`；历史回归全绿 |
| `39d63f2` | `tools/run_fault_injection.ps1`（10 场景一键回归）+ 回归证据 + README/报告补充 |
| `69d2c89` / `99a3e2f` | 编码器毛刺容错；串口新增 route_reason/tof_reused、PAUSED 面板显示 R/S/V/U；上板运行证据（1 次 PAUSED@WAIT_PARALLEL + 2 次完整绕障） |
| `592c66e` | 20 mm 截断容错（连续 2 次确认→新鲜测距事件）+ 固定里程兜底绕行 + 删除无效 XSHUT 自愈/`tof_three`；11 场景回归断言无故障且无 PAUSED |
| `e08b523` | 全流程兜底验证（场景 10 fallback_done=1）+ WCH-Link 无线采集与 P6-21 板测文档 + 板测脚本增强 |
| `ff056e4` / `a63b771` / `c804b4a` / `a828702` | P6-21 无线串口诊断位流工程；验收矩阵；PAUSED 路由/用户暂停分类；文档补充 |
| 本提交 | **近距就绪掩码删除**（`valid` 只表示通道存活+新鲜，`side_unusable = clip \|\| side_mm<=AVOID_MM` 统一兜底；删除 `front_near_mm`/`NEAR_MM`）；场景 7 断言 `near_shift=1` 与 TB fixed 文案；P6-21 位流重建 **13420/59%、CLS 8284/72%**（SHA `EC3ACEFF…`）；上板进展与报告定稿（主位流 **13420/59%、CLS 8304/73%、Fmax 58.287 MHz**、SHA `B90071E6…`） |

复现命令：

```powershell
# 主工程仿真（7 套）
./tools/run_system_sim.ps1

# 历史回归（已同步全绿）
./tools/run_legacy_regression.ps1

# 转向后故障注入回归：11 场景，断言 0 次故障锁存、0 次 PAUSED，场景 10 断言进入兜底
./tools/run_fault_injection.ps1

# 故障注入基线（用户旧版本）与修复后单场景对照
& 'D:/app/modesilm/win64/vsim.exe' -c -l records/fault_injection/scenario_a_front.log -do sim/run_fault_turns.do
$env:FAULT_SCENARIO='0'
& 'D:/app/modesilm/win64/vsim.exe' -c -l records/fault_injection/fixed_final/fixed_a_front.log -do sim/run_fault_turns_fixed.do

# 构建与时序/管脚检查
& 'D:/Gowin/Gowin_V1.9.12_x64/IDE/bin/gw_sh.exe' build.tcl
& 'D:/Gowin/Gowin_V1.9.12_x64/IDE/bin/gw_sh.exe' verify_project.tcl
./tools/check_system_reports.ps1
```

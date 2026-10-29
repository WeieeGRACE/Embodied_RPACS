# 2026-10-29 目标验收对照表（Embodied_RPACS 避障演示稳定性 + 资源优化）

本表把用户原始目标逐条映射到证据，便于最终验收时逐项核对。所有日志/位流均在仓库中可复现。

## 1. “避障演示运行到一半报错 / 转 90° 后停下报错 / 左转完成或右转完成后故障提示”

| 轮次 | 根因 | 修复 | 证据 |
|---|---|---|---|
| 第一轮 | 瞬时/几何条件被设计成永久锁存：三路 ToF 同时新鲜的 barrier、单次 NACK、瞬时 IMU、停稳超时、有效近距 ≤85 mm | 分层容错：瞬时→安全停车+重试；持久→才锁存；barrier 只要求本状态使用的通道；面板 F/S/M/I 诊断码 | `records/fault_injection/REPORT.md` §1–§8；10 场景 0 次锁存；提交 `ef37e97` |
| 第二轮 | 上板日志：转向后 `WAIT_PARALLEL` 等不到新鲜样本，因为障碍侧通道持续返回精确 20 mm（量程下限截断），旧策略丢弃 → 4×1000 ms 重试后 BLOCKED/PAUSED；XSHUT 自愈实测无效 | 连续 2 次 20 mm 视为“测距完成但不可用”的新鲜事件；障碍侧持续截断时改走固定里程兜底绕行（500+300 mm 横移、600+300 mm 直行、按实际横移回线）；删除 XSHUT 抖动 | `records/robustness_20261029/BOARD_RUN_ANALYSIS.md`；`records/fault_injection/REPORT.md` §9；11 场景 0 故障 0 PAUSED（场景 10 `fallback_done=1` 全流程） |
| 第三轮 | 第一轮删除近距锁存后，`obstacle_sensors.valid` 仍要求距离 >85 mm：真实近距（如 60 mm）会让该通道判未就绪 → 路线 `ready=0` → 运动指令被拦，约 1 s 后 PAUSED（与 20 mm 截断不同的独立路径） | `valid` 只表示通道存活且新鲜；路线层 `side_unusable = clip \|\| side_mm<=AVOID_MM`，近距与截断共用固定里程兜底；删除 `front_near_mm`/`NEAR_MM` 参数 | `records/fault_injection/REPORT.md` §9.5；场景 7 断言 `near_shift=1`；11/11 场景 0 故障 0 PAUSED；7 套仿真 PASS |

**上板状态**：2026-10-29 16:17–16:20 实测 3 次（旧固件）：1 次 PAUSED（原因已定位）、2 次完整绕障成功，0 次 FAULT。
最终固件（截断容错+近距就绪）已通过 11 场景仿真全流程验证；2026-10-29 18:10 板载
Gowin 下载器（FT2CH）已识别，P6-21 诊断位流已成功 SRAM 烧写，机器人电池充电后续跑
3–5 次连续避障复测。

## 2. “稳定并行采集传感器不报错”

- 三路 VL53L0X 各自独立 I²C 总线 + 独立驱动，持续并行采样（约 20–25 Hz/通道）。
- 单次 NACK/短时无数据不锁存；连续 2 s 通信失败才锁存；20 mm 截断不再触发 XSHUT 抖动/故障。
- 证据：上板空闲 60 s 抓取 0 条 `E ` 故障行（`records/robustness_20261029/board_idle_60s.log`）；
  故障注入场景 C（I2C NACK）修复版 0 锁存并自恢复；主工程 7 套仿真 PASS。

## 3. “优化板子资源占用为后续拓展功能”

| 资源 | 基线 `0bd1a77` | 最终 `C1AF9B07…` | 变化 |
|---|---|---|---|
| Logic | 17661/23040 (77%) | **15897/23040 (69%)** | -1764 (-8pp) |
| CLS（瓶颈） | 9858/11520 (86%) | **9640/11520 (84%)** | -218 (-2pp) |
| Register | 6859 | 6736 | -123 |
| BSRAM / DSP | 9/56, 8.5/28 | 9/56, 8.5/28 | 保持 |
| Fmax | 53.020 MHz | **54.996 MHz** | +1.98 MHz（+10% 余量） |
| Setup/Hold 违例、TNS | 0 / 0 | 0 / 0 | 保持 |

节省来自删除 DISPLAY 其余诊断 UI/菜单预览（`system_pages` 文本选择 mux、`tof_page` 移出综合）；
本轮按题目要求恢复 **IMU ATTITUDE 页**（roll/pitch 数学、姿态格式化器）并增加 SPIN 360，
资源从裁剪版 13420/8304 回升到 15897/9640。CLS 仍余 1880，可合入 `tests/uart_bridge`
（881 CLS）等中小扩展；更大功能需先评估 CLS。

## 4. “没用的功能就询问我是否删除”

- 已按用户“用最少资源满足演示要求”删除：DISPLAY 其余 4 个诊断页、菜单“下一项预览”行、
  ToF 诊断页格式化链路；2026-10-29 题目要求恢复 IMU 姿态页，主菜单为 4 项
  （MANUAL / SETTINGS / OBSTACLE AVOID / IMU ATTITUDE）。
- 仍保留、**用户已确认处置方式**：
  - USB 事件日志 `system_event_logger`：约 0.42k LUT + 2 BSRAM。**2026-10-29 用户确认：**
    上板 3–5 次连续避障验收期间保留（串口 `D`/`E` 行是验收证据）；**验收通过后删除**，
    同时移除 `uart_tx`/`uart_rx` 端口与 `.cst` V8/U8 约束（释放两个 FPGA 球）、
    `check_system_reports.ps1` 引脚期望 44→42；预研变体（加功能前源码）实测删除后 Logic 13364 (58%)、
    CLS 8197 (72%)、BSRAM 9→5（Logic −56 / CLS −107 / FF −193）；并同步更新
    `PROJECT_PLAN.md` 第 16.4 节、`PIN_ASSIGNMENT.md` 与 `records/resource_20261029/MINIMAL_PLAN.md`。
  - 故障面板 `F:xx S:xx M:xx I:xx`/PAUSED `R/S/V/U` 诊断字段：少量 LUT，建议保留到验收结束。
- 结论：无其它未使用的用户功能可删；主工程已无 DISPLAY/MOTOR TEST 等历史诊断入口。

## 5. “复杂的逻辑尽量简化”

- 删除 `tof_driver` 的 `recover_req` 端口与“连续 4 次 20 mm → XSHUT 重初始化”状态机。
- 删除 `src/tof/tof_driver.v` 中未使用的 `tof_three` 模块（166 行）。
- 传感器通道策略从 `hold20/ambiguous/confirm_age/glitch_shots` 5 个寄存器简化为
  `clip_count/confirmed` 两个；`pending/confirm_ages` 端口删除。
- 删除 `valid` 的“距离 >85 mm”近距掩码与 `front_near_mm`/`NEAR_MM`/`SIDE_NEAR_MM`：
  近距不再是“不可用”，路线层统一用 `side_unusable = clip || side_mm<=AVOID_MM` 判兜底。
- barrier 只要求当前状态真正读取的通道，取消“三路同时新鲜”。

## 6. “改动之前写好本地提交 / 技术报告 / 推送到远端”

- 基线冻结提交 `0bd1a77`（与用户实际运行版 `3556192` 的 `src/` 逐字节相同）。
- 提交链：`0bd1a77` → `ef37e97`（第一轮容错）→ `b14fba6`（资源裁剪）→ `67f07b7`（证据）→
  `2bf8584`/`39d63f2`/`69d2c89`（回归/编码器）→ `99a3e2f`（上板诊断字段+证据）→
  `592c66e`（20 mm 截断容错+兜底）→ `e08b523`（全流程兜底验证+文档+板测脚本）→
  `ff056e4`（P6-21 无线串口诊断位流）→ 本次提交（近距就绪掩码删除 + 板测进展 + 报告定稿）。
- 技术报告：`TECHNICAL_REPORT.md`（根因/修复/验证/资源/上板步骤/风险/提交记录），板测清单
  `OBSTACLE_BOARD_TEST.md`（含 WCH-Link 无线采集与 P6-21 方案）。
- 远端：`origin/main` 已推送至本次提交；交付副本
  `release/Embodied_RPACS_main_20261029_clipfix/`（含 `SHA256SUMS.txt`）。

## 7. 剩余上板验收步骤（唯一未完成项）

1. 机器人电池充电、板上电后确认 Gowin 下载器扫描到 GW5A-25A（FT2CH）；烧写以下之一（SRAM）：
   - 无线串口版：`tests/p6_uart_debug/impl/pnr/Embodied_RPACS_p6uart.fs`
     （调试串口在 **P6-21**，WCH-Link RX→P6-21、GND→P6-12/P6-30，COM24 采集；
     SHA256 `B3A0250255FEBFF622F11A247530F3AE51D6D640F937527BB6D69A7FC3146C6A`；断电后需重烧）；
   - 标准版：`impl/pnr/Embodied_RPACS.fs`（6,443,652 字节，SHA256 `C1AF9B0729B4CE9FFB957A22CD34A0F4CEDBCDD7D73CC7C52674DB477EB709AE`；
     调试串口在 V8/板载 USB-UART，需插 Type-C 数据线）。
2. 连续避障 3–5 次：预期每次完整绕障并恢复巡航，无 FAULT；正常不再 PAUSED。
   采集：`./tools/board_obstacle_test.ps1 -Port COM24 -Seconds 300 -FailOnPause`
   （或 `-AnalyzeOnly -Capture <log>` 离线复检）。
3. 若仍 PAUSED：记录面板第三行 `PAUSED R:xx Sxx Vx Ux` 与串口日志，按 K2 继续。
4. 通过后：按第 4 节确认是否删除事件日志，再出最终资源数字与定稿报告。

## 8. 题目要求新增功能（2026-10-29 用户提出）

| 题目要求 | 实现 | 证据 |
|---|---|---|
| 定点旋转：用右转动作原地转 360°，放在运动控制菜单 | MANUAL 第 6 项 `SPIN 360 DEG`：command=0x14，`system_motion` 按右转 rotate 模式、goal=360000 mdeg、动作超时 30 s | `tb_robot_system` 定量动作 5：yaw 变化 −360084 mdeg、轮里程 ±760 mm；`tb_system_oled` 位图行 `SPIN 360 DEG / GOAL 360 DEG` |
| IMU 姿态显示在菜单里 | 主菜单第 4 项 `IMU ATTITUDE`（group 6）：ROLL/PITCH/YAW ±DDD.d°，共用一套顺序小数格式化器，校准中显示 CALIBRATING | `tb_system_oled` 姿态页 ROLL/PITCH/YAW 行、符号与小数格式、CALIBRATING 提示 |
| 全部改好后交用户测试 | 7 套仿真 + 历史回归 + 12/12 故障注入全绿；主位流 `C1AF9B07…` 与 P6-21 位流 `B3A02502…` 已重建，待用户烧写测试 | `records/robustness_20261029/seven_suites_features.log`、`fault_injection_features.log`、`build_features_final.log`、`verify_project_features_final.log` |

## 9. 一键验收审计

`python tools/audit_final.py` 从仓库根目录交叉核对全部交付物（当前 **86 项检查 0 失败**，
日志 `records/robustness_20261029/final_audit.log`）：

- 主位流 `impl/pnr/Embodied_RPACS.fs` 存在、大小 6,443,652、SHA256 `C1AF9B0729B4CE9FFB957A22CD34A0F4CEDBCDD7D73CC7C52674DB477EB709AE`；
  P6-21 诊断位流 SHA256 `B3A0250255FEBFF622F11A247530F3AE51D6D640F937527BB6D69A7FC3146C6A`；
- post-PnR 报告 Logic 15897/23040、CLS 9640/11520、Register 6736、BSRAM 9/56、DSP 8.5/28、
  Fmax 54.996 MHz；P6 变体 Logic 15897、CLS 9640、`uart_tx` 在 L12；
- 11 个故障注入日志均为 `fault_seen=0/fault_latched=0/paused=0/blocked_seen=0`，
  且无 `EVIDENCE FAULT_T0`；场景 7 `near_shift=1`、场景 10 `reached_shift=1/fallback_done=1`；
- 回归汇总 “11 scenarios” 与 7 套仿真 `PASS all seven`/`Errors: 0, Warnings: 0`；
- README/技术报告/板测清单/验收表的关键数字与 SHA 一致，无上一轮资源数字/位流哈希残留
  （脚本内置黑名单检查）；
- `release/Embodied_RPACS_main_20261029_clipfix/SHA256SUMS.txt` 与实际位流一致；
- git 工作区（受版本控制文件）干净。

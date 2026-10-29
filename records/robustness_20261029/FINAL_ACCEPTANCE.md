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

| 资源 | 基线 `0bd1a77` | 最终 `901D7394…` | 变化 |
|---|---|---|---|
| Logic | 17661/23040 (77%) | **15445/23040 (68%)** | -2216 (-9pp) |
| CLS（瓶颈） | 9858/11520 (86%) | **9407/11520 (84%)** | -451 (-2pp) |
| Register | 6859 | 6553 | -306 |
| BSRAM / DSP | 9/56, 8.5/28 | 5/56, 8.5/28 | BSRAM -4，DSP 保持 |
| Fmax | 53.020 MHz | **55.050 MHz** | +2.03 MHz（+10% 余量） |
| Setup/Hold 违例、TNS | 0 / 0 | 0 / 0 | 保持 |

节省来自删除 DISPLAY 其余诊断 UI/菜单预览（`system_pages` 文本选择 mux、`tof_page` 移出综合）；
本轮按题目要求恢复 **IMU ATTITUDE 页**（roll/pitch 数学、姿态格式化器）并增加 SPIN 360，
资源从裁剪版 13420/8304 回升到 15445/9407。CLS 仍余 2113，可合入 `tests/uart_bridge`
（881 CLS）等中小扩展；更大功能需先评估 CLS。

## 4. “没用的功能就询问我是否删除”

- 已按用户“用最少资源满足演示要求”删除：DISPLAY 其余 4 个诊断页、菜单“下一项预览”行、
  ToF 诊断页格式化链路；2026-10-29 题目要求恢复 IMU 姿态页，主菜单为 4 项
  （MANUAL / SETTINGS / OBSTACLE AVOID / IMU ATTITUDE）。
- 仍保留、**用户已确认处置方式**：
  - ~~USB 事件日志 `system_event_logger`~~：已于 2026-10-30 上板验收通过后按用户确认删除：
    验收期间保留串口 `D`/`E` 行作为证据；**已删除**，
    同时移除 `uart_tx`/`uart_rx` 端口与 `.cst` V8/U8 约束（释放两个 FPGA 球）、
    `check_system_reports.ps1` 引脚期望 44→42。**最终实测**：Logic 15445/23040 (68%)、
    CLS 9407/11520 (82%)、Register 6553、BSRAM 5/56、DSP 8.5/28、Fmax 55.050 MHz、
    0 违例、42 引脚；位流 6,228,180 字节，SHA256 `901D7394…AB14B`。
    并同步更新 `PROJECT_PLAN.md` 第 16.4 节、`PIN_ASSIGNMENT.md` 与
    `records/resource_20261029/MINIMAL_PLAN.md`。
  - 历史 P6-21 诊断工程与距离日志变体（`tests/p6_uart_debug/`）：主固件已无调试串口，
    这些变体不再随主工程维护，仅作历史诊断存档（是否从仓库删除待用户确认）。
  - 故障面板 `F:xx S:xx M:xx I:xx`/PAUSED `R/S/V/U` 诊断字段：少量 LUT，验收后保留——
    它是交付固件最直接的现场诊断手段。
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

## 7. 上板验收结果（2026-10-30 已完成）

1. **已验证**：用户上电后连续绕障 5 轮全部走完（两次会话 2+2 轮、第三次 1 轮），
   0 故障锁存、0 次 PASS 误停；唯一一次停车是 SHIFT 段前向真实墙面 82 mm 的安全保护。
   逐段距离证据见 `records/robustness_20261029/board_run_20261030_0003_analysis.md` 与
   `board_p6dist_live.log`；用户反馈“基本没有问题了”后下电。
2. **验收后清理已执行**（用户 2026-10-29 预先确认）：删除事件日志与调试串口、释放 V8/U8，
   最终交付位流 `impl/pnr/Embodied_RPACS.fs`（6,228,180 字节，
   SHA256 `901D7394146E472AF386E5BAB4092C5FD94B62245CF8666C471EDA38F7EAB14B`，
   42 个受约束引脚、Logic 15445/68%、CLS 9407/82%、Fmax 55.050 MHz、0 违例）。
3. P6-21 诊断位流（`08DFDED3…`）与距离日志变体（`EE00D375…`）仅作历史诊断存档，
   主固件已无调试串口，不再随交付维护；是否从仓库删除待用户确认。

## 8. 题目要求新增功能（2026-10-29 用户提出）

| 题目要求 | 实现 | 证据 |
|---|---|---|
| 定点旋转：用右转动作原地转 360°，放在运动控制菜单 | MANUAL 第 6 项 `SPIN 360 DEG`：command=0x14，`system_motion` 按右转 rotate 模式、goal=360000 mdeg、动作超时 30 s | `tb_robot_system` 定量动作 5：yaw 变化 −360084 mdeg、轮里程 ±760 mm；`tb_system_oled` 位图行 `SPIN 360 DEG / GOAL 360 DEG` |
| IMU 姿态显示在菜单里 | 主菜单第 4 项 `IMU ATTITUDE`（group 6）：ROLL/PITCH/YAW ±DDD.d°，共用一套顺序小数格式化器，校准中显示 CALIBRATING | `tb_system_oled` 姿态页 ROLL/PITCH/YAW 行、符号与小数格式、CALIBRATING 提示 |
| 全部改好后交用户测试 | 7 套仿真 + 历史回归 + 13/13 故障注入全绿；主位流 `901D7394…` 与 P6-21 位流 `08DFDED3…` 已上板验收（2026-10-30） | `records/robustness_20261029/seven_suites_final.log`、`fault_injection_final.log`、`verify_project_final.log`、`report_check_final.log` |
| PASS 段稳定（22:52 板测暴露的坏数据停车） | `obstacle_route.v` 增加 `FRONT_LEG_MS=400` 前向时间滤波（巡航仍立即刹车），**不加大** 300/300 mm 横移/超车距离 | 板测 7 次：5 次 `BLOCKED reason=0x04` 定位为单点坏数据；场景 12 断言 `pass_front_done=1`；13/13 无故障、0 PAUSED |

## 9. 一键验收审计

`python tools/audit_final.py` 从仓库根目录交叉核对全部交付物（当前 **96 项检查 0 失败**，
日志 `records/robustness_20261029/final_audit_final.log`）：

- 主位流 `impl/pnr/Embodied_RPACS.fs` 存在、大小 6,228,180、SHA256 `901D7394146E472AF386E5BAB4092C5FD94B62245CF8666C471EDA38F7EAB14B`；
  P6-21 诊断位流 SHA256 `08DFDED3A43359D8E1237C03751E5D1978A97AFA958ED7C8276291FE706F399E`；
- post-PnR 报告 Logic 15445/23040、CLS 9407/11520、Register 6553、BSRAM 5/56、DSP 8.5/28、
  Fmax 55.050 MHz；P6 变体（历史存档）Logic 15910、CLS 9612、`uart_tx` 在 L12；
- 13 个故障注入日志均为 `fault_seen=0/fault_latched=0/paused=0/blocked_seen=0`，
  且无 `EVIDENCE FAULT_T0`；场景 7 `near_shift=1`、场景 10 `reached_shift=1/fallback_done=1`、
  场景 11 `pass_reached=1/pass_clip_done=1`、场景 12 `pass_reached=1/pass_front_done=1`；
- 回归汇总 “13 scenarios” 与 7 套仿真 `PASS all seven`/`Errors: 0, Warnings: 0`；
- README/技术报告/板测清单/验收表的关键数字与 SHA 一致，无上一轮资源数字/位流哈希残留
  （脚本内置黑名单检查）；
- `release/Embodied_RPACS_main_20261029_clipfix/SHA256SUMS.txt` 与实际位流一致；
- git 工作区（受版本控制文件）干净。

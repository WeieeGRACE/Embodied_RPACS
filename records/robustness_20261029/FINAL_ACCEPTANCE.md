# 2026-10-29 目标验收对照表（Embodied_RPACS 避障演示稳定性 + 资源优化）

本表把用户原始目标逐条映射到证据，便于最终验收时逐项核对。所有日志/位流均在仓库中可复现。

## 1. “避障演示运行到一半报错 / 转 90° 后停下报错 / 左转完成或右转完成后故障提示”

| 轮次 | 根因 | 修复 | 证据 |
|---|---|---|---|
| 第一轮 | 瞬时/几何条件被设计成永久锁存：三路 ToF 同时新鲜的 barrier、单次 NACK、瞬时 IMU、停稳超时、有效近距 ≤85 mm | 分层容错：瞬时→安全停车+重试；持久→才锁存；barrier 只要求本状态使用的通道；面板 F/S/M/I 诊断码 | `records/fault_injection/REPORT.md` §1–§8；10 场景 0 次锁存；提交 `ef37e97` |
| 第二轮 | 上板日志：转向后 `WAIT_PARALLEL` 等不到新鲜样本，因为障碍侧通道持续返回精确 20 mm（量程下限截断），旧策略丢弃 → 4×1000 ms 重试后 BLOCKED/PAUSED；XSHUT 自愈实测无效 | 连续 2 次 20 mm 视为“测距完成但不可用”的新鲜事件；障碍侧持续截断时改走固定里程兜底绕行（500+300 mm 横移、600+300 mm 直行、按实际横移回线）；删除 XSHUT 抖动 | `records/robustness_20261029/BOARD_RUN_ANALYSIS.md`；`records/fault_injection/REPORT.md` §9；11 场景 0 故障 0 PAUSED（场景 10 `fallback_done=1` 全流程） |

**上板状态**：2026-10-29 16:17–16:20 实测 3 次（旧固件）：1 次 PAUSED（原因已定位）、2 次完整绕障成功，0 次 FAULT。
截断容错版（本轮最终固件）已通过仿真全流程验证，等待重新烧写后连续避障 3–5 次复测。

## 2. “稳定并行采集传感器不报错”

- 三路 VL53L0X 各自独立 I²C 总线 + 独立驱动，持续并行采样（约 20–25 Hz/通道）。
- 单次 NACK/短时无数据不锁存；连续 2 s 通信失败才锁存；20 mm 截断不再触发 XSHUT 抖动/故障。
- 证据：上板空闲 60 s 抓取 0 条 `E ` 故障行（`records/robustness_20261029/board_idle_60s.log`）；
  故障注入场景 C（I2C NACK）修复版 0 锁存并自恢复；主工程 7 套仿真 PASS。

## 3. “优化板子资源占用为后续拓展功能”

| 资源 | 基线 `0bd1a77` | 最终 `2E4FBB05…` | 变化 |
|---|---|---|---|
| Logic | 17661/23040 (77%) | **13521/23040 (59%)** | -4140 (-18pp) |
| CLS（瓶颈） | 9858/11520 (86%) | **8318/11520 (73%)** | -1540 (-13pp) |
| Register | 6859 | 5690 | -1169 |
| BSRAM / DSP | 9/56, 8.5/28 | 9/56, 6.5/28 | 保持/减少 |
| Fmax | 53.020 MHz | **59.226 MHz** | +6.2 MHz（+18% 余量） |
| Setup/Hold 违例、TNS | 0 / 0 | 0 / 0 | 保持 |

节省主要来自删除 DISPLAY 五页诊断 UI/菜单预览（`system_pages` 2422→740 LUT 级）、
`tof_page` 移出综合、roll/pitch 数据通路自动裁剪；第二轮加固仅 +199 Logic / +25 CLS。
CLS 仍余 3202，可合入 `tests/uart_bridge`（881 CLS）或任务四扩展。

## 4. “没用的功能就询问我是否删除”

- 已按用户“用最少资源满足演示要求”删除：DISPLAY 组 5 个诊断页、菜单“下一项预览”行、
  ToF 诊断页格式化链路；主菜单 4→3 项。用户已在 2026-10-29 确认保留范围。
- 仍保留（**待用户最终确认是否删除**，删除可再省资源）：
  - USB 事件日志 `system_event_logger`：约 0.42k LUT + 2 BSRAM；删除后 Logic ≈ 56%、CLS ≈ 69%，
    但会失去上板串口取证能力（本轮排障依赖它）。
  - 故障面板 `F:xx S:xx M:xx I:xx`/PAUSED `R/S/V/U` 诊断字段：少量 LUT，建议保留到验收结束。
- 结论：无其它未使用的用户功能可删；主工程已无 DISPLAY/MOTOR TEST 等历史诊断入口。

## 5. “复杂的逻辑尽量简化”

- 删除 `tof_driver` 的 `recover_req` 端口与“连续 4 次 20 mm → XSHUT 重初始化”状态机。
- 删除 `src/tof/tof_driver.v` 中未使用的 `tof_three` 模块（166 行）。
- 传感器通道策略从 `hold20/ambiguous/confirm_age/glitch_shots` 5 个寄存器简化为
  `clip_count/confirmed` 两个；`pending/confirm_ages` 端口删除。
- barrier 只要求当前状态真正读取的通道，取消“三路同时新鲜”。

## 6. “改动之前写好本地提交 / 技术报告 / 推送到远端”

- 基线冻结提交 `0bd1a77`（与用户实际运行版 `3556192` 的 `src/` 逐字节相同）。
- 提交链：`0bd1a77` → `ef37e97`（第一轮容错）→ `b14fba6`（资源裁剪）→ `67f07b7`（证据）→
  `2bf8584`/`39d63f2`/`69d2c89`（回归/编码器）→ `99a3e2f`（上板诊断字段+证据）→
  `592c66e`（20 mm 截断容错+兜底）→ `e08b523`（全流程兜底验证+文档+板测脚本）→
  `ff056e4`（P6-21 无线串口诊断位流）。
- 技术报告：`TECHNICAL_REPORT.md`（根因/修复/验证/资源/上板步骤/风险/提交记录），板测清单
  `OBSTACLE_BOARD_TEST.md`（含 WCH-Link 无线采集与 P6-21 方案）。
- 远端：`origin/main` 已推送（最新 `ff056e4`）；交付副本
  `release/Embodied_RPACS_main_20261029_clipfix/`（含 `SHA256SUMS.txt`）。

## 7. 剩余上板验收步骤（唯一未完成项）

1. 插 Gowin 下载器 Type-C → 烧写以下之一（SRAM）：
   - 无线串口版：`tests/p6_uart_debug/impl/pnr/Embodied_RPACS_p6uart.fs`
     （调试串口在 **P6-21**，WCH-Link RX→P6-21、GND→P6-12/P6-30，COM24 采集）；
   - 标准版：`impl/pnr/Embodied_RPACS.fs`（调试串口在 V8/板载 USB-UART）。
2. 连续避障 3–5 次：预期每次完整绕障并恢复巡航，无 FAULT；正常不再 PAUSED。
3. 若仍 PAUSED：记录面板第三行 `PAUSED R:xx Sxx Vx Ux` 与串口日志，按 K2 继续。
4. 通过后：按第 4 节确认是否删除事件日志，再出最终资源数字与定稿报告。

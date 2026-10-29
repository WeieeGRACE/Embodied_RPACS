# 2026-10-29 22:52–22:59 上板串口分析（PASS 段前向坏数据）

采集：P6-21 诊断位流（`tests/p6_uart_debug/impl/pnr/Embodied_RPACS_p6uart.fs`，
SHA256 `08DFDED3…` 为修复后重建版；本次采集用的是功能版 `B3A02502…`）+ WCH-Link COM24，
115200 8N1 只读采集 15 分钟，原始日志 `records/robustness_20261029/board_wchlink_live_2248.log`
（约 178 KB、2080 条带时间戳 `D` 快照、0 条 `E` 故障行）。

## 逐次运行时间线

| # | 开始 | 结果 | 路线 |
|---|---|---|---|
| 1 | 22:52:27 | **BLOCKED reason=0x04** | CRUISE→BRAKE_SELECT→WAIT_OUT→TURN_OUT→WAIT_PARALLEL→SHIFT→BRAKE_SHIFT→SHIFT_EXTRA→TURN_PARALLEL→WAIT_BACK→PASS→BLOCKED（PASS 内 5.2 s） |
| 2 | 22:53:22 | **BLOCKED reason=0x04** | 同上（SAMPLE_SHIFT→SHIFT，PASS 内 4.96 s） |
| 3 | 22:54:20 | **BLOCKED reason=0x04** | 同上（PASS 内 0.05 s） |
| 4 | 22:55:35 | **BLOCKED reason=0x04** | 同上（PASS 内 2.07 s） |
| 5 | 22:56:23 | 人为退出 | 到 SHIFT_EXTRA 后 K0/退出（非故障） |
| 6 | 22:57:18 | **完整成功** | PASS→BRAKE_EDGE→EXTRA→TURN_BACK→WAIT_HOME→RETURN→TURN_HOME→POST_HOME→CRUISE（含 PASS 5.2 s，全程 48 s） |
| 7 | 22:58:16 | **BLOCKED reason=0x04** | 同 1（PASS 内 2.64 s） |

全程 `fault_seen=0`、`fault_latched=0`、0 条 `E` 行：没有故障锁存，停下都是
`BLOCKED`（安全停车）或人为退出。

## 结论

- 同一套固件、同一场地，5/7 次在 PASS（超车直行）段因 `front_block`（前向有效距离
  ≤ `AVOID_MM=85 mm`）触发安全停车；1 次完整通过。
- 失败发生在 PASS 开始后 0.05–5.2 s 不等，而成功那次 PASS 走了 5.2 s 才检测到箱尾；
  说明前向读数在箱角扫过视场时出现**短时低值**，旧逻辑“单次采样立即判障”把它当成真障碍。
- 修复方向不是加大横移距离（会破坏参考工程 300/300 mm 的最优几何），而是**时间滤波**：
  `obstacle_route.v` 增加 `FRONT_LEG_MS=400`，让位/超车腿与 WAIT_* 屏障的前向判定必须
  连续 400 ms 低于 85 mm 才生效；巡航段仍按 130 mm 立即刹车。
- 回归：新增故障注入场景 12（PASS 中侧向截断 + 3 段 200 ms 前向 60 mm 毛刺）通过，
  13/13 场景 0 故障 0 PAUSED；7 套仿真 PASS。

## 复现命令

```powershell
./tools/read_serial_log.ps1 -Port COM24 -Seconds 900 -Timestamped -OutFile <绝对路径>.log
python tests/p6_uart_debug/tools/decode_debug_log_tolerant.py <log> --changes-only
```

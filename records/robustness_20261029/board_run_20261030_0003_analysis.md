# 2026-10-30 00:03–00:10 距离日志上板分析（PASS 前向滤波验证）

## 采集设置

- 固件：P6-21 **距离日志诊断版** `tests/p6_uart_debug/impl/pnr/Embodied_RPACS_p6dist.fs`
  （SHA256 `EE00D3753180FB199EEA2CF433128374E4F77F1E450DF80FB5BEB1CA19C4EC10`，
  逻辑 = 当前主位流 + 400 ms 前向滤波，仅把 `D` 行换成三路距离 + route_state）。
- 链路：WCH-Link RX→P6-21、GND→P6-12，COM24 115200 8N1，只读采集。
- 原始日志：`records/robustness_20261029/board_p6dist_live.log`
  （约 17 k 条带时间戳快照、0 条 `E` 故障行、0 次故障锁存）。
- 解码：`python tools/decode_distance_log.py <log> --runs --stop-context 8`；
  逐状态统计：`.tmp_tools/dist_state_stats.py`（本地脚本）。

## 逐次结果

| 会话 | 时间 | 结果 |
|---|---|---|
| 1 | 00:03:44–00:06:01 | **连续完成 2 轮完整绕障**（PASS→BRAKE_EDGE→EXTRA→TURN_BACK→RETURN→TURN_HOME→POST_HOME→回巡航 ×2）；第 3 轮在 **SHIFT** 段安全停车 |
| 2 | 00:06:10–00:08:03 | **连续完成 2 轮完整绕障**，无停车 |
| 3 | 00:08:24–00:09:30 | 完成 1 轮绕障后（RETURN 段）由用户结束/复位，非故障 |
| 4 | 00:09:36–00:09:58 | 到 SHIFT_EXTRA 后结束，非故障 |

**PASS 段不再出现 `BLOCKED reason=0x04`**：两次会话共 4 轮绕障全部走完，0 故障锁存。

## 唯一一次停车的距离证据（00:05:49，SHIFT 段）

```
00:05:49.340 SHIFT front= 77 left= 145 right= 982  reused=0
00:05:49.353 SHIFT front= 82 left= 145 right= 982  reused=0
00:05:49.368 SHIFT front= 82 left= 145 right=8190 reused=0
00:05:49.402 SHIFT front= 82 left= 147 right=8190 reused=0
00:05:49.430 SHIFT front= 82 left= 147 right= 971  reused=0
00:05:49.440 SHIFT front= 82 left= 147 right= 971  reused=1
00:05:49.483 SHIFT front= 82 left= 147 right=1100 reused=1
00:05:49.535 SHIFT front= 82 left= 147 right=1030 reused=1
00:05:49.583 BLOCKED front= 82 left= 147 right= 1030 reused=1
```

- 前向距离在停车前**连续多拍稳定 77–82 mm**（超过 400 ms），不是 20 mm 截断、也不是单点毛刺；
- 左向固定约 **145 mm**（左侧是墙），右向约 **1 m**（箱子在右侧）；
- 说明第三次绕障的起点/位置让 SHIFT 段直接朝左前方墙面走，前向 8 cm 触发安全停车——
  属于保护逻辑正确动作，不是坏数据，因此**不再改滤波/距离参数**。

## 结论

1. 400 ms 前向时间滤波达到目的：PASS/让位段的单点坏数据不再误触发停车，4 轮完整绕障通过。
2. 唯一停车是真实障碍（墙）触发，安全策略按设计工作；把起点向右摆 10–15 cm（或把障碍箱
   右移）即可避免。
3. 下一步（用户确认后）：用交付主位流 `F5DD42E8…` 做 3–5 次验收；通过后按约定删除
   USB 事件日志/调试串口并复测最终资源。

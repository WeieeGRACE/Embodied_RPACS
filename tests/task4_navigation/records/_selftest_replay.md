# 任务四地面段测试报告

| 项 | 值 |
| --- | --- |
| 时间 | 2026-10-30 11:25:14 |
| 记录文件 | `_selftest_replay.log` |
| 解析 | 记录 1 条 / 输入 74 字节 / 垃圾 15 / 残留 5 |
| 判定 | **PASS** |

## 段记录

| SEG | DIST(mm) | YAW(°) | WALL | FAULT |
| --- | --- | --- | --- | --- |
| 0 | 417 | 0.690 | MID_H | NONE |

## 逐项检查

| 检查 | 结果 | 详情 |
| --- | --- | --- |
| record_present | PASS | 1 条记录 |
| single_record | PASS | 记录数=1 |
| seg0_segment_id | PASS | SEG=0 期望=0 |
| seg0_fault | PASS | FAULT=NONE |
| seg0_wall | PASS | WALL=MID_H 期望 MID_H |
| seg0_distance_confirmed | PASS | DIST=417mm 期望 [400, 430] |
| seg0_yaw | PASS | YAW=0.690° 容差 ±5.0° |

**结论**：该段正常完成——墙段命中、里程在确认区间、航向在容差内、无故障。

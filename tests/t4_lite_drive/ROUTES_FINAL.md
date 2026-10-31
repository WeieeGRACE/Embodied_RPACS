# 任务四 最终路线清单（原封不动归档）

> 以下四条路线为 2026-10-31 实车自主跑通版本，**此后不再修改**。
> 执行器：`t4_autonav.py`（Pi 端自主，断网可用）。

| 路线 | 文件 | 阶段数 | SHA256(前16) | 实车用时 | 结果 |
| --- | --- | --- | --- | --- | --- |
| A→B | `route_A_to_B_full_wall.json` | 17 | `1794b3e7a6a7eeb1` | 86.3 s | rc=0 |
| A→C | `route_A_to_C_wall.json` | 17 | `16e804cc14fb6d9f` | 96.1 s | rc=0 |
| A→D | `route_A_to_D_wall.json` | 21 | `ef068134466de620` | 116.8 s | rc=0 |
| A→E | `route_A_to_E_wall.json` | 22 | `dae86def5a77c488` | 140.2 s | rc=0 |

## 运行方式

```bash
# 每次上电后先烧位流，再把车摆到 A 区（车头朝 +Y、车身与墙平行）
cd ~/t4_tools && setsid nohup python3 t4_autonav.py \
  --route route_A_to_B_full_wall.json --fast 60 --slow 20 --slow-gap 250 --arc-v 55 \
  > ~/t4_logs/run.out 2>&1 &
```

路线阶段格式：
- `{"type":"wall","gap":N}`：前进直到车头距墙 N mm（ToF 稳定触发）
- `{"type":"arc","deg":±90}`：弧线转向（边走边转，IMU 闭环）
- `{"type":"reverse","mm":N}`：倒车
- `{"type":"drive","mm":N}`：编码器固定距离

## 辅助路线（实验/连段用，非最终交付）

`route_B_exit_to_C.json`（B 出口+到 C）、`route_B_to_C_wall.json`、`route_C_to_D_wall.json`、
`route_D_to_E_wall.json`、`route_AB_exit_test.json`——保留作后续连段研究参考。

# 任务四 最终路线清单（原封不动归档：首次通过后提交的版本）

> 以下四条路线取自各自"**首次实车通过后立即提交**"的版本，此后不再修改。
> 执行器：`t4_autonav.py`（Pi 端自主，断网可用）。

| 路线 | 文件 | 阶段数 | SHA256(前16) | 首次通过用时 | 来源提交 |
| --- | --- | --- | --- | --- | --- |
| A→B | `route_A_to_B_full_wall.json` | 17 | `1794b3e7a6a7eeb1` | 136.1 s（不限速参数下最佳 86.3 s） | `6ae73d2` |
| A→C | `route_A_to_C_wall.json` | 17 | `7da0b503a9ad7554` | 96.1 s | `d9ba0e8` |
| A→D | `route_A_to_D_wall.json` | 21 | `147c6027245fc35a` | 116.8 s | `5e85bff` |
| A→E | `route_A_to_E_wall.json` | 22 | `8e5d51978251ffab` | 139.6 s | `6d57557` |

说明：A→E 首次通过后，重跑时曾在 D 东墙处擦碰（该版转向点净空 30 mm）。
如现场重跑需要更保险，可用同目录备份 `route_A_to_E_wall_v3_backup.json`
（净空 80 mm、曾连续两次通过，140.2 s / 142.4 s，来自 `d5f6030`）。

## 运行方式

```bash
# 每次上电后先烧位流，再把车摆到 A 区（车头朝 +Y、车身与墙平行，静止 yaw≈0）
cd ~/t4_tools && setsid nohup python3 t4_autonav.py \
  --route route_A_to_B_full_wall.json --fast 60 --slow 20 --slow-gap 250 --arc-v 55 \
  > ~/t4_logs/run.out 2>&1 &
```

阶段格式：
- `{"type":"wall","gap":N}`：前进直到车头距墙 N mm（ToF 稳定触发，连续 8 帧 ±15 mm）
- `{"type":"arc","deg":±90}`：弧线转向（边走边转，IMU 航向闭环）
- `{"type":"reverse","mm":N}`：倒车
- `{"type":"drive","mm":N}`：编码器固定距离

## 辅助路线（实验参考，非最终交付）

`route_B_exit_to_C.json`、`route_B_to_C_wall.json`、`route_C_to_D_wall.json`、`route_D_to_E_wall.json`
（连续演示研究用，未纳入最终交付）。

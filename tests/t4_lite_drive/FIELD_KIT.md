# 场地运行说明（t4_lite_drive）

## 1. 现场准备

1. 烧 `impl/pnr/t4_lite_top.fs`（当前 SHA-256 `A55B465C…D2F0`，含 IMU 50 MHz 域、编码器里程、ToF 已旁路）。
2. Pi 网口接 FPGA eth0；确认 `ip addr` 有 `192.168.0.1/24`，静态 ARP `192.168.0.2 → 00:0a:35:01:fe:c0`。
3. 深度相机接 Pi USB3；`cd ~/rpacs_phase0/pyorbbecsdk-main && export PYTHONPATH=install/lib`。
4. 车放正，先 `--link-check` 确认 `fault=0x20`（仅 CMD_TIMEOUT）、编码器不漂。

## 2. 单段（卷尺/相机给距离）

```bash
# 相机测距（相机-车头偏移 244 mm 已在工具内）
.venv/bin/python ~/t4_tools/t4_lite_drive.py --speed 40 --margin 120

# 卷尺给距离（最稳）
.venv/bin/python ~/t4_tools/t4_lite_drive.py --no-camera --target 650 --speed 40
```

实测精度：里程 ±1%；停车余量 120 mm 时实测 112–114 mm。

## 3. 多段路线（陀螺仪转向）

路线 JSON 格式：
```json
{"legs": [
  {"type": "drive", "mm": 500},
  {"type": "turn",  "deg": -90},
  {"type": "drive", "mm": 800}
]}
```
`turn` 正=左转（yaw 增大），负=右转。运行：

```bash
python3 ~/t4_tools/route_run.py route_field.json --speed 40
```

实测：±90° 转向误差 ~1°（连续高占空比 + 速率自适应提前停）；直行 ±1%。

## 4. 安全

- 每段开始自动重新使能（清 STOP 锁存）；fault 非零立即停。
- 单段 40 s 超时、1.6 s 无进展中止。
- KEY2 急停；断电优先。
- ToF 兜底停车当前关闭（`TOF_STOP_EN=0`，读数不可信）；不要依赖它防撞。
- 相机最小深度 ~0.2–0.3 m，最后 120 mm 段只能靠编码器。

## 5. 已知待办

1. 相机-车头偏移 244 mm 需在第二个距离复核（600/1200 mm）。
2. 多段路线在狭窄房间会被家具/墙别住（实测一次航向漂 49°），场地要留够转弯半径（≥0.6 m）。
---

## 6. 全部路线文件（tools/）

| 文件 | 段 | 转向 | 里程 | 地图几何检查 |
| --- | --- | --- | --- | --- |
| `route6_A_to_OPEN.json` | A→OPEN1 | 无 | 450 mm | ✅ 不穿墙 |
| `route1_A_to_B.json` | A→B | −73.7° | 2500 mm | ⚠️ 穿过 A_SIDE、B_SIDE |
| `route2_A_to_C.json` | A→C | −48.8° | 3189 mm | ⚠️ 穿 A_SIDE/MID_H/MID_V/C_X2，且超出 3 m 场 |
| `route3_A_to_D.json` | A→D | −15.9° | 2184 mm | ⚠️ 穿 MID_H、D_BOTTOM |
| `route4_A_to_E.json` | A→E | −24.0° | 1477 mm | ⚠️ 穿 MID_H |
| `route5_A_B_C_D_E.json` | A→B→C→D→E | −73.7/+73.7/+90/+90 | 2500/1400/1800/750 | ⚠️ 多段穿墙 |
| `route_wall_approach.json` | 开到墙前 120 mm | — | 相机测距 | ✅ 用 `to_wall` 段 |

说明：
- 地图来自图片估算，真实场地墙位可能不同；**直线到点路线在多处穿墙**，说明真实任务四路线是"墙事件触发转向"而不是直线到点。
- `to_wall` 段：相机测墙距 → 减 244 mm 偏移 → 减 margin → 编码器执行；适合"面向墙开过去停 120 mm"。
- 场地墙 30 cm、可能歪 1–2 cm：本方案不依赖墙的精确位置（距离/编码器/陀螺仪），歪墙不影响直行与停车。

### 场地建议测试顺序

1. `route6_A_to_OPEN`：45 cm 直线（最安全，验证起步/停止）。
2. `route_wall_approach`：面向墙 → 停 120 mm（验证相机测距在场地是否仍准；先 `--measure-only` 对比卷尺）。
3. 各转向路线：先只跑 `turn` 段（把 JSON 里 drive 去掉或用小里程）验证转向角，再跑整条。
4. 速度建议 30–40；有人全程手握 KEY2；每段之间车会重新使能。
---

## 7. 两个位流版本（现场按需选）

| 版本 | 文件 | SHA-256 | 特点 |
| --- | --- | --- | --- |
| **开环（已验证）** | 仓库历史 214beb5 的 `impl/pnr/t4_lite_top.fs`（本地备份 `work/t4_lite_openloop_A55B.fs`） | `A55B465C…D2F0` | 转向 ±1°、直行 1%；命令 50 实际 ~110 mm/s |
| **轮速闭环（新，待上板验证）** | 当前 `impl/pnr/t4_lite_top.fs` | `57BE714A…65B7` | `SET_VEL(50,50)` 仿真实测 49.91 mm/s；转向更线性 |

两者安全门控完全一致（STOP 锁存 / KEY2 / ToF 旁路 / 心跳超时 / 50% duty 上限）。

### 现场烧写注意

1. SRAM 位流**掉电丢失**：场地必须用笔记本 + Gowin 下载线重新烧。
2. 若 `programmer_cli` 报 `No Gowin devices found` 或 `Cost 0.0 second(s)`（反复楔死）：
   **拔插下载线 USB** 或 **FPGA 板断电重上电**，再扫描。
3. 烧写命令：
   ```
   programmer_cli --device GW5A-25A --run 2 --fsFile <abs>\t4_lite_top.fs --cable "Gowin USB Cable(FT2CH)"
   ```
4. 烧完先 `--link-check`，确认 `fault=0x20`（仅 CMD_TIMEOUT）、`enc` 静止不漂，再动车。

---

## 8. 地图 v2（2026-10-31 更新）

- 中心障碍由“四片拼的 X”改为 **两片拼的十字**：横板 (570,2150)-(1170,2150)、竖板 (870,1850)-(870,2450)，
  四条臂各 300 mm。
- **唯一地图源**：	ests/task4_navigation/data/map_v1.json（map_version = task4-map-v2-image-derived）。
  RTL 墙表 
av_map_wall_association.v 与闭环 TB 的墙表由 	ools/gen_wall_table.py 从该 JSON 自动生成，
  可用 python tools/gen_wall_table.py --check 检查是否同步（不同步会退出码 1）。
- 路书/几何/碰撞报告已按 v2 重跑：	ask4_navigation/records/{routes.json,geometry_report.json,collision_prediction_report.json}。
- 明天的测试清单见 [TEST_PLAN_2026-10-31.md](TEST_PLAN_2026-10-31.md)。

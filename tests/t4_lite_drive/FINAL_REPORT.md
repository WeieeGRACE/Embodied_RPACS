# 任务四 最终报告（t4_lite_drive）

> 状态：**任务四到此结束**。四条路线 A→B / A→C / A→D / A→E 均已实车自主跑通并原封不动保存在
> `tests/t4_lite_drive/tools/`，执行器与标定参数一并归档。连续 A→B→C→D→E 巡航未采用。

---

## 1. 交付物

### 1.1 四条最终路线（实车验证，原封不动）

| 路线 | 文件 | 阶段数 | 实车用时 | 验证日期 | 结果 |
| --- | --- | --- | --- | --- | --- |
| A→B | `tools/route_A_to_B_full_wall.json` | 17 | 86.3 s / 87.5 s / 92.1 s | 2026-10-31 | rc=0 ×3 |
| A→C | `tools/route_A_to_C_wall.json` | 17 | 96.1 s / 95.6 s | 2026-10-31 | rc=0 ×2 |
| A→D | `tools/route_A_to_D_wall.json` | 21 | 116.8 s / 117.1 s | 2026-10-31 | rc=0 ×2 |
| A→E | `tools/route_A_to_E_wall.json` | 22 | 140.2 s / 142.4 s | 2026-10-31 | rc=0 ×2 |

文件 SHA256（前 16 位）：
- A→B `1794b3e7a6a7eeb1`
- A→C `16e804cc14fb6d9f`
- A→D `ef068134466de620`
- A→E `dae86def5a77c488`

### 1.2 执行与工具链

| 文件 | 作用 |
| --- | --- |
| `tools/t4_autonav.py` | **Pi 端自主执行器**：一次下发整条路线，逐阶段执行（按墙触发 / 弧线转向 / 倒车 / 定距），断网继续跑，本地写 `~/t4_logs/autonav_*.jsonl` |
| `tools/t4_wallrun.py` | 按墙触发单步执行（调试用） |
| `tools/t4_steps.py` | 分段执行 + 每段实测（调试用） |
| `tools/t4_lite_drive.py` | UDP 链路工具（链路检查/遥测/相机测距/单段运动） |
| `tools/closed_loop_calib.py` | 闭环标定（速度/转向/滑行），写 `calib.json` |
| `tools/t4_pi_logger.py` + `tools/t4_logs.py` | Pi 本地数据采集（断网不丢数据） |
| `tools/pull_logs.ps1` | PC 端拉取 Pi 日志并清理空间 |
| `tools/gen_routes_wall.py`、`build_routes_clean.py`（work 中） | 路线生成与整车轮廓净空验算 |

### 1.3 位流与标定

- 位流：闭环 PI 版，SHA256 `57BE714A7616FECC95164303C3E1CA8C5E3850AA1FC0E8C8C7C385779D0865B7`
- 烧写：SRAM（**断电丢失，重新上电必须重烧**）
  `programmer_cli --device GW5A-25A --run 2 --fsFile <...>/t4_lite_top.fs --cable "Gowin USB Cable(FT2CH)"`
- 标定（`calib.json`）：`cmd_scale=0.9129`，转向差速 ±50 → ±24.8 °/s，STOP 滑行约 2°，
  弧线 `arc_radius=200 / arc_v=55`，三点掉头 `tp_v=60 / tp_diff=30`

---

## 2. 技术路线与方法

### 2.1 场地与车辆几何（实测校准）

- 场地：3 m × 3 m，600 × 600 mm 方块网格；墙板 30 cm（600 mm 长），安装存在 1–2 cm 误差
- 车辆：320 mm（长）× 240 mm（宽），**绕后轴转向**（轮轴距车尾 25 mm）
  - 原地转向扫掠半径 **≈319 mm**
  - 弧线转向（R=200）外侧前角需 **≈435 mm**

### 2.2 核心方法：方块路口 + 按墙触发

1. **方块路口法**：车完整停进 600×600 方块，在方块内完成 90° 弧线转向；转完位于下一条走廊中线
2. **按墙触发**：每段停位由 ToF 实测墙距决定（稳定触发：连续 8 帧在目标 ±15 mm 内），
   安装误差与里程误差不累积
3. **闭环弧线转向**：以 IMU 航向为目标，差速 `vl=v(1-track/2R)`；STOP 后等滑行再判定
4. **图纸预验**：`gen_routes_wall.py` 的整车轮廓净空计算（A→B/C/D 最小 39.8 mm，A→E 30 mm）

### 2.3 已实现的自主执行特性

- Pi 端整条路线自主执行（只需局域网 UDP + ToF，**SSH/WiFi 断开不影响**）
- 两段速（远离墙 60 mm/s，接近墙 20 mm/s）
- 每步本地日志（`~/t4_logs/autonav_*.jsonl`，断网后回 PC 拉取）
- 安全保护：车头 <70 mm 连续 3 帧急停、弧线超距止转、故障锁存后自动重新使能

---

## 3. 测试记录摘要

- 闭环标定：50 → 54.8 mm/s，100 → 100.2 mm/s，转向 ±24.8 °/s，滑行 2.0°
- 分段测试 12 项全过：直行 ±8 mm/1 m、原地转 ±4°、弧线转 ±1°、三点掉头 ±3°、对墙停 120 mm ±8 mm
- 四条路线实车自主跑通（见 1.1），转向误差 ≤1°，停距误差 ≤15 mm
- 连续 A→B→C 曾一次跑通（A→B 87.5 s + B 出口/B→C 57.3 s，rc=0）

## 4. 未完成项与已知限制（如实记录）

| 项 | 状况 | 原因 |
| --- | --- | --- |
| 连续 A→B→C→D→E | 未通过 | ① C→D 右转每次多转约 7°（STOP 后滑行 2–7° 未补偿）；② 长直段无墙基准，横向偏差累积；③ ToF 在开口/墙端会误触发 |
| D 点 | 仅 A→D 单点达标 | D 是只朝北开口的口袋，出场需倒车+左转（已算方案：倒车 505 mm 后东转，净空 74.7 mm，未实车验证） |
| B 点出口 | 单步验证通过（倒车 495.5 mm + 右转 89.3°） | 连段时受位置偏差影响 |
| 左右 ToF | 未接入 | 现有位流只暴露前向 ToF；曾尝试加两路+扩展遥测，因"左右 ToF 只做偏置检测、跳变多"及时间原因回滚 |
| 地图精度 | 图片估算 | 内部墙位未逐点实测，安装误差 1–2 cm 会放大转向点偏差 |

## 5. 复现步骤

```bash
# 0) 上电后先烧位流（SRAM 掉电丢失）
programmer_cli --device GW5A-25A --run 2 --fsFile <...>/t4_lite_top.fs --cable "Gowin USB Cable(FT2CH)"

# 1) 链路检查（期望 fault=0x20，ToF 有读数）
ssh aipro "python3 ~/t4_tools/t4_lite_drive.py --link-check"

# 2) 闭环标定（车放空旷处，前方 ≥1.5 m）
ssh aipro "python3 ~/t4_tools/closed_loop_calib.py"     # 写 calib.json

# 3) 把车摆到 A 区（车头朝 +Y、车身与墙平行），确认静止 yaw ≈ 0°

# 4) 跑路线（Pi 端自主执行，可断网）
ssh aipro "cd ~/t4_tools && setsid nohup python3 t4_autonav.py \
  --route route_A_to_B_full_wall.json --fast 60 --slow 20 --slow-gap 250 --arc-v 55 \
  > ~/t4_logs/run.out 2>&1 &"

# 5) 回 PC 拉日志并释放 Pi 空间
powershell -ExecutionPolicy Bypass -File pull_logs.ps1
```

## 6. 如果继续（建议的下一步）

1. **转向滑行补偿落实验证**：`t4_autonav.arc()` 已实现"等滑行 + 原地微脉冲补正"（每次 ≤2°），
   但没有在真实路线上验证；先在 A→B 上对比开启/关闭的终点误差
2. **C→D 专项**：把东行段改为"编码器固定距离 + 到墙前 300 mm 再停"的组合，并用航向闭环确认 180° 累计误差 <2°
3. **场地微调**（最省力的连续演示方案）：B 口袋北墙、D 口袋南墙各外移 15 cm，使路口能原地掉头
4. **地图实测**：卷尺量 6–8 个关键墙位，更新 `tests/task4_navigation/data/map_v1.json` 后重新规划

---

附：当日详细过程见 `records/2026-10-31_分段测试记录.md`、`records/2026-10-31_按墙触发测试记录.md`、
`records/2026-10-31_总结与计划.md`；历史路线版本备份在同仓库外的 `work/legacy_routes/`。

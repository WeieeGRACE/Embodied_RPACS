# 任务四 落地测试 Runbook（上板照做，省时间）

> 目标：把 **A→OPEN_WAYPOINT_1 单段探针**（50 mm/s，硬限速 60 mm/s）在地面上跑通，并自动判读结果。
> 完整安全说明见 [FIELD_TEST_GUIDE.md](FIELD_TEST_GUIDE.md)；工具自检见 §6。

---

## 0. 速度限制（已做进 RTL，无法越线）

| 层 | 限制 |
| --- | --- |
| 段表 | 本段 `speed_mm_s = 50` |
| 航向修正 | 每轮最多 ±`speed/3`（约 ±17 mm/s） |
| **顶层硬限幅** | `nav_speed_limit`：左右轮目标速度 **钳死在 ±60 mm/s**（新增，RTL 最后一道闸门） |
| PID 输出 | 占空比上限 25%（`PWM_LIMIT`） |
| 斜坡 | `pwm_and_startup_ramp` 逐拍上升 |

> 即使段表/航向修正写错、或以后调参调大，轮速也超不过 60 mm/s。改这个值要改 `MAX_WHEEL_SPEED_MM_S` 并重编位流。

---

## 1. 烧位流（二选一）

| 变体 | 文件 | SHA-256 | 调试串口 |
| --- | --- | --- | --- |
| **无线（推荐，车不用拖线）** | `impl/pnr/task4_navigation_single_leg_probe_wireless.fs` | `270D817D…222CC7` | L12（P6-21 → WCH-Link → PC COM24） |
| USB | `impl/pnr/task4_navigation_single_leg_probe.fs` | `F09B6BCC…49CFF4` | V8（板载 CH9102） |

```powershell
cd tests\task4_navigation
# 无线版（推荐）
programmer_cli --device GW5A-25A --run 2 `
    --fsFile (Resolve-Path impl\pnr\task4_navigation_single_leg_probe_wireless.fs) `
    --cable "Gowin USB Cable(FT2CH)"
# 或 USB 版：--fsFile (Resolve-Path impl\pnr\task4_navigation_single_leg_probe.fs)
```

烧完建议先跑一次位流校验（PC 上）：

```powershell
python tools\t4_floor.py --bitstream impl\pnr\task4_navigation_single_leg_probe_wireless.fs --check-only
```

---

## 2. 上电检查（电机电源先断）

1. 复位后不按启动键：`STBY/PWMA/PWMB` 必须为低（万用表）。
2. 等 `led_ready` 亮；OLED 三路 ToF 有读数且刷新；IMU 静止初始化完成。
3. 电机电源仍断开时按一下启动键：`led_run` 不应亮（传感器/标定门未过时驱动被禁止）。
4. 接电机电源、**车架空**：按启动键，确认两轮方向正确（沿 +Y 前进）、急停键能立即切断。
5. 急停后释放再复位，确认**不自动续行**。

以上都过 → 进入地面单段。

---

## 3. Pi 侧：一条命令采集 + 判读

```bash
# 无线版：WCH-Link 在 Pi 上通常枚举为 /dev/ttyACM0（或 /dev/serial/by-id/*1a86*）
# USB 版：板载 CH9102 也是 /dev/ttyACM0 或 /dev/ttyUSB0
cd ~/rpacs_phase0 2>/dev/null || true

# 一条命令：采集 60 s + 自动判读 + 生成报告
~/rpacs_phase0/pyorbbecsdk-main/.venv/bin/python <repo>/tests/task4_navigation/tools/t4_floor.py \
    --seconds 60

# 不知道端口时它自动探测；也可显式指定：
... t4_floor.py --seconds 60 --port /dev/ttyACM0
```

> PC 上同理：`python tools\t4_floor.py --seconds 60 --port COM24`（WCH-Link）。

**操作节奏**：命令跑起来后 → 把车放到 A 起点、车头朝 +Y → 清空周边 → 按启动键 → 等车停稳。
采集结束后自动打印：

```
[floor] 判定：PASS
         T4 SEG=0 DIST=417mm YAW=0.690° WALL=MID_H FAULT=NONE
[floor] 报告：records/floor_<时间>.md
```

---

## 4. 判定含义

| 判定 | 含义 | 下一步 |
| --- | --- | --- |
| **PASS** | 墙段 MID_H 命中、里程 400–430 mm、航向 ±5° 内、无故障 | 记录实测停车距离/轮差，更新动态参数 |
| **SAFE_STOP** | 有故障码或未确认墙，但在硬停 520 mm 内停住 | 查 FAULT：1 STOP/2 ESTOP/3 RANGE/4 INPUT/5 GEOMETRY |
| **FAIL** | 无记录、距离越界、超硬停、未知故障码、或记录了多段 | 保留日志，先别重试，按 §5 排查 |

退出码：0=PASS，2=SAFE_STOP，1=FAIL（脚本可直接串到流程里）。

---

## 5. 失败分支速查

| 现象 | 优先检查 |
| --- | --- |
| 完全没有记录 | 串口接线/波特率（115200）；是否按了启动键；`led_ready` 是否亮 |
| DIST 明显 < 400 | 启动点不在 A、车头方向偏、ToF 提前看到墙、轮径/编码器标定 |
| WALL 不是 MID_H（=10） | 场景不对（MID_H 不在正前方 100–700 mm 内）、ToF 老化/遮挡 |
| FAULT=3 RANGE | ToF 数据过期/遮挡；检查三路刷新与 age |
| FAULT=4 INPUT | 编码器非法跳变/IMU 异常；查线束与干扰 |
| FAULT=5 GEOMETRY | 位姿/地图门限不满足；检查起点与朝向 |
| FAULT=1/2 | 普通停止/急停（人工触发即正常） |

---

## 6. 工具链自检（不用板子，可先确认工具没问题）

```bash
python tools/t4_selftest.py            # 13 个场景全 PASS 才算工具正常
python tools/t4_floor.py --replay records/_selftest_replay.log   # 回放生成报告
```

---

## 7. 测完要记的数（用于更新碰撞预测假设）

| 项 | 实测值 |
| --- | --- |
| 停车距离（触发停止到车停稳） | ____ mm（原假设 20 mm） |
| 反应行程（人/系统延迟×速度） | ____ mm（原假设 13.5 mm） |
| 左右轮里程差 | ____ mm |
| 三路 ToF 刷新周期 / 有效年龄 | ____ ms / ____ ms |
| 最终停车点相对目标偏差 | ____ mm |

把这些填进 `records/` 的本次报告，再决定是否用实测值重跑 `tools/collision_prediction.py`。

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
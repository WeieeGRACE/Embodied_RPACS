# t4_lite_drive —— 深度相机测距 + 编码器执行 + ToF 兜底停车

> 目的：不再依赖 ToF 测距与地图关联。**真实距离由 Pi 深度相机测定**，FPGA 只负责
> "按编码器里程执行 + ToF 近了就停"。ToF 固定偏置（前 +60 mm）已在 FPGA 内标定。

## 1. 结构

| 侧 | 内容 |
| --- | --- |
| FPGA | `t4_lite_top`：ETH UDP 链路（SET_VEL/STOP/心跳）+ 双轮编码器里程 + 前向 ToF 兜底停车 + TB6612 |
| Pi | `tools/t4_lite_drive.py`：深度相机测墙距 D → 目标 = D − 120 mm → 20 Hz SET_VEL、50 Hz 读 TELEM 里程 → 到点 STOP |

遥测 `enc_l/enc_r` = 左右轮累计里程（**mm**），主机用它闭环。

## 2. 网络

- FPGA：192.168.0.2:5000（接收），回复广播 192.168.0.1:5001
- Pi eth0：192.168.0.1/24；静态 ARP `192.168.0.2 → 00:0a:35:01:fe:c0`（`nmcli` profile `ethfpga`）

## 3. Pi 上用法

```bash
# 深度相机环境（Orbbec SDK）
cd ~/rpacs_phase0/pyorbbecsdk-main
export PYTHONPATH=install/lib

# 只测距（不动车）
.venv/bin/python <repo>/tests/t4_lite_drive/tools/t4_lite_drive.py --measure-only

# 测距 + 自动开一段（默认余量 120 mm，速度 100 mm/s）
.venv/bin/python <repo>/tests/t4_lite_drive/tools/t4_lite_drive.py --speed 100

# 不用相机，直接给行程
.venv/bin/python <repo>/tests/t4_lite_drive/tools/t4_lite_drive.py --no-camera --target 800
```

输出示例：

```
[cam] 墙面距离 中值=951 mm  最小=948 mm  (帧 20/20)
[drive] 目标行程 = 951 - 120 = 831 mm
[drive] 链路在线：enc=(0, 0) fault=NONE
[drive] 到达目标：里程 831.4 mm >= 831 mm，发 STOP
[drive] 完成：最终里程 833.2 mm（目标 831），fault=NONE，v=(0,0)
```

## 4. 安全（全部硬连线，不依赖 Pi）

1. 心跳 300 ms 超时 → `link_ok=0` → STBY=0
2. 命令 500 ms 超时 / STOP 锁存 / 任一故障位 → STBY=0
3. **ToF 兜底停车**：前向 ≤120 mm → 立即切断电机（独立于上位机）
4. KEY2（C15）急停
5. 速度硬限幅（`MAX_V_MM_S`，默认 300 mm/s；本工具默认只开 100 mm/s）

## 5. 上板步骤

1. 烧 `impl/pnr/t4_lite_top.fs`
2. Pi 网口接 FPGA eth0，确认 `ip addr` 有 192.168.0.1/24
3. 车放正、车头朝墙（距离 0.5–2 m 内），`--measure-only` 确认相机距离与卷尺一致（±30 mm）
4. `--speed 100` 跑一段；车应按相机距离 −120 mm 停下
5. 有异常：KEY2 急停 / 直接断电

## 6. 已知限制

- 深度相机需在 Pi 上运行（USB 3.0 口），测距取中央 ROI 中值；墙面倾斜/强反光会影响
- 编码器为开环里程（无 IMU 融合），长距离（>2 m）会有 1–3% 累积误差；本测试段通常 <1 m
- ToF 只做兜底，不参与距离判断

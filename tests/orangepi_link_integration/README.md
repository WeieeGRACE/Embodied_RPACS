# 深度相机板（Orange Pi AIpro）↔ FPGA 链路联调

> 目的：在同一块 **Orange Pi AIpro** 上同时跑「深度相机」和「FPGA 链路」，证明两者互不拖累，
> 并产出可用于后续建图的「遥测 + 相机时间戳」数据。
>
> **本工程独立于主工程**：不改 `src/`、不改 `build.tcl`、不动主工程位流；只引用
> `tests/uart_bridge/tools` 里的协议编解码（单一事实来源）。任何新测试都放在
> `tests/<名字>/` 下，验证通过后才谈移植。

---

## 1. 它测什么

| 用例 | 内容 | 判定 |
| --- | --- | --- |
| **P** 预检 | 串口存在/可开、3 s 内收到遥测、速率 45~60 Hz、解析错误 0、心跳年龄 ≤150 ms、故障字节可读 | 7 项全 PASS |
| **L** 链路采集 | 10 Hz 心跳 + 50 Hz 遥测，逐帧记 CSV（编码器、姿态、速度、故障、序号、心跳年龄） | 帧数>0，速率达标 |
| **C** 相机并发 | 相机按 8 fps 抓深度帧，同时链路照常跑 120 s | 相机 fps ≥6、链路最低速率 ≥45 Hz、CPU 均值 ≤85% |
| **R** 报告 | 上述 JSON 汇总成 Markdown | 自动生成，含 PASS/FAIL 结论 |

这三项合起来回答一个问题：**"相机抢占 CPU/带宽时，50 Hz 遥测会不会掉、会不会出现假失联"** —— 这正是后续建图能不能用 FPGA 姿态/里程计的前提。

---

## 2. 硬件接法（二选一）

### 方案 A：FPGA 的 USB 串口线插到 Pi（推荐，免接线）

```
FPGA 板 (GW5A-25A)  --- USB(CH9102, 即现在接电脑 COM14 的那根线) --->  Orange Pi USB 口
```
Pi 上会出现 `/dev/ttyUSB0`（或 `/dev/serial/by-id/usb-1a86_...CH9102...`）。

### 方案 B：用 Pi 的 40pin UART 接 FPGA 的 UART 引脚

| FPGA 引脚 | 方向 | Pi 40pin | 说明 |
| --- | --- | --- | --- |
| `uart_tx` V8 | FPGA → Pi | Pin 10 (RXD) | 3.3 V TTL |
| `uart_rx` U8 | Pi → FPGA | Pin 8 (TXD) | 3.3 V TTL，需交叉 |
| GND | — | Pin 6 | **必须共地** |

Pi 侧要启用 UART：`sudo raspi-config`→Interface→Serial（关 console、开 hardware），设备为 `/dev/ttyS0` 或 `/dev/ttyAMA0`；
此时把 `config/link.json` 的 `ports.linux` 改成实际设备名。

> ⚠️ 只用 3.3 V TTL：**不要接 5 V**；FPGA 侧输入已有内部上拉。

### 位流要求

FPGA 里必须跑 **链路测试位流**（SRAM，掉电即失效）：

```powershell
cd tests\uart_bridge
.\run_build.ps1 -Download          # 完整链路版（含 ACK）
# 或带真实 IMU 姿态的版本：
gw_sh build_imu.tcl
programmer_cli --device GW5A-25A --run 2 --fsFile impl\pnr\uart_bridge_imu.fs --cable "Gowin USB Cable(FT2CH)"
```

该位流把 `PWMA/AIN1/AIN2/PWMB/BIN1/BIN2/STBY` 全拉 0（TB6612 standby），**电机不可能动**；
只读编码器与 IMU。主工程位流仍在 Flash/可随时重新下载，本工程只写 SRAM。

---

## 3. 板子环境准备（一次性）

```bash
# 1) 串口权限：把用户加入 dialout（之后重新登录生效）
sudo usermod -aG dialout HwHiAiUser

# 2) pyserial（装进相机 venv 即可，已在 2026-10-29 装好 3.5）
~/rpacs_phase0/pyorbbecsdk-main/.venv/bin/pip install pyserial

# 3) 确认设备
ls -l /dev/ttyUSB* /dev/serial/by-id/ 2>/dev/null
lsusb | grep -i 1a86        # CH9102 应出现
```

---

## 4. 从 PC 一键部署 + 预检

```powershell
cd tests\orangepi_link_integration
.\tools\deploy_to_pi.ps1              # 部署到 ~/rpacs_link_integration 并跑预检
.\tools\deploy_to_pi.ps1 -InstallDeps # 顺带装 pyserial
```

部署内容：本工程 `src/config/tools` + `tests/uart_bridge/tools/*.py`（协议编解码，不复制代码）。

## 5. 板上直接运行

```bash
cd ~/rpacs_link_integration
export UART_BRIDGE_TOOLS=$PWD/uart_tools
PY=~/rpacs_phase0/pyorbbecsdk-main/.venv/bin/python

$PY src/preflight.py                                   # 7 项预检
$PY src/link_client.py --duration 10 --csv records/link.csv --json records/link.json
$PY src/camera_link_soak.py --duration 120 --fps 8 --json records/soak_cam.json
$PY src/report.py records/*.json --out records/report.md
```

PC 上无相机时可先验证链路：`python src\camera_link_soak.py --port COM14 --duration 30 --no-camera`

## 6. 无硬件的离线自检

```powershell
python tools\offline_selftest.py
```
合成 50 Hz 遥测字节流（含坏帧）→ 用 `FileTransport` 回放 → 校验解析、速率、CSV、报告渲染。
**当前状态：7/7 PASS**（无需任何硬件，可随时回归）。

---

## 7. 目录结构

```text
tests/orangepi_link_integration/
├── README.md                  本文
├── config/link.json           端口/波特率/验收阈值（改这里即可）
├── src/
│   ├── link_client.py         心跳+遥测采集+CSV（基础件）
│   ├── preflight.py           7 项链路预检（含可离线测试的 analyze()）
│   ├── camera_link_soak.py    相机 + 链路并发 soak
│   └── report.py              JSON → Markdown 报告
├── tools/
│   ├── offline_selftest.py    无硬件自检（7 项）
│   └── deploy_to_pi.ps1       PC → Pi 部署 + 远端预检
└── records/                   运行结果（JSON/CSV/MD）
```

---

## 8. 当前状态（2026-10-29 13:23，**已实测通过**）

| 项 | 状态 |
| --- | --- |
| 工程搭建 | ✅ 7 个脚本 + 配置 + 文档 |
| 离线自检 | ✅ **8/8 PASS**（`python tools/offline_selftest.py`，无需硬件） |
| 硬件接线 | ✅ FPGA 板载 USB 串口(CH9102) → Pi `/dev/ttyACM0`；Orbbec 深度相机 → Pi USB3 |
| FPGA 位流 | ✅ `uart_bridge_imu.fs`（SRAM；断电后需重烧） |
| 预检 | ✅ **7/7 PASS**（50.00 Hz、0 解析错误、心跳年龄 95 ms） |
| 链路采集 30 s | ✅ 1500 帧零丢失，帧间隔 20.000 ms（抖动 ≤0.1 ms），yaw 漂移 0.01 °/s |
| 相机+链路并发 120 s | ✅ **9/9 PASS**：相机 8.01 fps / 963 帧，链路 50.0 Hz（最低 49.0），CPU 30.8% |
| 完整记录 | `records/2026-10-29_B1_深度相机与链路并发测试.md` |

### 复现命令

```powershell
# 1) PC：烧链路位流到 FPGA SRAM（板子断电后必须重做）
cd tests\uart_bridge
gw_sh build_imu.tcl
programmer_cli --device GW5A-25A --run 2 --fsFile impl\pnr\uart_bridge_imu.fs --cable "Gowin USB Cable(FT2CH)"

# 2) PC → Pi 同步
cd tests\orangepi_link_integration
.\tools\deploy_to_pi.ps1 -SkipPreflight
```

```bash
# 3) Pi：全套（预检 → 链路 30 s → 相机并发 120 s → 报告）
cd ~/rpacs_link_integration
PYTHONPATH=$HOME/rpacs_phase0/pyorbbecsdk-main/install/lib bash run_all.sh 120
```

结果在 Pi 的 `~/rpacs_link_integration/records/`，可用 `scp` 拉回。

---

## 9. 与主工程的关系

- 本工程**只读**引用 `tests/uart_bridge/tools`（协议库）与 FPGA 的串口，不改主工程任何文件；
- 验证通过后，移植动作只有两步：① 把链路 RTL 合进主工程（见 `tests/uart_bridge/docs/INTEGRATION_GUIDE.md`）；
  ② 把 Pi 侧 `link_client` 的采集循环接到建图脚本上。两者都留到测试通过后再做。

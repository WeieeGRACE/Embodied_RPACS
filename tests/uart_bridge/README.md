# uart_bridge：上位机 ↔ FPGA 链路的独立测试工程

> 对应 `tests/orangepi_depth_perception/PLAN.md` §9.1（链路 v1）与决策 **D3/R12**：
> **独立测试工程、主工程零改动**。本工程只做通信链路与协议验证，
> **不驱动电机、不接管现有控制逻辑**。

---

## 1. 这个工程解决什么

深度相机（Orange Pi AIpro）需要给主控下发速度/停车命令，并把编码器、姿态、故障
回传给上位机做建图与决策。本工程把这个链路**从协议到板级验证**完整做出来：

```text
上位机 (PC / Orange Pi)
   │  命令 20 Hz, 心跳 10 Hz
   ▼
UART 115200 8N1  (PC 侧 COM14, FPGA 引脚 U8/V8)
   │
   ▼
FPGA  ┌─────────── uart_bridge_top ────────────┐
      │ uart_rx ─► host_frame_rx ─► host_cmd_decode │
      │                │                  │        │
      │          crc16_ccitt        host_link_watchdog
      │                │                  │        │
      │                └─► uart_bridge_core ◄─ telem_builder
      │                          │                 │
      │                    frame_tx_engine ─► uart_tx ─► 遥测 50 Hz
      │ 编码器输入（只读计数）──┘                    │
      │ 电机输出全部拉低 + STBY=0（安全）           │
      └────────────────────────────────────────────┘
```

**安全边界（重要）**：本工程的位流把 `PWMA/AIN1/AIN2/PWMB/BIN1/BIN2/STBY` 全部拉 0
（TB6612 处于 standby，输出关闭），ToF/OLED 总线释放为高阻，只保留串口、LED 与编码器输入。
因此**烧写本工程后车不会动**；断电重启即恢复原有固件（只写 SRAM，不碰外部 Flash）。

---

## 2. 目录结构

```text
tests/uart_bridge/
├── docs/PROTOCOL_v1.md      协议冻结文档（帧格式、消息、时序、CRC、安全、以太网迁移）
├── src/                     RTL（纯 Verilog-2001，50 MHz）
│   ├── uart_rx_8n1.v        8N1 接收（过采样、中点采样、帧错误）
│   ├── crc16_ccitt.v        CRC-16/CCITT-FALSE（逐位展开，约 40 LUT，无查找表）
│   ├── host_frame_rx.v      帧解析：AA 55|TYPE|LEN|PAYLOAD|CRC|0D + 错误分类 + 超时
│   ├── host_cmd_decode.v    命令解码（SET_VEL/STOP/HEARTBEAT/SET_LIMIT/CLEAR_FAULT）
│   ├── host_link_watchdog.v 心跳看门狗（默认 300 ms，锁存失联）
│   ├── telem_builder.v      23 字节遥测快照
│   ├── frame_tx_engine.v    发帧引擎（边发边算 CRC）
│   ├── uart_bridge_core.v   传输无关核心 + 安全仲裁
│   ├── uart_bridge_top.v    板级顶层（不接电机）
│   └── uart_bridge_top.cst/.sdc
├── sim/                     ModelSim 测试（SystemVerilog）
│   ├── tb_core.sv           36 项：CRC 向量、帧解析错误谱、命令/ACK、安全、遥测节拍
│   ├── tb_loopback.sv       24 项：真实 115200 双向回环，ACK/TELEM 内容校验
│   ├── tb_motion_adapter.sv 19 项：电机侧适配器（模式/方向/幅值/刹车锁存/去重）
│   ├── tb_system_closed_loop.sv 9 项：串口→链路→适配器→轮子模型→编码器→遥测 全闭环
│   └── run_*.do
├── tools/                   上位机工具（Python + PowerShell）
│   ├── protocol.py          与传输无关的编解码（流式解析 + 错误统计）
│   ├── transport.py         UART / UDP / 文件回放 三种传输实现
│   ├── uart_link.py         CLI：ports/selftest/monitor/send/bench
│   ├── soak_test.py         长稳测试（默认 10 min，统计误码率与遥测速率）
│   ├── pi_host_example.py   Orange Pi 侧最小示例（--list/--monitor/--setvel）
│   ├── protocol_selftest.py 协议自检（34 项）
│   └── uart_link.ps1        PowerShell 前端
├── records/                 构建日志、台架测试 JSON、长稳日志、验证报告
├── build.tcl / run_build.ps1 高云综合脚本（gw_sh）
├── run_sim.ps1              ModelSim 全回归脚本
└── uart_bridge.gprj         高云 IDE 工程（可直接打开）
```

---

## 3. 快速开始

### 3.1 仿真（无需硬件）

```powershell
cd tests/uart_bridge
.\run_sim.ps1
```

预期：`compile / core / loopback` 三个套件全部 PASS（约 1 分钟）。

### 3.2 综合 + 资源报告

```powershell
cd tests/uart_bridge
.\run_sim.ps1              # 仿真回归（compile + core + loopback）
.\run_build.ps1            # 纯链路版本综合，打印资源
.\run_build.ps1 -Download  # 纯链路版本综合 + 烧写 SRAM（易失）

# IMU 变体（遥测带真实 yaw/pitch/roll）
D:/Gowin/Gowin_V1.9.12_x64/IDE/bin/gw_sh.exe build_imu.tcl
D:/Gowin/Gowin_V1.9.12_x64/Programmer/bin/programmer_cli.exe --device GW5A-25A --run 2 `
    --fsFile impl/pnr/uart_bridge_imu.fs --cable "Gowin USB Cable(FT2CH)"
```

当前资源（GW5A-25，50 MHz）：

| 顶层 | Logic | Register | CLS | 备注 |
| --- | --- | --- | --- | --- |
| `uart_bridge_top`（纯链路） | 1066（5%） | 808（4%） | 804（7%） | 基准版本 |
| `uart_bridge_imu_top`（链路 + 真实姿态） | 5305（24%） | 2529（11%） | 3615（32%） | +IMU 驱动/姿态解算、4 BSRAM、10 DSP |

两个顶层分别用 `build.tcl` / `build_imu.tcl` 综合，产物 `impl/pnr/uart_bridge.fs` 与
`impl/pnr/uart_bridge_imu.fs`。

### 3.3 串口台架测试

先关掉 Fluent Serial Assistant（一个 COM 口只能一个程序占用）：

```powershell
.\tools\uart_link.ps1 -Action ports
.\tools\uart_link.ps1 -Action monitor -Port COM14     # 看 50 Hz 遥测
.\tools\uart_link.ps1 -Action bench   -Port COM14     # 19 项全链路验收
python tools\soak_test.py --port COM14 --duration 600 # 10 min 长稳
```

`bench` 覆盖：遥测速率、CLEAR_FAULT、SET_VEL/ACK/seq 回显、限速裁剪、
STOP 锁存、失联检测（300 ms）、恢复、误码率。

---

## 4. 协议摘要（详见 docs/PROTOCOL_v1.md）

```text
帧：AA 55 | TYPE | LEN | PAYLOAD | CRC16-CCITT(小端) | 0D   (总长 = 7 + LEN)
下行：0x01 SET_VEL(5B: int16 L, int16 R, uint8 seq) 0x02 STOP 0x03 HEARTBEAT(seq)
      0x04 SET_LIMIT(u16) 0x05 CLEAR_FAULT
上行：0x81 TELEM(23B: 编码器/姿态/速度/故障位/序号/心跳年龄)  0x82 ACK(3B)
```

安全规则：
- 上位机只写"目标速度"寄存器，**从不直接写 PWM**；
- 心跳超时 300 ms → `LINK_LOST` 锁存 + 目标速度归零；
- `STOP` 锁存，必须 `CLEAR_FAULT` 且链路在线才能解除；
- 任何解析错误只丢弃，不改变任何寄存器。

> **换千兆网时**：帧格式、消息、CRC、安全逻辑都不变，只替换 `transport.py` 的
> 实现（已提供 `UdpTransport`）与 RTL 的字节源适配器（`host_frame_rx` 零改动）。

---

## 5. 实测结果（2026-10-29，板级）

| 项目 | 结果 |
| --- | --- |
| 综合 + PnR + 位流 | ✅ 通过（GW5A-25A，50 MHz，无时序告警） |
| 仿真回归 | ✅ `tb_core` 36 项 + `tb_loopback` 24 项全过 |
| 遥测速率 | ✅ 50.0 Hz（2250 B/s @115200，带宽占用约 20%） |
| 台架验收 bench | ✅ **19/19 PASS**（含失联 299 ms 检测、STOP 锁存、限速裁剪） |
| 10 min 长稳 | ✅ **30,000 帧 / 0 误码**，速率 50.0 Hz（±0.1） |
| 电机安全 | ✅ 电机输出全 0 / STBY=0，位流不可能驱动轮子 |

详细证据见 `records/`（`2026-10-29_uart_bridge_validation.md`、bench JSON、长稳日志）。

---

## 6. 与主工程集成时的资源结论（用户明确关注点）

主工程 `Embodied_RPACS` 现状：**Logic 20334/23040（89%）、CLS 10728/11520（94%）**。
剩余：Logic 2706、CLS 792。

本链路核心需要 **Logic 1066、CLS 804**：

| 维度 | 结论 |
| --- | --- |
| Logic | ✅ 放得下（剩 1640） |
| **CLS** | ❌ **差 12 个**（804 > 792），集成后 CLS 会到 100% 以上 |

因此**直接把本工程合进主工程不可行**，三个可选路径（推荐第 1 条）：

1. **以 `tests/task4_navigation` 为基座重新划分**（该工程 CLS 仅 53%，PLAN §9.3 已建议）；
2. 精简本链路后再合入（可裁项：8 位化诊断计数器、去掉 ACK、去掉 tx 侧 rd 地址复用），
   预计可降到 **约 650 CLS**，留 140 余量；
3. 裁掉主工程 UI/日志 RTL 后合入（回归面最大，不推荐）。

> 结论已写入 `records/2026-10-29_uart_bridge_validation.md` §6 与 PLAN §9.3 的后续动作。

---

## 7. 已知限制 / 下一步

| 项 | 状态 |
| --- | --- |
| IMU 姿态（yaw/pitch/roll） | 本工程遥测中固定为 0；IMU 驱动（`src/imu/imu660ra_driver.v`）未合入，避免资源与引脚冲突。下一步做"链路 + IMU"最小组合板 |
| 编码器速度换算 | `COUNTS_PER_M=1116` 为占位值，需按实车轮径/减速比标定 |
| 以太网 v2 | 协议层已就绪（`UdpTransport`），RTL 需新增 RGMII/UDP 字节源 |
| 与 ASRPRO 语音通道仲裁 | 仲裁优先级已在设计中预留（急停 > 停车 > 上位机 > 语音），本工程未接语音通道 |

---

## 8. 相关文档

- 协议冻结版：`docs/PROTOCOL_v1.md`
- 验证报告：`records/2026-10-29_uart_bridge_validation.md`
- 上游计划：`tests/orangepi_depth_perception/PLAN.md` §9
- 主工程：`src/robot_system_top.v`、`src/uart/`

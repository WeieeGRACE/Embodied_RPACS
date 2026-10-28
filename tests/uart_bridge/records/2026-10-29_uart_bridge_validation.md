# UART 链路 v1 验证报告（独立测试工程 `tests/uart_bridge`）

| 项 | 值 |
| --- | --- |
| 日期 | 2026-10-29 |
| 范围 | 上位机 ↔ FPGA 链路 v1：帧协议、命令分发、安全仲裁、遥测、板级验证 |
| 结果 | **综合通过、仿真 60 项全过、板级台架 19/19 全过、600 s 长稳 0 误码** |
| 器件 | GW5A-25A（GW5A-LV25UG324C2/I1），50 MHz |
| 工具链 | Gowin EDA V1.9.12、ModelSim SE-64 10.7、Python 3.12 + pyserial 3.5 |
| 串口 | COM14（CH9102 USB-UART），115200 8N1 |
| 上游依据 | `tests/orangepi_depth_perception/PLAN.md` §9.1 / §9.4、决策 D3/R12 |

---

## 1. 交付物

| 类别 | 文件 |
| --- | --- |
| 协议冻结文档 | `docs/PROTOCOL_v1.md`（v1.1，含 CRC 向量、时序、仲裁、以太网迁移） |
| RTL（9 个模块） | `src/uart_rx_8n1.v`、`crc16_ccitt.v`、`host_frame_rx.v`、`host_cmd_decode.v`、`host_link_watchdog.v`、`telem_builder.v`、`frame_tx_engine.v`、`uart_bridge_core.v`、`uart_bridge_top.v` |
| 约束 | `src/uart_bridge_top.cst`、`.sdc`（引脚与主工程一致） |
| 仿真 | `sim/tb_core.sv`（36 项）、`sim/tb_loopback.sv`（24 项）、`run_*.do`、`run_sim.ps1` |
| 上位机工具 | `tools/protocol.py`、`transport.py`（UART/UDP/文件三种传输）、`uart_link.py`、`soak_test.py`、`protocol_selftest.py`、`uart_link.ps1` |
| 构建 | `build.tcl`、`run_build.ps1`、`uart_bridge.gprj` |
| 产物 | `impl/pnr/uart_bridge.fs`（SRAM 位流）、`impl/pnr/uart_bridge.rpt.txt` |

---

## 2. 协议要点（v1.1 相对 v1.0 的修订）

1. **`SET_VEL` 增加 `seq` 字节**（LEN 4 → 5）：FPGA 原样回显到 `TELEM.cmd_seq`，
   上位机据此统计命令丢包率。原设计遥测里有 `cmd_seq` 却没有来源，属设计缺陷，已修。
2. **心跳超时 200 ms → 300 ms**：v1.0 的"5 Hz 心跳 + 200 ms 超时"等周期，PC 侧
   任何调度抖动都会误判失联。板级实测（5 Hz 心跳）确实反复触发，故冻结为
   **心跳 10 Hz、超时 300 ms**（5 Hz 时仍有 100 ms 余量）。
3. **上电即 `link_lost=1`**：显式使能原则，第一次连接必须发一条 `CLEAR_FAULT`
   才接受速度命令（台架已验证）。

其余（帧格式、CRC 参数、消息编号、故障位图、仲裁优先级）与 v1.0 一致。

---

## 3. 仿真结果

| 套件 | 检查项 | 结果 | 日志 |
| --- | --- | --- | --- |
| `tb_core.sv` | **36 项全过**：CRC 向量 V1–V4、帧解析错误谱（CRC/尾/LEN/重同步/字节间超时/逐字节喂入）、命令解码、ACK 状态码、限速裁剪、STOP 锁存、失联与恢复、遥测节拍 | ✅ 0 fail | `records/tb_core.log` |
| `tb_loopback.sv` | **24 项全过**：真实 115200 8N1 双向回环（TB 逐位发、逐位收），ACK 与 TELEM 逐字节校验（含 int32/int16 小端、seq 回显） | ✅ 0 fail | `records/tb_loopback.log` |
| `protocol_selftest.py` | 34 项 Python 编解码自检 | ✅ 0 failures | — |

复核命令：`.\run_sim.ps1`（编译 + 两套回归，约 1 分钟）。

关键覆盖点（对应 `docs/PROTOCOL_v1.md` §8 的 T01–T12）：CRC 标准值 `0x29B1` 双向一致；
任何解析错误都不改变寄存器；`STOP` 锁存后 `SET_VEL` 被 ACK 拒绝（状态 0x02）；
失联后 `CLEAR_FAULT` 在链路未恢复时被拒（0x01）。

---

## 4. 资源与时序

| 资源 | 占用 | 占比 |
| --- | --- | --- |
| Logic | **1066** / 23040（850 LUT + 216 ALU + 0 ROM16） | 5% |
| Register | **808** / 23685（FF 802 + IO FF 6） | 4% |
| CLS | **804** / 11520 | 7% |
| SSRAM | 0 | — |
| 时钟资源 | PRIMARY 1/8（13%） | — |

- 综合/PnR/位流生成全部通过，PnR 报告无时序告警（`impl/pnr/uart_bridge.rpt.txt`）。
- 单帧处理：TELEM 30 字节 @115200 = 2.6 ms，20 ms 周期下占用约 13% 串口带宽；
  实测总带宽占用 2250 B/s ≈ 20%（含 ACK）。

---

## 5. 板级实测（COM14，烧写 SRAM）

### 5.1 台架验收 `uart_link.py bench`

```
19/19 checks passed
报告：records/2026-10-29_uart_bridge_bench.json
```

| # | 检查 | 结果 |
| --- | --- | --- |
| 1 | 遥测流存在（上电 fault=0x22 = LINK_LOST\|CMD_TIMEOUT） | ✅ |
| 2 | 遥测速率 50 Hz | ✅ 50.0 Hz |
| 3 | `CLEAR_FAULT` ACK OK | ✅ |
| 4 | 遥测 `LINK_LOST` 位被清除 | ✅ fault=0x20 |
| 5 | `SET_VEL` ACK OK | ✅ |
| 6 | ACK 回显 seq | ✅ 期望 16 得到 16 |
| 7 | 遥测回显 `cmd_seq` | ✅ cmd_seq=16 |
| 8 | 超限命令被拒（限速 50 时下发 300） | ✅ REJECT_OVER_LIMIT |
| 9 | `STOP` ACK OK | ✅ |
| 10 | 遥测出现 `STOP_LATCHED` 位 | ✅ fault=0x09 |
| 11 | 停车后 `SET_VEL` 被拒 | ✅ REJECT_STOP_LATCHED |
| 12 | 停车后 `CLEAR_FAULT` ACK OK | ✅ |
| 13 | `STOP_LATCHED` 位清除 | ✅ fault=0x00 |
| 14 | 停止心跳后检出 `LINK_LOST` | ✅ |
| 15 | 失联检出时间 | ✅ **299 ms**（超时配置 300 ms） |
| 16 | 遥测上报心跳年龄 | ✅ hb_age=317 ms |
| 17 | 恢复后 `CLEAR_FAULT` ACK OK | ✅ |
| 18 | 链路恢复（LINK_LOST 清除） | ✅ fault=0x20（仅剩命令超时，属预期） |
| 19 | CRC 误码率 | ✅ 0（109 帧） |

### 5.2 长稳测试 `soak_test.py --duration 600`（T15）

```
duration 600.0 s
frames_total 30000      error_frames 0      error_rate 0.0
telem_rate mean 50.00 Hz  min 49.95  max 50.07
parser_stats good=30000 crc_err=0 sync_err=0 tail_err=0 len_err=0 timeout=0
```

- 10 分钟 **30,000 帧零误码**（目标 < 1e-6，实测 0）；
- 遥测速率稳定在 50.0 Hz（±0.1 Hz）；
- 每 10 s 窗口的心跳年龄 14–96 ms（心跳 10 Hz，周期 100 ms），未出现误判失联；
- 证据：`records/2026-10-29_soak_600s.json`、`records/2026-10-29_soak_600s.log`。

---

## 6. 安全说明（重要）

1. **本工程位流不驱动电机**：`PWMA/AIN1/AIN2/PWMB/BIN1/BIN2/STBY` 全部拉 0
   （TB6612 STBY=0 → 输出级关闭），ToF/OLED 总线释放为高阻，只有串口/LED/编码器输入有效。
   即使收到速度命令，也只在内部寄存器里存在，**不会输出任何 PWM**。
2. **只写 SRAM**（`programmer_cli --run 2`），不碰外部 Flash：断电重启即恢复原有固件。
3. 编码器通道只做只读计数（x1 正交解码），不驱动任何执行器。
4. 若后续把链路合入整车工程，必须保留 `host_cmd_decode` 的"只写目标速度、不写 PWM"
   与 `host_link_watchdog` 的停车锁存语义（PLAN §9.4）。

---

## 7. 集成到主工程的资源结论

主工程现状：**Logic 20334/23040（89%）、CLS 10728/11520（94%）** → 剩余 Logic 2706、CLS 792。

| 维度 | 本链路需求 | 余量 | 结论 |
| --- | --- | --- | --- |
| Logic | 1066 | 2706 | ✅ 可容纳 |
| **CLS** | **804** | **792** | ❌ **差 12（集成后 >100%）** |

三条可选路径：

1. **以 `tests/task4_navigation` 为基座重新划分**（该板级工程 CLS 仅 53%），
   把链路与导航一起放进去 —— 与 PLAN §9.3 的结论一致，推荐；
2. **用"精简版"链路**（已实现并上板验证，见 §7.1）；
3. 裁掉主工程 UI/日志 RTL 再合入 —— 回归面最大，不推荐。

### 7.1 精简版实测（`uart_bridge_lite_top`，已实现并上板验证）

精简手段只有两项：诊断计数器 16→8 位 + **不发 ACK**（上位机改用 50 Hz 遥测里的
`cmd_seq` 回显判断命令是否生效）。综合结果：

| 版本 | Logic | Register | CLS | 结论 |
| --- | --- | --- | --- | --- |
| 完整版 `uart_bridge_top` | 1066 | 808 | **804** | 主工程 CLS 余量 792 → **差 12** |
| **精简版 `uart_bridge_lite_top`** | **1004** | **747** | **753** | **可放进主工程（余 39 CLS）** |

上板验证（烧写 `uart_bridge_lite.fs`，持续心跳）：

```text
TELEM frames      : 110
ACK frames (lite) : 0        <- 符合设计（精简版不发 ACK）
cmd_seq 77 echoed : True     <- SET_VEL 被接受，遥测回显
STOP bit in fault : True     <- 停车锁存生效
last fault        : 0x21     <- STOP_LATCHED | CMD_TIMEOUT（预期）
parser stats      : {'good': 110, 'crc_err': 0, 'sync_err': 0, 'len_err': 0, ...}
```

> 代价：失去逐命令 ACK（但 20 Hz 命令流 + 遥测回显足以判断生效与否）、诊断计数器上限 255。
> **39 CLS 余量仍偏紧**：接入时若还要加适配器/仲裁逻辑，建议同时走路径 1（换基座）。

> 结论：**当前形态不能直接合入主工程（CLS 差 12）**，须走路径 1 或 2。
> 这一步留到"链路 + 电机"联调时决策，不在今晚改动主工程。

---

## 8. IMU 变体（遥测带真实姿态）

用户要求"能读取板子上的传感器信息"，因此在链路核心不动的前提下加了第二个顶层
`src/uart_bridge_imu_top.v`：复用主工程已验证的 `imu660ra_driver` + `imu_attitude_fixed`
（200 Hz 采样、CORDIC/atan2/开方求姿态），把 roll/pitch/yaw（毫度）换算成 0.01° 填进 TELEM。

### 8.1 资源开销（同一工程、两个顶层分别综合）

| 顶层 | Logic | Register | CLS | BSRAM | DSP |
| --- | --- | --- | --- | --- | --- |
| `uart_bridge_top`（纯链路） | 1066（5%） | 808（4%） | 804（7%） | 0 | 0 |
| `uart_bridge_imu_top`（链路 + IMU 姿态） | **5305（24%）** | 2529（11%） | **3615（32%）** | 4（pROM，IMU 配置镜像） | 10/28 |

即 **IMU 驱动 + 姿态解算额外占 4239 Logic / 2811 CLS / 4 BSRAM / 10 DSP**。
对主工程而言这部分**本来就有**（主工程已含 IMU + 姿态），所以把链路合入主工程只多 1066 Logic
（结论仍见 §7）。独立测试工程里 24%/32% 余量充足。

### 8.2 板级实测（烧写 `uart_bridge_imu.fs`，同样只写 SRAM）

```
19/19 checks passed   （records/2026-10-29_uart_bridge_imu_bench.json，失联检出 278 ms）
```

真实姿态样例（5 s，250 帧，误码 0；未发心跳故 fault=0x22）：

```text
0.087 yaw=  +1.28 pitch=  -3.56 roll=  -1.15 spd=(0,0) fault=0x22
0.287 yaw=  +1.29 pitch=  -3.56 roll=  -1.15 spd=(0,0) fault=0x22
0.687 yaw=  +1.30 pitch=  -3.58 roll=  -1.15 spd=(0,0) fault=0x22
...
# frames=250 stats={'good': 250, 'crc_err': 0, ...}
```

- pitch ≈ −3.6°、roll ≈ −1.15° —— 小车确实不是完全水平，是真实重力的测量结果；
- yaw 在静止时缓慢漂移（+1.28 → +1.30，约 0.004°/s 量级），符合低成本 IMU 特性；
- 完整样例：`records/2026-10-29_imu_telemetry_sample.txt`。

> **烧写说明**：SRAM 掉电即失效，因此板子断电重启会恢复原来的整车固件；
> 今晚最后烧的是 **IMU 变体**（便于明早直接看真实姿态）。

---

## 9. 未完成 / 后续

| 项 | 说明 |
| --- | --- |
| IMU 姿态 | ✅ 已做：`uart_bridge_imu_top.v` 把 imu660ra + 姿态解算接进遥测（§8），实测 pitch −3.6°/roll −1.15° 为真实重力测量 |
| 编码器速度 | `COUNTS_PER_M=1116` 为占位值，需按实车轮径与减速比标定 |
| 电机闭环 | 只到"目标速度寄存器"，未接 `src/motor` 闭环与 PWM；联调时按路径 1/2 决策 |
| 以太网 v2 | 协议层已就绪（`transport.py: UdpTransport` 可直接跑同一帧格式），RTL 需新增 RGMII/UDP 字节源 |
| ASRPRO 语音仲裁 | 优先级已在设计中预留，本工程未接语音通道 |

---

## 10. 复现步骤

```powershell
cd tests/uart_bridge

# 1) 仿真（无需硬件）
.\run_sim.ps1

# 2) 综合 + 资源报告
.\run_build.ps1

# 3) 烧写 SRAM（易失）
.\run_build.ps1 -Download

# 4) 台架验收（先关闭 Fluent Serial Assistant）
.\tools\uart_link.ps1 -Action bench -Port COM14

# 5) 10 min 长稳
python tools\soak_test.py --port COM14 --duration 600 --out records/soak.json
```

---

## 11. 证据文件索引

| 文件 | 内容 |
| --- | --- |
| `records/build_synth.log`、`build_synth2.log` | 综合/PnR 日志 |
| `records/download_sram.log` | SRAM 烧写日志（User Code 0x000067E2） |
| `records/tb_core.log` | 36 项仿真 PASS |
| `records/tb_loopback.log` | 24 项真实波特率回环 PASS |
| `records/2026-10-29_uart_bridge_bench.json` | 台架 19 项逐条结果 |
| `records/2026-10-29_soak_600s.json/.log` | 10 min 长稳统计 |
| `impl/pnr/uart_bridge.rpt.txt` | 资源/引脚报告 |
| `impl/pnr/uart_bridge.fs` | SRAM 位流 |

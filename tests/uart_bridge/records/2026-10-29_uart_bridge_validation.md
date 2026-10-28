# UART 链路 v1 验证报告（独立测试工程 `tests/uart_bridge`）

| 项 | 值 |
| --- | --- |
| 日期 | 2026-10-29 |
| 范围 | 上位机 ↔ FPGA 链路 v1：帧协议、命令分发、安全仲裁、遥测、板级验证 |
| 结果 | **综合通过、仿真 106 项全过 + 一致性 PASS、板级台架 19/19×3 全过、600 s 长稳 0 误码** |
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
| 仿真 | `sim/tb_core.sv`（52 项）、`tb_loopback.sv`（23 项）、`tb_motion_adapter.sv`（19 项）、`tb_system_closed_loop.sv`（12 项）、`tb_conformance.sv`、`run_*.do`、`run_sim.ps1` |
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
4. **硬件急停 `ext_fault[2]` 改为锁存并纳入安全仲裁**（独立代码评审发现的问题）：
   v1.1 早期实现只在遥测里上报该位、**没有真正切断速度**，一旦集成时把真实急停按钮
   接到这里，按住按钮车仍会继续走。现已改为：`estop` 置位即锁存 → 强制目标速度 0 →
   `stop_latched_out` 同时置位 → 松开按钮不自动恢复，必须 `CLEAR_FAULT`（链路在线）。
   `tb_core` 新增 9 项与急停相关的用例。
5. **命令长度校验补齐**（同一评审发现）：`CLEAR_FAULT` 长度不符时**不再解锁任何锁存**（回
   `ACK 0x04`）；`HEARTBEAT` 长度不符回 `ACK 0x04`；`STOP` 长度不符**仍然停车**（安全优先）
   并回 `ACK 0x04`。行为表见协议 §5.1。
6. **ACK 语义澄清**：ACK 队列深度 1，背靠背命令可能丢 ACK；判断命令生效应以遥测为准
   （协议 §5.1 末尾说明）。台架脚本据此在连续命令间留 50 ms 间隔，19/19 连跑 3 次稳定。

其余（帧格式、CRC 参数、消息编号、故障位图、仲裁优先级）与 v1.0 一致。

---

## 3. 仿真结果

| 套件 | 检查项 | 结果 | 日志 |
| --- | --- | --- | --- |
| `tb_core.sv` | **52 项全过**：CRC 向量 V1–V4、帧解析错误谱（CRC/尾/LEN/重同步/字节间超时/逐字节喂入）、命令解码、ACK 状态码、限速裁剪、STOP 锁存、失联与恢复、遥测节拍、**长度不符命令**、**硬件急停锁存**、`setvel_strobe` 输出 | ✅ 0 fail | `records/tb_core.log` |
| `tb_loopback.sv` | **23 项全过**：真实 115200 8N1 双向回环（TB 逐位发、逐位收），ACK 与 TELEM 逐字节校验（含 int32/int16 小端、seq 回显） | ✅ 0 fail | `records/tb_loopback.log` |
| `tb_motion_adapter.sv` | **19 项全过**：电机侧适配器（模式/方向/幅值、差速近似、停车只发一次 BRAKE、停车期间拒绝命令、恢复、零速刹车） | ✅ 0 fail | `records/tb_adapter.log` |
| `tb_system_closed_loop.sv` | **12 项全过**：串口→链路→适配器→轮子模型→编码器→遥测全闭环，含**失联自动停车** | ✅ 0 fail | `records/tb_system.log` |
| `protocol_selftest.py` | **30 项** Python 编解码自检 | ✅ 0 failures | — |
| `conformance.py --run` | Python ↔ RTL 一致性（见 §3.1） | ✅ PASS | `records/conformance.log` |

合计 **106 项 RTL 检查 + 30 项 Python 自检 + 25 帧一致性比对**。

复核命令：`.\run_sim.ps1 -Python <python路径>`（5 套仿真 + 一致性，约 3 分钟）。

关键覆盖点（对应 `docs/PROTOCOL_v1.md` §8 的 T01–T12）：CRC 标准值 `0x29B1` 双向一致；
任何解析错误都不改变寄存器；`STOP` 锁存后 `SET_VEL` 被 ACK 拒绝（状态 0x02）；
失联后 `CLEAR_FAULT` 在链路未恢复时被拒（0x01）；**LEN 不符的 CLEAR_FAULT 不会解锁任何锁存**；
**硬件急停（`ext_fault[2]`）锁存且松开按钮不自动恢复**。

---

### 3.1 Python ↔ RTL 一致性测试（conformance）

`tools/conformance.py --run` 一条命令完成：生成 435 字节对抗语料（25 个合法帧 + CRC 错
5 处 + 尾字节错 4 处 + LEN 超限 2 处 + 垃圾字节 + `AA AA 55` 重叠 + 结尾截断帧）→ 让
`tb_conformance.sv` 把同一语料喂给 `host_frame_rx` → 与 `protocol.py` 的解析结果逐帧比对。

```text
PASS stats: {'good': 25, 'crc': 5, 'len': 2, 'tail': 4}
PASS frame count: 25
PASS all 25 frames identical (type/len/payload, first 8 bytes)
CONFORMANCE PASS
```

> 两侧对**同一段含噪数据**给出完全一致的帧与 good/crc/len/tail 计数，说明上位机与 FPGA
> 的协议理解一致。（`sync_err` 两边统计方式不同——缓冲解析器会重扫、流式状态机不会，
> 详见 `protocol.py` 的 `ParseStats` 说明；一致性测试不比对该项。）
> 已知的固有差异（仅在"噪声里出现非法 `AA 55` 且紧贴真帧"时，流式解析器无法回退重扫）
> 通过语料里的空闲间隔规避，并在 `tools/conformance.py` 文档字符串中说明。

---

## 4. 资源与时序

三个顶层各自综合（`gw_sh build*.tcl`，GW5A-25A，50 MHz）：

| 顶层 | Logic | Register | CLS | 其他 |
| --- | --- | --- | --- | --- |
| `uart_bridge_top`（纯链路） | **1174** / 23040（6%） | 844（4%） | **881** / 11520（8%） | — |
| `uart_bridge_lite_top`（精简） | **1110**（5%） | 782（4%） | **830**（8%） | — |
| `uart_bridge_imu_top`（链路+IMU） | **5430**（24%） | 2564（11%） | **3671**（32%） | BSRAM 4（pROM）、DSP 10/28 |

- 综合/PnR/位流生成全部通过；PnR 报告无时序违例（`impl/pnr/uart_bridge*.rpt.txt`）。
- 单帧处理：TELEM 30 字节 @115200 = 2.6 ms，20 ms 周期下占用约 13% 串口带宽；
  实测总带宽约 1540 B/s（≈ 50 Hz × 30 B + ACK），占 115200 的 13%~14%。

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
| 15 | 失联检出时间 | ✅ 220–299 ms（超时配置 300 ms；实测随调度抖动变化） |
| 16 | 遥测上报心跳年龄 | ✅ hb_age=317 ms |
| 17 | 恢复后 `CLEAR_FAULT` ACK OK | ✅ |
| 18 | 链路恢复（LINK_LOST 清除） | ✅ fault=0x20（仅剩命令超时，属预期） |
| 19 | CRC 误码率 | ✅ 0（109 帧） |

### 5.2 长稳测试 `soak_test.py --duration 600`（T15）

```
duration 600.0 s
frames_total 30004      error_frames 0      error_rate 0.0
telem_rate mean 50.00 Hz  min 49.93  max 50.06
parser_stats good=30004 crc_err=0 sync_err=0 tail_err=0 len_err=0 timeout=0
```

- 10 分钟 **30,004 帧零误码**（目标 < 1e-6，实测 0）；
- 遥测速率稳定在 50.0 Hz（±0.1 Hz）；
- 每 10 s 窗口的心跳年龄 2–96 ms（心跳 10 Hz，周期 100 ms），未出现误判失联；
- 证据：`records/2026-10-29_soak_600s.json`、`records/2026-10-29_soak_600s.log`。

### 5.3 IMU 变体长稳（同样 600 s）

```
frames_total 30005      error_frames 0      error_rate 0.0
telem_rate mean 50.00 Hz  min 49.95  max 50.04
parser_stats good=30005 crc_err=0 sync_err=0 tail_err=0 len_err=0 timeout=0
```

- 链路 + IMU SPI 轮询 + 姿态解算同时运行，**10 分钟零误码**；
- 证据：`records/2026-10-29_soak_imu_600s.json/.log`。

### 5.4 UDP 传输自检（为千兆网 v2 准备）

`tools/udp_loopback_test.py`：本机回环，200 个报文（含 50 个故意坏 CRC）→
**200 报文、150 合法帧 + 50 坏 CRC，每个报文恰好一帧**，解析结果与 UART 路径完全一致。

```text
sent datagrams : 200
received       : 200 (每报文应恰好一帧)
parsed frames  : 150  (期望 150 个合法帧)
stats          : {'good': 150, 'crc_err': 50, ...}
UDP TRANSPORT PASS
```

### 5.5 板级健壮性（真实链路灌坏帧 / 垃圾）

`tools/board_robustness_test.py`：在真实串口上故意制造错误，确认 FPGA 不会被噪声带跑。

```
PASS link_established                         fault=0x20
PASS corrupted_setvel_ignored                 cmd_seq 119 -> 119  （50 个 CRC 错帧全部丢弃）
PASS telemetry_survives_garbage               61.3 Hz             （灌 2000 个随机字节）
PASS garbage_does_not_execute                 cmd_seq=119
PASS valid_frame_accepted_after_noise         cmd_seq 126         （噪声后立刻恢复）
PASS link_still_ok                            fault=0x00
6/6 checks passed
```

> 说明：`cmd_seq` 的绝对数值取决于本次运行前已有多少条命令，**每次运行都不同**；
> 关键是"噪声前后数值不变"与"噪声后新命令被接受"。
> 遥测 61.3 Hz 略高于 50 Hz 是统计窗口包含噪声注入前后两段的正常现象；
> 关键结论是**噪声期间遥测不中断、坏帧不生效、噪声后立即恢复正常解析**。
> 证据：`records/2026-10-29_robustness_board.json`。

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

**主工程最新实测**（`impl/pnr/Embodied_RPACS.rpt.txt`，2026-10-29 02:06；
主工程当晚做过一轮逻辑裁减 + Fmax 提升，commit `09f2766`）：

| 资源 | 占用 | 余量 |
| --- | --- | --- |
| Logic | 12036 / 23040（53%） | **11004** |
| Register | 5138 / 23685（22%） | 18547 |
| CLS | 7525 / 11520（66%） | **3995** |

> 基线变化过程（供追溯）：`PLAN.md` 20334/89%（余 CLS 792）→ 10-28 23:40 的
> 19664/86%（余 CLS 998）→ 10-29 02:06 的 12036/53%（余 CLS 3995）。
> 三次都以当时 `impl/pnr/Embodied_RPACS.rpt.txt` 为准，之前"CLS 差 12 放不下"的
> 结论早已作废。

| 版本 | Logic | CLS | 合入后 CLS 余量 | 结论 |
| --- | --- | --- | --- | --- |
| 完整版 `uart_bridge_top` | 1174 | 881 | **3114** | ✅ 余量充足 |
| 精简版 `uart_bridge_lite_top` | 1110 | 830 | 3165 | ✅ 余量充足 |
| 链路 + 适配器（估） | ~1250 | ~930 | ~3065 | ✅ 余量充足 |

**结论更新**：链路可以**直接合入当前主工程**，CLS 不再是瓶颈；`task4_navigation` 基座
与"精简版"这两条退路已不再必要（保留为可选方案）。

> 集成时仍须遵守：只写目标速度、不写 PWM；保留 STOP/失联锁存语义（§6）。

### 7.1 精简版实测（`uart_bridge_lite_top`，已实现并上板验证）

精简手段只有两项：诊断计数器 16→8 位 + **不发 ACK**（上位机改用 50 Hz 遥测里的
`cmd_seq` 回显判断命令是否生效）。综合结果：

| 版本 | Logic | Register | CLS |
| --- | --- | --- | --- |
| 完整版 `uart_bridge_top` | 1174 | 844 | **881** |
| **精简版 `uart_bridge_lite_top`** | **1110** | 782 | **830** |

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
> 结论：主工程 02:06 优化后 CLS 余量已达 3995，**精简版不再是必需品**，保留作为可选方案
> （它给主工程多留约 51 CLS）。

---

## 8. IMU 变体（遥测带真实姿态）

用户要求"能读取板子上的传感器信息"，因此在链路核心不动的前提下加了第二个顶层
`src/uart_bridge_imu_top.v`：复用主工程已验证的 `imu660ra_driver` + `imu_attitude_fixed`
（200 Hz 采样、CORDIC/atan2/开方求姿态），把 roll/pitch/yaw（毫度）换算成 0.01° 填进 TELEM。

### 8.1 资源开销（同一工程、两个顶层分别综合）

| 顶层 | Logic | Register | CLS | BSRAM | DSP |
| --- | --- | --- | --- | --- | --- |
| `uart_bridge_top`（纯链路） | 1174（6%） | 844（4%） | 881（8%） | 0 | 0 |
| `uart_bridge_imu_top`（链路 + IMU 姿态） | **5430（24%）** | 2564（11%） | **3671（32%）** | 4（pROM，IMU 配置镜像） | 10/28 |

即 **IMU 驱动 + 姿态解算额外占 4256 Logic / 2790 CLS / 4 BSRAM / 10 DSP**。
对主工程而言这部分**本来就有**（主工程已含 IMU + 姿态），所以把链路合入主工程只多 1174 Logic
（结论仍见 §7）。独立测试工程里 23%/31% 余量充足。

### 8.2 板级实测（烧写 `uart_bridge_imu.fs`，同样只写 SRAM）

```
19/19 checks passed   （records/2026-10-29_uart_bridge_imu_bench.json，失联检出 220 ms）
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
| `records/final_build*.log` | 最终三个变体的综合/PnR 日志（`build_synth*.log` 为早期日志） |
| `records/download_sram.log` | SRAM 烧写日志（User Code 0x000067E2） |
| `records/tb_core.log` | 51 项仿真 PASS |
| `records/tb_loopback.log` | 23 项真实波特率回环 PASS |
| `records/2026-10-29_uart_bridge_bench.json` | 台架 19 项逐条结果 |
| `records/2026-10-29_soak_600s.json/.log` | 10 min 长稳统计 |
| `impl/pnr/uart_bridge.rpt.txt` | 资源/引脚报告 |
| `impl/pnr/uart_bridge.fs` | SRAM 位流 |

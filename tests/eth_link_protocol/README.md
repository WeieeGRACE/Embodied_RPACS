# 链路协议 v2：以太网（UDP）版本

> 目标：把已验证的链路协议（`tests/uart_bridge`，帧格式/CRC/命令/安全/遥测全部复用）搬到
> **千兆以太网**上跑：一个 UDP 报文 = 一个协议帧（`docs/PROTOCOL_v1.md` §7 的约定）。
>
> **独立测试目录**：主工程零改动；链路核心直接引用 `../uart_bridge/src`（单一事实来源）。

---

## 1. 这是什么

```
Pi 192.168.0.1:5001  ──UDP──>  FPGA 192.168.0.2:5000
      ^                              │
      └────── UDP 遥测/ACK <─────────┘   (目的 MAC 用广播，免 ARP 配置)
```

FPGA 侧结构：

```
RGMII PHY ── rgmii_to_gmii ──> eth_udp_rx_gmii ── payload 流 ──┐
                                                              ▼
                                              eth_link_bridge（本工程新增）
                                        ┌────────────────────────────────┐
   UDP payload 字节 ── 降速(每2拍1字节) ──> host_frame_rx → uart_bridge_core
                                        │        ↓ 指令/安全/遥测组帧        │
   UDP 发送 <── tx_start/tx_len/逐字节取数 ── 整帧缓存 ← frame_tx_engine    │
                                        └────────────────────────────────┘
                                                  │
                              eth_udp_tx_gmii ──> gmii_to_rgmii ──> PHY
```

即：**协议层完全不动**，只把 UART 的 `(byte, valid)` 换成 UDP payload 的字节流。

## 2. 关键参数

| 项 | 值 |
| --- | --- |
| 位流 | `impl/pnr/eth_link_top.fs`，SHA256 `AAA9CEC21470946A1956554C1BE8DD7A6E69AF0259D53B2A3DEB064D30C5C2B5` |
| 资源 | Logic 1511/23040（7%）、Register 982、CLS 1142/11520（10%）、IOLOGIC 11（RGMII DDR） |
| FPGA 网络 | MAC `00:0a:35:01:fe:c0`、IP `192.168.0.2`、UDP **5000** |
| 回包目的 | IP `192.168.0.1`、UDP **5001**、MAC **广播 FF:FF:FF:FF:FF:FF** |
| 主时钟 | 由 PHY 的 `rgmii_rx_clk`(125 MHz) 经 PLL 派生（与 `eth_udp_link` 相同，PLL 参数已按 C2/I1 重算） |
| 帧格式 | 与 UART 版完全一致：`AA 55 ｜ TYPE ｜ LEN ｜ PAYLOAD ｜ CRC16 ｜ 0D` |

> 为什么回包用广播 MAC：FPGA 侧没有 ARP 响应器，广播省掉"知道主机 MAC"这一步。
> 主机侧仍需要**静态 ARP**（把 192.168.0.2 指到 FPGA MAC），因为主机→FPGA 方向要发单播。

## 3. Pi 侧准备（本次已在板子上配好）

```bash
sudo nmcli connection add type ethernet ifname eth0 con-name ethfpga \
    ipv4.method manual ipv4.addresses 192.168.0.1/24 ipv4.never-default yes ipv6.method disabled
sudo nmcli connection up ethfpga
sudo ip neigh replace 192.168.0.2 lladdr 00:0a:35:01:fe:c0 dev eth0 nud permanent
```

检查：`ip -4 addr show eth0`（应有 192.168.0.1/24）、`cat /sys/class/net/eth0/speed`（1000）。

## 4. 上电后怎么测（三步）

**① PC：烧位流（SRAM，掉电失效；车断电后要重烧）**

```powershell
cd tests\eth_link_protocol
programmer_cli --device GW5A-25A --run 2 `
    --fsFile (Resolve-Path impl\pnr\eth_link_top.fs) `
    --cable "Gowin USB Cable(FT2CH)"
```

**② 确认网线**：FPGA RJ45 ↔ Pi eth0。

**③ Pi：跑验收**

```bash
cd ~
python3 eth_link_test.py                 # 6 项预检 + 6 项命令测试
python3 eth_link_test.py --duration 10 --json eth_link.json
```

判定项（脚本自动给 PASS/FAIL）：

| # | 检查 | 期望 |
| --- | --- | --- |
| P1 | UDP socket 建立 | 通过 |
| P2 | 3 s 内收到遥测 | 有帧 |
| P3 | 遥测速率 | 40~60 Hz |
| P4 | CRC/尾/LEN 错 | 0 |
| P5 | 心跳年龄 | ≤150 ms |
| P6 | 故障字节 | 可读 |
| C1 | CLEAR_FAULT → ACK | OK |
| C2/C3 | SET_VEL → ACK | OK 且 seq 回显 |
| C4 | 遥测 `cmd_seq` 回显 | 与下发一致 |
| C5/C6 | STOP → ACK + 遥测 STOP 位 | 通过 |

预期：**12/12 PASS**。

## 5. 已完成的验证（无需板子）

| 项 | 结果 |
| --- | --- |
| 桥接层仿真 `sim/tb_eth_link_bridge.sv` | **10/10 PASS**：上电即出 TELEM、格式/CRC/长度正确、心跳建链、SET_VEL→ACK+seq 回显、遥测周期正确、无溢出 |
| 综合 | 位流生成通过（见 §2） |
| 以太网物理层（前一步 `eth_udp_link`） | UDP 回环 0% 丢包、RTT 0.066 ms、3692 pps（详见 `tests/eth_udp_link`） |

仿真命令：

```powershell
cd tests\eth_link_protocol
D:\app\modesilm\win64\vsim.exe -c -do "do sim/run_bridge2.do"
```

## 6. 已知限制

1. **一次只有一帧在发**：`eth_link_bridge` 用单缓冲（128 B），收到新帧时若上一帧还没发完会短暂反压；
   对 50 Hz 遥测（30 B/帧）绰绰有余，高频突发场景需换 FIFO（后续可扩）。
2. **无 ARP 响应**：主机必须静态 ARP（已配）。后续自研可加 ARP 应答。
3. **回包用广播**：同一网段多台机器都会收到遥测；正式版建议改成单播（需要主机 MAC 参数或 ARP）。
4. 复位用板上 KEY0（B16，低有效）。
5. Windows 上若 `--local-port 5001` 绑定失败（系统保留端口），请在 Pi 上跑（Linux 无此限制）。

## 7. 目录

```text
tests/eth_link_protocol/
├── README.md
├── build.tcl                      综合脚本（top = eth_link_top）
├── src/
│   ├── eth_link_bridge.v          UDP payload <-> 协议帧（新写的核心，已仿真）
│   ├── eth_link_top.v             顶层：PHY/UDP + 桥接层 + PLL + 状态灯
│   ├── eth_udp_gmii/              厂商 UDP 收发（原样）
│   ├── gmii_rgmii_gmii/           厂商 RGMII<->GMII（原样）
│   ├── gowin_pll/                 PLL（参数已按 C2/I1 重算）
│   └── eth_udp_lookback_rgmii.cst RGMII 引脚 + 复位/LED
├── sim/tb_eth_link_bridge.sv      桥接层自检（10 项）
├── sim/run_bridge2.do             ModelSim 运行脚本
├── tools/eth_link_test.py         Pi 侧验收工具（复用 uart_bridge/tools）
└── records/                       仿真日志、构建日志
```

## 8. 后续（等这次跑通再做）

1. 把 `eth_link_bridge` 换成 FIFO 版支持并发多帧；
2. 加 ARP 响应器，去掉静态 ARP；
3. 遥测里的 IMU 姿态/编码器接真实信号（现在桥接层给核心的是 0，与 `uart_bridge_top` 的纯链路版一致）；
4. 移植到主工程时按 `tests/uart_bridge/docs/INTEGRATION_GUIDE.md` 的路线，把 UART 版链路替换成 ETH 版。

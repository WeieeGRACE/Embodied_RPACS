# 千兆以太网（RGMII）链路测试

> 目标：验证 **Orange Pi eth0 ↔ FPGA 板 RJ45** 的 1 Gbps 链路可用，为后续把链路协议
> 搬上以太网（`docs/PROTOCOL_v1.md` §7 的 v2 约定）打基础。
>
> **独立测试目录**：只用厂商例程位流 + Pi 侧测试脚本，不改主工程、不改 `tests/uart_bridge`。

---

## 1. 现状与结论（2026-10-29 22:1x）

| 项 | 状态 |
| --- | --- |
| 物理链路 | ✅ Pi eth0 与 FPGA RJ45 已 up，**1000 Mb/s / full duplex** |
| Pi 侧网络 | ✅ 已配置静态 `192.168.0.1/24`（NM 连接名 `ethfpga`）+ 静态 ARP |
| 测试位流 | ⏳ 使用厂商预编译位流（见 §2），**待烧写**（当前 JTAG 下载线卡死，等重插） |
| 测试脚本 | ✅ `tools/udp_echo_test.py`（含本机自检 PASS） |

---

## 2. 测试位流

厂商例程 `eth_udp_loopback_rgmii`（小梅哥 ACG525 例程源码）：
把收到的 UDP 负载**原样回环**发给发送方，用来验证 PHY/RGMII/UDP 收发通路。

| 项 | 值 |
| --- | --- |
| 文件 | `impl/pnr/vendor_eth_udp_loopback_rgmii.fs`（厂商 2023-08-18 预编译） |
| SHA256 | `EDF446344BC540274055F56EBE42C985A5685E1037209C287776DCB50AA7E464` |
| 资源 | Logic 1469/23040（7%）、Register 1113、CLS 1236/11520（11%） |
| FPGA MAC | `00:0a:35:01:fe:c0` |
| FPGA IP / 端口 | **192.168.0.2** / UDP **5000** |
| 时钟 | 无外部时钟输入，由 PHY 的 `rgmii_rx_clk`（125 MHz）派生（例程设计如此） |

⚠️ 该例程**不实现 ARP**（只处理 ETH_type 0x0800），所以 Pi 侧必须手工填静态 ARP 表。

> 本目录的 `build.tcl` 保留用于将来重新综合，但**当前 Gowin EDA V1.9.12 对该例程的
> PLL 参数报错**（`FCLKIN/IDIV_SEL` PFD 超范围 125 MHz > 87.5 MHz）。要重建需要把
> `src/gowin_pll/gowin_pll.v` 的 `IDIV_SEL` 改成 2（PFD≈41.7 MHz）或换用 IP 向导重配。
> 现阶段直接用厂商预编译位流即可。

## 3. 接线

```
FPGA 板 RJ45  ──────────────  Orange Pi eth0（1 Gbps，直连，无需交换机）
```

板上 `eth_rst_n`/`eth_mdc`/`eth_mdio` 由例程直接拉高（不做 PHY 配置，依赖 PHY 默认自协商）。

## 4. Pi 侧网络配置（已应用，重装系统后照做）

```bash
sudo nmcli connection add type ethernet ifname eth0 con-name ethfpga \
    ipv4.method manual ipv4.addresses 192.168.0.1/24 ipv4.never-default yes ipv6.method disabled
sudo nmcli connection up ethfpga
# 例程不回 ARP，手工填表：
sudo ip neigh replace 192.168.0.2 lladdr 00:0a:35:01:fe:c0 dev eth0 nud permanent
```

> `ipv4.never-default yes` 很关键：否则 192.168.0.0/24 的默认路由可能抢走 WiFi 上网。

## 5. 运行测试

```powershell
# PC：烧厂商位流（SRAM，掉电失效）
cd tests\eth_udp_link
programmer_cli --device GW5A-25A --run 2 `
    --fsFile (Resolve-Path impl\pnr\vendor_eth_udp_loopback_rgmii.fs) `
    --cable "Gowin USB Cable(FT2CH)"
```

```bash
# Pi
cd ~/rpacs_link_integration   # 或把本目录 tools/ 拷过去
python3 tools/udp_echo_test.py --count 200 --interval 0.01 --json records/udp_echo.json
```

判定（脚本内置）：

| 指标 | 目标 |
| --- | --- |
| 丢包率 | ≤ 1% |
| RTT 中位数 | < 5 ms |
| 负载校验 | 回环内容与发送一致（脚本校验 magic+序号） |

## 6. 本机自检（无硬件）

```powershell
python tools\udp_echo_test.py --selftest
```
本地起一个 UDP 回环服务端，验证统计逻辑（当前 **SELFTEST PASS**）。

## 7. 目录

```text
tests/eth_udp_link/
├── README.md                  本文
├── build.tcl                  重新综合用（当前 IDE 版本需先修 PLL，见 §2）
├── src/                       厂商 RGMII/UDP 例程源码（原样拷贝，仅作参考/重建）
│   ├── eth_udp_gmii/          UDP 收发、CRC32、IP 校验和
│   ├── gmii_rgmii_gmii/       RGMII↔GMII 转换
│   ├── gowin_pll/             125 MHz PLL
│   ├── eth_data_buf/          payload FIFO
│   ├── eth_msg_buf/           报文头 FIFO
│   ├── top/                   eth_udp_loopback_rgmii.v
│   └── eth_udp_lookback_rgmii.cst    RGMII 引脚约束
├── tools/udp_echo_test.py     Pi 侧回环测试 + 本机自检
├── impl/pnr/vendor_eth_udp_loopback_rgmii.fs   待烧的厂商位流
└── records/                   运行记录
```

## 8. 下一步（把链路协议搬到以太网）

协议层已经与传输解耦（`tests/uart_bridge/tools/transport.py` 的 `UdpTransport`，
一个 UDP 报文 = 一个完整帧）。移植需要：

1. 在 FPGA 侧增加一个「UDP 负载 → 字节流」适配器，接到 `host_frame_rx` 的
   `(byte, valid)` 输入（注意 `docs/PROTOCOL_v1.md` §5.2 要求的字节间隙）；
2. 遥测 `frame_tx_engine` 的输出改为写入 UDP payload（每帧一个报文）；
3. 用本目录的例程先验证 PHY 通路（当前任务），再替换 payload 逻辑。

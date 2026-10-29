# 千兆以太网（RGMII）链路测试

> 目标：验证 **Orange Pi eth0 ↔ FPGA 板 RJ45** 的 1 Gbps 链路可用，为后续把链路协议
> 搬上以太网（`docs/PROTOCOL_v1.md` §7 的 v2 约定）打基础。
>
> **独立测试目录**：只用厂商例程位流 + Pi 侧测试脚本，不改主工程、不改 `tests/uart_bridge`。

---

## 1. 现状与结论（2026-10-29 22:3x，**已通过**）

| 项 | 状态 |
| --- | --- |
| 物理链路 | ✅ Pi eth0 ↔ FPGA RJ45，**1000 Mb/s / full duplex** |
| Pi 侧网络 | ✅ 静态 `192.168.0.1/24`（NM 连接 `ethfpga`）+ 静态 ARP |
| 测试位流 | ✅ **自建** `impl/pnr/eth_udp_link.fs`（厂商例程按 C2/I1 硅片重编） |
| 回环测试 | ✅ 100/100 ~ 3000/3000 全回包，**0% 丢包**，RTT 中位 **0.066 ms** |
| 吞吐 | ✅ 3692 pps（128 B ≈ 3.8 Mbps）、719 pps（1400 B ≈ 8 Mbps），均为 Pi 侧 Python 上限 |
| 记录 | `records/2026-10-29_B2_千兆以太网链路验证.md` |

**关键坑（务必记住）**：厂商预编译的 `vendor_eth_udp_loopback_rgmii.fs` 是给
**GW5A-LV25UG324ES（ES 硅片）** 编的，PLL 参数在我们的 **C2/I1** 上非法 →
PLL 不锁 → 全设计无时钟 → **FPGA 一个包都不发**（Pi 侧 `rx_packets` 恒为 0）。
解决办法只有重新综合：PLL 改成 `IDIV_SEL=2 / FBDIV_SEL=2 / MDIV_SEL=7 / ODIV0_SEL=7`
（PFD 62.5 MHz、VCO 875 MHz、CLKOUT0 125 MHz），其余 RTL 不动。详见测试记录 §2。

---

## 2. 测试位流

把厂商例程 `eth_udp_loopback_rgmii`（收到的 UDP 负载原样回环）**按我们的器件重编**：

| 项 | 值 |
| --- | --- |
| 位流 | `impl/pnr/eth_udp_link.fs` |
| SHA256 | `0D11273EE652B7A1C4F9501D3667DAD0876AD159B7C8DCED3A2DB6ECC813E262` |
| 资源 | Logic 1105/23040（5%）、Register 872、CLS 980/11520（9%） |
| FPGA MAC / IP / 端口 | `00:0a:35:01:fe:c0` / **192.168.0.2** / UDP **5000** |

重建命令（本目录）：

```powershell
& 'D:/Gowin/Gowin_V1.9.12_x64/IDE/bin/gw_sh.exe' build.tcl
# -> impl/pnr/eth_udp_link.fs
programmer_cli --device GW5A-25A --run 2 `
    --fsFile (Resolve-Path impl/pnr/eth_udp_link.fs) --cable "Gowin USB Cable(FT2CH)"
```

> 对照用：`impl/pnr/vendor_eth_udp_loopback_rgmii.fs`（厂商 ES 版，**在本板不可用**：
> PLL 不锁，无任何输出）。

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
# PC：烧重编位流（SRAM，掉电失效；每次 FPGA 断电后都要重烧）
cd tests\eth_udp_link
programmer_cli --device GW5A-25A --run 2 `
    --fsFile (Resolve-Path impl\pnr\eth_udp_link.fs) `
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

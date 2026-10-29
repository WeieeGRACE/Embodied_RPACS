# 以太网链路（v2）移植到主工程的指南

> 前提：`tests/eth_link_protocol`（协议 over UDP）与 `tests/eth_motion_test`（电机输出）
> 都已在板上验证通过，仿真 10/10 与 15/15。本文说明把它们并入主工程时的步骤、资源与风险。
>
> **未执行**：按用户要求，测试阶段只改 `tests/`；移植要等动车测试完成、用户确认后再做。

---

## 1. 要加什么

主工程目前是「UART 链路 + 主控 FSM + 传感器 + OLED + ASR」。以太网版本需要新增：

| 新增模块 | 来源 | 作用 |
| --- | --- | --- |
| `eth_udp_rx_gmii.v` / `eth_udp_tx_gmii.v` / `crc32_d8.v` / `ip_checksum.v` | 厂商例程（已拷进 `tests/eth_link_protocol/src/eth_udp_gmii/`） | RGMII 上的 UDP 收发 |
| `rgmii_to_gmii.v` / `gmii_to_rgmii.v` | 同上 | RGMII ↔ GMII |
| `gowin_pll`（已按 C2/I1 重算） | `tests/eth_link_protocol/src/gowin_pll/` | 125 MHz 时钟（**注意：厂商参数在 C2/I1 上不锁，必须用重算版**） |
| `eth_link_bridge.v` | 测试工程新写 | UDP payload ↔ 协议帧（含接受 FIFO、整帧发送缓冲） |
| 链路核（`host_frame_rx` 等 7 个文件） | `tests/uart_bridge/src/` | 与 UART 版完全相同的协议层 |

如果要保留电机控制，还需把 `eth_motion_bridge.v` 的输出接到主工程已有的
`tb6612_driver_interface`（主工程本来就有，直接复用）。

## 2. 引脚与时钟

| 信号 | 引脚 | 说明 |
| --- | --- | --- |
| RGMII TXD[3:0] | F1, F2, D1, D2 | 板载 PHY，原厂约束 |
| RGMII TX_CLK / TXEN | C1 / C2 | |
| RGMII RXD[3:0] / RXDV / RX_CLK | F5, F6, G1, G3 / E1 / L5 | |
| ETH_MDIO / MDC / RST_N | E4 / D3 / G6 | 例程直接拉高，不做 MDIO 配置 |
| `clk_50m` | T9 | **主工程用它做系统时钟**；以太网版主时钟来自 PHY 的 `rgmii_rx_clk`(125 MHz) |

⚠️ **时钟域**：主工程的 FSM/传感器都在 50 MHz，链路桥接在 125 MHz（PHY 派生）。
移植时要么：
- 在 125 MHz 域完成协议收发，再用异步 FIFO 与 50 MHz 域交换命令/遥测；或
- 把整个主控搬到 125 MHz（风险大，不建议）。

推荐前者：在 `eth_link_bridge` 两侧各加一级异步 FIFO（命令下行 / 遥测上行各一个）。

## 3. 资源预算（实测）

| 设计 | Logic | Register | CLS |
| --- | --- | --- | --- |
| 主工程（无 logger，post-acceptance 基线） | 13518（59%） | 6736 | 8328（72%） |
| 以太网链路（`eth_link_top`） | 1511（7%） | 982 | 1142（10%） |
| 电机测试位流（`eth_motion_top`） | 1928（8%） | 1116 | 1401（12%） |
| **估算：主工程 + 以太网链路 + 电机** | **≈15500–16000（67–70%）** | ≈7700 | **≈9700–10000（84–87%）** |

CLS 会接近 90%，但仍在 11520 以内；上板前务必重新跑时序报告（主工程已有
`tools/check_system_reports.ps1` 做这项检查）。

## 4. 移植步骤（建议顺序）

1. **引脚让位**：`post-acceptance-no-logger` 分支已删掉调试 UART（V8/U8 释放）——
   如果主工程还要用 V8/U8，注意别和以太网引脚冲突（两者不同球，不冲突）。
2. 把上表 RTL 拷进主工程 `src/link/`（不建议直接引用 tests 路径）。
3. 顶层加 RGMII 端口、PLL、桥接层；先用**只读**配置验证：遥测回传 + 命令解析，
   电机输出保持 0（等价于 `eth_link_top`）。
4. 验证通过后，再把 `target_v_left/right` 接到主工程 `motion_control_and_mode_fsm`
   或直接接 `tb6612_driver_interface`（先低速、轮子离地）。
5. 保留 UART 链路作为回退：两套链路可以同时存在，用 CMake/tcl 参数切换。

## 5. 已知坑（都踩过）

| 坑 | 现象 | 处理 |
| --- | --- | --- |
| 厂商 `.fs` 直接烧 | FPGA 一个包不发 | 厂商位流是 ES 硅片编的；必须按 C2/I1 重算 PLL（`IDIV=2/FBDIV=2/MDIV=7/ODIV0=7`）后重编 |
| 无 ARP | Pi 发不出单播 | 静态 ARP（`ip neigh replace`）或自己实现 ARP 应答 |
| 回包目的 MAC | 不知道主机 MAC | 先用广播 MAC，稳定后改单播 |
| 心跳 300 ms | 相机负载下 330 ms 掉链 | 上位机 20 Hz 心跳，或 `HB_TIMEOUT_MS` 调 500 |
| LINK_LOST 锁存 | 恢复后仍拒绝命令 | 必须 `CLEAR_FAULT`（已写进工具） |
| 125 MHz 下 16 位分频 | 遥测不出 | `ms_div` 必须是 32 位（已修） |
| 单帧发送缓冲 | 高频遥测会反压 | 50 Hz 遥测无影响；高频需换 FIFO |

## 6. 验证清单（移植后逐项打勾）

- [ ] `eth_link_test.py` 12/12 PASS
- [ ] `eth_soak.py --duration 600` PASS（50 Hz、0 丢包、健康帧 ≥95%）
- [ ] `cam_eth_soak.py --duration 300` PASS（相机 8 fps 时链路不掉）
- [ ] `eth_fault_test.py` 全 PASS
- [ ] 主工程回归：`tools/run_legacy_regression.ps1` + `tools/check_system_reports.ps1`
- [ ] 轮子离地的电机方向/急停/断链保护（见 `tests/eth_motion_test/README.md` §3）

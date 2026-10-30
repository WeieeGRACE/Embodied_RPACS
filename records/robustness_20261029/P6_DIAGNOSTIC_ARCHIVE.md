# P6-21 诊断工程测试报告与归档记录（2026-10-30）

## 1. 归档决定

- 主交付固件在 2026-10-30 上板验收通过后，已按用户预先确认删除 USB 事件日志与调试串口
  （释放 V8/U8，42 个受约束引脚）。P6-21 诊断工程因此失去用途。
- 用户 2026-10-30 指示：**不删除，作为仓库历史存档保留好，并补齐测试报告归档**。
- 归档实现：
  - 源码/脚本留在 `tests/p6_uart_debug/`；
  - 主工程删日志前的源码快照 `frozen_src/robot_system_top.v`、`frozen_src/system_motion.v`
    （取自提交 `8b04f7d`）；
  - 两个已交付诊断位流 + 校验和 `archive/*.fs`、`archive/SHA256SUMS.txt`；
  - 可重建入口 `build_frozen_uart.tcl`、`build_frozen_distance.tcl`；
  - 说明文档 `tests/p6_uart_debug/ARCHIVE.md`。

## 2. 归档位流

| 变体 | 文件 | 大小 | SHA256 | Logic | CLS | 串口引脚 |
|---|---|---|---|---|---|---|
| 普通日志版（状态位 `D` 行） | `archive/Embodied_RPACS_p6uart.fs` | 6,443,628 B | `08DFDED3A43359D8E1237C03751E5D1978A97AFA958ED7C8276291FE706F399E` | 15910/23040 (70%) | 9612/11520 (84%) | P6-21 / L12 |
| 距离日志版（三路距离 `D` 行） | `archive/Embodied_RPACS_p6dist.fs` | 6,389,916 B | `EE00D3753180FB199EEA2CF433128374E4F77F1E450DF80FB5BEB1CA19C4EC10` | 15919/23040 (70%) | 9665/11520 (84%) | P6-21 / L12 |

两版均为 50 MHz、0 时序违例、`uart_tx` 约束在 L12；运动/避障逻辑与同期主固件一致，
仅调试串口引脚与 `debug_state` 负载不同。

## 3. 重建验证（归档源码可用性）

命令与日志：

```powershell
cd tests/p6_uart_debug
gw_sh build_frozen_uart.tcl      # 普通日志版 -> Embodied_RPACS_p6uart_frozen.fs
gw_sh build_frozen_distance.tcl  # 距离日志版 -> Embodied_RPACS_p6dist_frozen.fs
```

日志：`records/robustness_20261029/p6_frozen_uart_build.log`、
`records/robustness_20261029/p6_frozen_dist_build.log`（均 build exit 0）。

| 重建产物 | 大小 | 重建 SHA256 | 报告结果 |
|---|---|---|---|
| `Embodied_RPACS_p6uart_frozen.fs` | 6,443,628 B | `93B6A3C36F43C6D897204B94385532B66AE58CF74FB5047B5E0BA062E0EBEA8A` | Logic 15910/23040、CLS 9612/11520、Register 6746、BSRAM 9/56、Fmax 56.049 MHz、L12 |
| `Embodied_RPACS_p6dist_frozen.fs` | 6,389,916 B | `313CE6325D223FE500D993E92559D33B6BDAC50D4CE0B5698F1546CE0FB9E614` | Logic 15919/23040、CLS 9665/11520、Register 6757、BSRAM 9/56、Fmax 54.821 MHz、L12 |

资源/时序与归档位流一致；Gowin 位流不是逐字节可复现（重建 SHA 与归档 SHA 不同属正常），
**交付与取证以 `archive/` 中位流及其 SHA256 为准**。

## 4. 归档位流的上板使用记录

| 位流 | 上板时间 | 用途与证据 |
|---|---|---|
| `B3A02502…`（普通日志版，功能轮/含 SPIN 360+IMU 页+PASS 兜底，早于 400 ms 滤波） | 2026-10-29 20:36 | 定位 PASS 段 `BLOCKED reason=0x04`；证据 `records/robustness_20261029/board_wchlink_run_20261029_2046.log` |
| 同上 `B3A02502…` | 2026-10-29 22:52–22:59 | 7 次运行、5 次 PASS 停车、1 次完整；证据 `board_wchlink_live_2248.log`，分析 `board_run_20261029_2252_analysis.md` |
| `08DFDED3…`（普通日志版，加入 400 ms 前向滤波后的归档版） | 未再上板 | 仅为归档/重建一致性保留 |
| `EE00D375…`（距离日志版） | 2026-10-30 00:03–00:10 | 验收取证：25,216 条距离快照，5 轮完整绕障、0 故障，唯一停车为 SHIFT 段真实墙面 82 mm；证据 `board_p6dist_live.log`，分析 `board_run_20261030_0003_analysis.md` |

## 5. 完整性检查

```powershell
Get-FileHash tests/p6_uart_debug/archive/*.fs -Algorithm SHA256
Get-Content tests/p6_uart_debug/archive/SHA256SUMS.txt
python tools/audit_final.py     # 含两条归档位流校验（96 -> 98 项）
```

## 6. 限制与约定

- 归档工程**不再随主固件维护**：主 `src/` 已无 `uart_tx/uart_rx` 与 `debug_state/debug_valid/
  event_errors`，因此必须用 `frozen_src/` + `build_frozen_*.tcl` 重建，不能用历史 `build.tcl` /
  `build_distance.tcl` 直接构建。
- 归档位流只用于复现历史串口证据；交付演示请使用主位流（无调试串口，诊断看 OLED 面板
  `F/S/M/I` 码）。
- 若未来重新引入串口诊断，应新建独立分支/工程，不要修改归档内容。

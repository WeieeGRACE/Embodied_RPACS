# P6-21 诊断工程（已归档，2026-10-30）

本目录是主工程避障演示的 **P6-21 无线串口诊断变体**，已随主固件删除 USB 事件日志/调试串口
而停止维护，保留在仓库中作为历史诊断存档。主交付固件（`impl/pnr/Embodied_RPACS.fs`，
SHA256 `901D7394…AB14B`）**没有调试串口**，现场诊断请看 OLED 避障面板的 `F/S/M/I` 码；
只有需要复现 2026-10-29/30 的串口证据时才使用这里的归档位流。

## 两个变体

| 变体 | 用途 | 归档位流 | SHA256 | 资源 |
|---|---|---|---|---|
| 普通日志版 | `D` 行输出 route_state/reason/tof_reused 等状态位（原诊断格式） | `archive/Embodied_RPACS_p6uart.fs`（6,443,628 B） | `08DFDED3A43359D8E1237C03751E5D1978A97AFA958ED7C8276291FE706F399E` | Logic 15910/23040 (70%)、CLS 9612/11520 (84%)、Fmax 56.049 MHz |
| 距离日志版 | `D` 行输出 front/left/right 三路距离 + route_state + 20 mm 截断确认 | `archive/Embodied_RPACS_p6dist.fs`（6,389,916 B） | `EE00D3753180FB199EEA2CF433128374E4F77F1E450DF80FB5BEB1CA19C4EC10` | Logic 15919/23040 (70%)、CLS 9665/11520 (84%) |

两个变体的调试串口 TX 都在 **P6-21（GPIO18，FPGA 球 L12）**，接线：WCH-Link RX→P6-21、
GND→P6-12/P6-30，115200 8N1；其余运动/避障逻辑与主固件逐行相同（普通版对应功能版
`F5DD42E8…`，距离版对应同一套 RTL 仅换 `debug_state` 负载）。

> 历史沿革：普通日志版最初构建为 `B3A02502…`（2026-10-29 21:14，含 SPIN 360/IMU ATTITUDE/
> PASS 兜底），20:36 与 22:52 两次上板定位就是用该版本；加入 400 ms 前向滤波后重建为上面
> 归档的 `08DFDED3…`（未再上板）。距离日志版 `EE00D375…` 是 2026-10-30 00:03–00:10
> 验收取证的固件。

## 目录内容

- `src/robot_system_top_p6uart.cst`、`src/robot_system_top.sdc`、`src/imu/*.hex`、`src/ui/*.hex`：
  变体专用约束/初始化文件；
- `src/system_motion_dist.v`：距离版专用的 `system_motion` 副本（只改 `debug_state` 负载）；
- `frozen_src/robot_system_top.v`、`frozen_src/system_motion.v`：主工程删除日志/串口**之前**
  的源码快照（从提交 `8b04f7d` 提取），保证归档工程仍可独立重建；
- `archive/*.fs` + `archive/SHA256SUMS.txt`：两个已交付诊断位流及其校验和；
- `build.tcl` / `build_distance.tcl`：历史构建脚本（依赖当时的主 `src/`，现已不可直接使用）；
- `build_frozen_uart.tcl` / `build_frozen_distance.tcl`：**归档重建入口**（用 `frozen_src/`
  + 现行 `../../src/` 中未受影响的模块），输出 `Embodied_RPACS_p6uart_frozen.fs` /
  `Embodied_RPACS_p6dist_frozen.fs`；
- `tools/decode_debug_log_tolerant.py`：容忍时间戳前缀的旧格式解码器。

## 重建与采集

```powershell
cd tests/p6_uart_debug
& 'D:/Gowin/Gowin_V1.9.12_x64/IDE/bin/gw_sh.exe' build_frozen_uart.tcl      # 普通日志版
& 'D:/Gowin/Gowin_V1.9.12_x64/IDE/bin/gw_sh.exe' build_frozen_distance.tcl   # 距离日志版
```

Gowin 位流不是逐字节可复现的：重建只用于验证归档源码可用，交付/取证请以
`archive/` 中的位流与 SHA256 为准。

采集（只读，115200 8N1）：

```powershell
./tools/read_serial_log.ps1 -Port COM24 -Seconds 300 -Timestamped -OutFile <绝对路径>.log
python tests/p6_uart_debug/tools/decode_debug_log_tolerant.py <log> --changes-only   # 普通版
python tools/decode_distance_log.py <log> --runs --stop-context 8                   # 距离版
```

## 归档验证记录

- 两个归档位流的生成构建、资源/时序报告与 P6-21/L12 约束检查见
  `records/robustness_20261029/P6_DIAGNOSTIC_ARCHIVE.md`；
- 使用这两版采集的上板证据：`records/robustness_20261029/board_wchlink_run_20261029_2046.log`、
  `board_wchlink_live_2248.log`、`board_p6dist_live.log`，分析见
  `records/robustness_20261029/BOARD_RUN_ANALYSIS.md` 与
  `board_run_20261030_0003_analysis.md`。

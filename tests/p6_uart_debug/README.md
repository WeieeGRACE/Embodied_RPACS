# P6-21 无线串口调试位流（临时诊断工程）

用途：主工程避障演示需要让车自由跑动，板载 **USB-UART（CH9102F，Type-C）** 会被 USB 线拖住。
本工程把主工程**调试串口 TX** 从 FPGA 球 **V8**（板载 CH9102F）改到 **L12 = GPIO18 = P6-21**，
这样一根杜邦线就能把串口接到 WCH-LinkW 的无线串口助手，实现在车上无 USB 线的日志采集。
**逻辑与主工程完全相同**，只有调试串口引脚不同；电机/ToF/IMU/OLED/按键/ASR 接线都不变。

## 接线（只加一根线）

| 信号 | 板上位置 | 说明 |
|---|---|---|
| 调试串口 TX（FPGA → PC） | **P6-21**（GPIO18，FPGA 球 L12） | 115200 8N1，3.3 V，空闲高 |
| 地 | **P6-12** 或 **P6-30**（专用 GND） | 必须共地 |
| WCH-LinkW | **RX ← P6-21** | 不要接 3V3/5V（目标板自供电） |

电脑端接收器出现 `WCH-Link SERIAL (COMxx)`（本机为 COM24），两端都设 115200。
注意 WCH-Link 手册要求“调试功能和串口助手不能同时使用”；FPGA 烧写仍用 Gowin 下载器。

## 构建与烧写

```powershell
cd tests/p6_uart_debug
& 'D:/Gowin/Gowin_V1.9.12_x64/IDE/bin/gw_sh.exe' build.tcl      # -> impl/pnr/Embodied_RPACS_p6uart.fs
$fs = (Resolve-Path impl/pnr/Embodied_RPACS_p6uart.fs).Path      # programmer_cli 需要绝对路径
& 'D:/Gowin/Gowin_V1.9.12_x64/Programmer/bin/programmer_cli.exe' `
    --device GW5A-25A --operation_index 2 --fsFile "$fs"
```

已知构建结果（2026-10-29 23:30 重建，含 SPIN 360 / IMU ATTITUDE、PASS 兜底与前向 400 ms 滤波）：
Logic 15445/23040 (68%)、CLS 9407/11520 (84%)、Register 6553、Fmax 55.050 MHz。
Bitstream 6,228,180 字节，SHA256 `08DFDED3A43359D8E1237C03751E5D1978A97AFA958ED7C8276291FE706F399E`。

## 采集

```powershell
./tools/read_serial_log.ps1 -Port COM24 -Seconds 300 -Timestamped `
    -OutFile records/robustness_20261029/board_wchlink_run.log
./tools/decode_debug_log.ps1 -Path records/robustness_20261029/board_wchlink_run.log -ChangesOnly
```

`D` 行包含 route_reason/route_state/tof_reused，`E` 行是故障事件。若出现乱码，先确认
P6-21 与 GND 接线、COM24 是否 115200、WCH-Link 是否已配对成功。

## 距离日志诊断变体（2026-10-29 新增）

当 PASS/让位段仍出现 `BLOCKED reason=0x04` 时，用这个变体直接看三路距离，避免只凭
状态码猜测。它把 `D` 行的 64 位负载换成距离：

- `[15:0]` front_mm、`[31:16]` left_mm、`[47:32]` right_mm；
- `[50:48]` tof_reused（20 mm 截断确认）、`[55:51]` route_state、`[56]` fault、
  `[57]` auto_run、`[58]` avoid_mode。

构建：`gw_sh build_distance.tcl`（复用本目录 cst/sdc/hex，仅把 `system_motion.v` 换成
`src/system_motion_dist.v`，输出 `impl/pnr/Embodied_RPACS_p6dist.fs`）。
已构建：6,389,916 字节，SHA256 `EE00D3753180FB199EEA2CF433128374E4F77F1E450DF80FB5BEB1CA19C4EC10`，
Logic 15919/70%、CLS 9665/84%，`uart_tx` 在 P6-21/L12。

采集与解码：

```powershell
./tools/read_serial_log.ps1 -Port COM24 -Seconds 300 -Timestamped -OutFile <绝对路径>.log
python tools/decode_distance_log.py <日志> --changes-only
```

解码输出会给出每次状态切换时的三路距离；停止那帧的 front_mm 若持续 ≤85 说明真靠得近，
若只是瞬跳/`----`（20 mm 截断）则是坏数据。**该变体只用于诊断，不用于交付演示。**


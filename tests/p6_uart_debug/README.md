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
& 'D:/Gowin/Gowin_V1.9.12_x64/Programmer/bin/programmer_cli.exe' `
    --device GW5A-25A --operation_index 2 --fsFile impl/pnr/Embodied_RPACS_p6uart.fs
```

已知构建结果：Logic 13518/23040 (59%)、CLS 8328/11520 (73%)、BSRAM 9/56、
Bitstream SHA256 `AF7FC649E55A3130C84378539159E6B6ED4DA6200009767DDC885A53A6594F00`。

## 采集

```powershell
./tools/read_serial_log.ps1 -Port COM24 -Seconds 300 -Timestamped `
    -OutFile records/robustness_20261029/board_wchlink_run.log
./tools/decode_debug_log.ps1 -Path records/robustness_20261029/board_wchlink_run.log -ChangesOnly
```

`D` 行包含 route_reason/route_state/tof_reused，`E` 行是故障事件。若出现乱码，先确认
P6-21 与 GND 接线、COM24 是否 115200、WCH-Link 是否已配对成功。

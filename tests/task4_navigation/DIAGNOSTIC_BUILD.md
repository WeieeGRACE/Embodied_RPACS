# 任务四诊断位流

本次生成的是电机禁用诊断版本，顶层为 `task4_navigation_diagnostic_top`，不会驱动 PWMA/PWMB、方向脚或 STBY。OLED 仍使用 R3/T3，KEY0 为 B16，KEY1 为 A15，KEY2 为 C15，ToF 和 IMU 管脚保持原任务四约束。

## 仿真

在本目录执行：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\run_sim.ps1
& 'D:/app/modesilm/win64/vsim.exe' -c -l records/diag_compile.log -do sim/run_board_compile.do
& 'D:/app/modesilm/win64/vsim.exe' -c -l records/diagnostics.log -do 'vsim -c work.tb_oled_wire; run -all; quit -f'
& 'D:/app/modesilm/win64/gw_sh.exe' build_diagnostic.tcl
```

OLED 线级测试检查帧完成和内容变化；板级测试检查上电、急停锁存、复位清除和电机输出保持关闭。串口诊断以 `T4D` 开头，包含构建号、按键、帧计数、三路 ToF 原始值/接受值/年龄/错误码/状态和输入有效性。

## 位流

交付文件为 `impl/pnr/task4_navigation_diagnostic.fs` 和同名 `.bin`。本版本不自动烧录。烧录前确认 Programmer 已重新搜索 USB Cable，下载时钟可先降到 2.5 MHz。

上板只接通 FPGA、OLED、传感器和串口，电机驱动保持断电。确认 OLED 显示 `T4 DIAG`、帧计数变化、串口持续输出，并验证 KEY0/KEY2 后再恢复普通导航位流。

# 主工程 IMU 姿态 OLED 上板说明

主工程为仓库根目录的 `Embodied_RPACS.gprj`，顶层 `robot_system_top`。
下载文件为 `impl/pnr/Embodied_RPACS.fs`，使用 Gowin Programmer 的 SRAM 下载。
本次集成复用主工程唯一的 IMU 驱动和六轴采样，姿态角仅用于显示。

## 工程配置

- FPGA：GW5A-LV25UG324C2/I1，系统时钟 50 MHz。
- OLED：SSD1306 128×64，地址 0x3C，SCL=R3、SDA=T3，LVCMOS33/3.3 V；SDA 为开漏输出。
- IDE 的 CPU、SSPI 均作为普通 IO。两个保存配置位于 `impl/Embodied_RPACS_process_config.json`、`impl/project_process_config.json`，复制工程时须保留。
- 工程若已在 IDE 中打开，先关闭并重新打开，以重新加载保存配置。
- IMU、编码器、电机、串口、按键和 LED 接线沿用原主工程。

## 上板验收

1. 下载主工程 `.fs`，按 KEY0 复位，保持底盘与 IMU 静止。OLED 应从初始化进入 `IMU:CALIBRATING`；200 Hz 下平均 200 个有效陀螺仪样本，校准约 1 秒，另有初始化等待。
2. 校准期间就绪灯不亮，按 KEY1 不应启动；校准完成且姿态有效后就绪灯亮。校准期间的按键不会排队自动启动，须松开后重新按 KEY1。
3. 检查 Roll、Pitch、标记 REL 的 Yaw，约每秒刷新 5 次，数值精度显示至 0.1°。缓慢改变姿态，检查方向与连续变化。Yaw 为相对积分角，复位重新归零，长期有残余漂移。
4. 串口 115200/8N1；主机持续每 100 ms 发送一个二进制 `A5`。就绪后按 KEY1，双轮按原主工程目标运行 8 秒。不能发送 ASCII 文本 `A5` 代替二进制字节。
5. 按 KEY2 检查急停；复位、静止重新校准后再次启动，再停止发送心跳，检查 500 ms 看门狗停机及事件日志。故障锁存后须 KEY0 复位。
6. 再按 KEY0，确认重新进入校准、校准期间禁止启动，随后重新就绪。

IMU 驱动错误、错误寄存器异常、ready 丢失和采样超时均接入 OLED 的 IMU 故障显示，并保留原电机停机锁存。显示在下一帧更新，电机关断不等待显示刷新。
OLED 显示的是 IMU 状态，电机、急停及心跳故障仍通过原 LED/事件 UART 观察。
UART 的 `S I 00` 仍表示驱动启动及首帧采样成功；运动就绪还须等待姿态校准完成。

## 复现验证

从仓库根目录执行：

```powershell
./tools/run_system_sim.ps1
& 'D:/Gowin/Gowin_V1.9.12_x64/IDE/bin/gw_sh.exe' tests/gowin/build_system.tcl
./tools/check_system_reports.ps1
```

`tests/gowin/verify_system_project.tcl` 直接打开保存的 `.gprj` 并运行完整流程，不覆盖选项，用于复核 IDE 保存配置。
仿真覆盖主系统、姿态、OLED 及既有底盘/单轮/双轮回归；系统仿真缩短校准到 16 帧及各等待参数，生产工程保持 200 帧。
当次仿真和编译结果见 `tests/records/2026-10-19_system_oled_report.md`。

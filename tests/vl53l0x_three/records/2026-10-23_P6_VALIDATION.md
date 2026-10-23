# 三路测距 P6 版验证（2026-10-23）

结论：P6 管脚迁移完成，四套仿真通过；显式构建和重开保存 GPRJ 构建均退出 0，已生成新版位流。没有下载 FPGA 或验证物理距离。此记录替代当前接线的旧 P11 验证，历史记录见 `2026-10-20_P11_VALIDATION.md`。

## 接线和修改

| 模块丝印 | 前：P6 / GPIO / FPGA | 左：P6 / GPIO / FPGA | 右：P6 / GPIO / FPGA |
| --- | --- | --- | --- |
| SCL | 7 / GPIO6 / G16 | 13 / GPIO10 / H13 | 17 / GPIO14 / J13 |
| SDA | 8 / GPIO7 / G18 | 14 / GPIO11 / H14 | 18 / GPIO15 / K14 |
| GPIO1 | 9 / GPIO8 / F17 | 15 / GPIO12 / K15 | 19 / GPIO16 / K12 |
| XSHUT | 10 / GPIO9 / F18 | 16 / GPIO13 / K16 | 20 / GPIO17 / K13 |

VIN/GND 改为专用 3.3 V 电源接线并与 FPGA 共地，不分配座子针脚。避开编码器 P6-1～4 和 ASRPRO P6-5/6，P6-11/12 及 29/30 是电源/地，不能当作顺序 GPIO 接线。旧 P11 版位流不得用于新接线。

本次未修改测距、OLED 或串口 RTL；更新 CST、接线文档和管脚检查。ASRPRO 串口测试的同名测距端口同步至本表，保持高阻保护。主工程未集成测距。

## ModelSim

`run_sim.ps1` 重跑四套测试，均退出 0；ModelSim 10.7，各套无编译错误/警告。日志快照为本目录 `2026-10-23_P6_*.log`。

- bus：0x29 地址、寄存器传输、连续 12 字节读、最后 NACK/STOP、地址/数据 NACK、短时钟拉伸、时钟超时、SDA 卡低和恢复。
- three：三路不同距离及字节序、例程顺序、延迟就绪、100 次轮询超时、缺失设备、单路故障不阻塞其他路、下一轮 XSHUT 恢复；物理 UART 解码比对报告字段，复位及电机恒低检查。
- oled：实际 OLED I²C 解码、显存像素比对、WAIT、五位距离及原始状态、隐藏旧距离、错误码、恢复、帧快照一致性。三路集成仿真同时运行 OLED。
- timing：50 MHz 未缩短驱动默认参数，XSHUT 1 ms、启动 20 ms、I²C 不超过 100 kHz，并完成模拟测量。其他故障等待采用加速参数。

## Gowin 和最终报告

Gowin V1.9.12 (64-bit)，器件 GW5A-LV25UG324C2/I1。`build.tcl` 和 `verify_saved_project.tcl` 均完成综合、布局布线及位流生成，退出 0。

- 50 MHz 约束，Fmax **86.996 MHz**；最差 setup slack **8.505 ns**，setup/hold TNS 均为 0。此结论针对内部同步路径，外部异步输入不作同步外设接口时序保证。
- Logic 4286/23040（19%），Register 2101/23685（9%），锁存器 0，IO 25。
- `check_reports.py` 核对固定 P6 映射、两套工程一致性、最终 25 端口位置/方向/驱动/上下拉/3.3 V、保存配置及实际 device.cfg 的 CPU/SSPI 普通 IO 设置、位流不早于源文件和四套仿真日志。
- 扫描其他 11 份 CST，仅允许 ASRPRO 高阻保护工程的同名测距管脚复用，未发现其他用途占用。T11 输入、DRIVE=OFF、PULL=NONE；电机七路 RTL 固定低。SCL/SDA/XSHUT 仍由 RTL 实现 0/Z，GPIO1 为无上下拉输入。
- 剩余警告 CV0016：imu_gnd_reserved 输入未使用，最终报告保留其 T11 输入及无上下拉配置。
- 工具提示不能打开用户目录 Gowin cache/sh.log，但两次构建均退出 0；完整 stdout/stderr 已重定向到 records，最终报告和位流已独立核验。

下载文件：`../impl/pnr/vl53l0x_three.fs`，大小 5958596 字节。

SHA256：`042e5057f6eb299634a7fbb465b3664f910d7d3d7c4bf49accac435f9b770eca`。

最终逐项证据见 `2026-10-23_P6_report_check.txt`；未提交的完整工具中间报告仍在本机 impl/pnr，可重新构建生成。最终校验值以实际 fs 及重新运行检查脚本为准。

## 上板交付

按 README 断电迁线，先连接专用 3.3 V 电源和公共地，再下载本版 fs；UART 115200/8N1，OLED 应显示 TOF DIAG。分别移动前/左/右目标核对方向、串口与 OLED 数值及尺量距离；断电断开一路后上电检查隔离，再断电恢复接线。ASRPRO 测试应使用同步更新的 P6 版位流。

现有资料缺少完整初始化/校准。本次保持原 Arduino 诊断流程；valid=1 仅代表通信及就绪检查通过，不证明物理距离有效。硬件结果全部待用户验证。

# ASRPRO 串口测试：P6 测距保护迁移验证（2026-10-23）

结论：测距高阻保护管脚已同步新 P6 接线，三套仿真通过；显式构建及重开保存 GPRJ 构建均退出 0，已生成新版位流。没有下载 FPGA 或验证真实语音模块。本工程不执行实际运动。

## 修改与接线

| 测距保护信号 | 前：P6 / GPIO / FPGA | 左：P6 / GPIO / FPGA | 右：P6 / GPIO / FPGA |
| --- | --- | --- | --- |
| SCL | 7 / GPIO6 / G16 | 13 / GPIO10 / H13 | 17 / GPIO14 / J13 |
| SDA | 8 / GPIO7 / G18 | 14 / GPIO11 / H14 | 18 / GPIO15 / K14 |
| GPIO1 | 9 / GPIO8 / F17 | 15 / GPIO12 / K15 | 19 / GPIO16 / K12 |
| XSHUT | 10 / GPIO9 / F18 | 16 / GPIO13 / K16 | 20 / GPIO17 / K13 |

与三路测距测试新版使用同一接线；测距 VIN/GND 使用专用 3.3 V 电源接线并与 FPGA 共地，不再分配座子针脚。本版替代旧 P11 高阻保护映射，旧位流不得用于新接线。ASRPRO 串口仍为 P6-5/D17 接收、P6-6/D18 发送，电脑 UART 仍为 U8/V8。

本次只修改测距保护 CST、pins.json、检查脚本及说明；ASR 协议/控制 RTL 未修改。SCL/SDA/XSHUT 恒 Z，GPIO1 无上下拉输入，IMU 保留输入和 T11 无上下拉配置；电机七路恒 0。主工程不集成测距。

## ModelSim

`run_sim.ps1` 重跑 protocol、control_fifo、top 三套，均退出 0；ModelSim 10.7，无编译错误/警告。protocol 覆盖协议帧与错误/超时；control_fifo 覆盖控制、反馈原子性、溢出和故障优先级；top 使用实际 50 MHz/115200 baud，覆盖全部命令及状态、串口物理位解码、并发收发、按键和复位，全程断言电机恒低及九个测距开漏保护端口高阻。日志快照见 `2026-10-23_P6_protocol.log`、`2026-10-23_P6_control_fifo.log`、`2026-10-23_P6_top.log`。

## Gowin 与最终报告

Gowin V1.9.12 (64-bit)，GW5A-LV25UG324C2/I1。`build.tcl` 与 `verify_saved_project.tcl` 均完成综合、布局布线及位流生成，退出 0；日志快照带 `2026-10-23_P6_` 前缀。

- Fmax **70.042 MHz**，满足 50 MHz；最差 setup slack **5.723 ns**，setup/hold TNS 均为 0。外部异步 UART/按键不作同步外部 I/O 时序保证。
- Logic 6364/23040（28%），Register 8794/23685（38%），锁存器 0，IO 38。
- `check_reports.py` 检查固定 P6 映射、pins.json、源码端口和 CST、最终 38 个端口的封装位置/方向/驱动/上下拉/电压；保存配置和实际 device.cfg 的 CPU/SSPI 普通 IO 设置均通过；位流不早于工程源文件，仿真日志通过。
- 扫描其他 11 份 CST：仅允许三路测距工程的同名测距端口复用；D17/D18 允许现有 asr_motion GPIO 独立工程同用途 UART 别名复用（uart_rx/uart_tx），其他用途冲突会失败。这是明确用途和路径的检查，未整体跳过该工程。
- 最终保留 10 条 CV0016 未使用输入警告：T11、六路 IMU 输入、三路 GPIO1；最终管脚报告确认其输入、DRIVE=OFF 和 PULL=NONE。工具另提示无法打开用户目录 Gowin cache/sh.log；完整 stdout/stderr 已保存，未阻止构建及产物核验。

下载文件：`../impl/pnr/asrpro_uart.fs`，5958596 字节。

SHA256：`2c47ecf476ea47514507ab075f6bd139e319f8f76b221fb460358186edb135c2`。

逐项证据见 `2026-10-23_P6_report_check.txt`。旧 P11 版验证保存在 `2026-10-20_P11_VALIDATION.md`，不代表当前管脚和位流。完整中间报告仍在本机 impl/pnr，可重新构建生成。

## 待用户上板

按 README 和 BOARD_CHECKLIST 下载 P6 版位流，检查三路测距新接线保留时 ASRPRO 收发仍正常，电机不动作；测距由独立三路测距位流验证。本次不修改语音模块固件，也不替代 ASRPRO 词表/音频资源生成、烧录和真实语音测试。测距驱动初始化/校准资料仍不完整，实际测距结果待测。

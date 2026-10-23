# 三路 VL53L0X 测试说明

适用工程：`tests/vl53l0x_three/`。对应 P6 接线版本，日期：2026-10-23。已执行结果见 [测试报告](TEST_REPORT.md)，故障定位过程见 [修复记录](records/2026-10-23_I2C_FIX_VALIDATION.md)。

## 1. 测试目的与范围

验证前、左、右三路独立 I²C 诊断驱动、三路调度、UART 原始数据输出、OLED 文字渲染，以及下载后真实模块的读取。保持现有资料中的 Arduino 寄存器流程，不引入外部初始化表或校准代码；主工程不集成测距，ASRPRO 测试工程仅核对同名保护管脚的一致性。

自动检查证明通信、字节序和解析一致性。`valid=1` 不代表已经完成传感器物理有效性判断。精度、有效量程、环境适应性及长期稳定性需要单独定量测试。

## 2. 环境与接线

| 项目 | 配置 |
| --- | --- |
| 开发板/FPGA | ACG525 / GW5A-LV25UG324C2/I1 |
| 系统时钟 | 50 MHz |
| 模块 | 三个六孔 VL53L0X-V2，VIN=3.3 V，专用电源线并共地 |
| 测距 I²C | 三条独立总线，名义 100 kHz，各地址 0x29 |
| OLED | SSD1306 128×64，地址 0x3C，SCL=R3、SDA=T3 |
| 调试串口 | TX=V8，115200、8N1，3.3 V 电平；本次电脑端 COM14 |
| 工具 | ModelSim、Gowin V1.9.12、Gowin Programmer、PowerShell、Python |

| 信号 | 前方 P6 / FPGA | 左侧 P6 / FPGA | 右侧 P6 / FPGA |
| --- | --- | --- | --- |
| SCL | 7 / G16 | 13 / H13 | 17 / J13 |
| SDA | 8 / G18 | 14 / H14 | 18 / K14 |
| GPIO1 | 9 / F17 | 15 / K15 | 19 / K12 |
| XSHUT | 10 / F18 | 16 / K16 | 20 / K13 |

断电后按模块丝印接线，不按照片孔位或原理图连接器编号推断信号顺序。SCL/SDA/XSHUT 仅允许拉低或高阻释放；GPIO1 无内部上拉。电机七路输出恒低，STBY=0；T11 为无上下拉输入。完整网络名和用途见 [PIN 文档](../../PIN_ASSIGNMENT.md)。旧 P11 接线及对应旧位流不适用于本测试。

## 3. 仿真与构建

以下命令在本测试工程目录执行。复测前保存已有 records 日志，避免覆盖历史证据。

```powershell
./run_sim.ps1
& 'D:/Gowin/Gowin_V1.9.12_x64/IDE/bin/gw_sh.exe' build.tcl *> records/build.log
& 'D:/Gowin/Gowin_V1.9.12_x64/IDE/bin/gw_sh.exe' verify_saved_project.tcl *> records/build_saved_project.log
python check_reports.py
```

每条命令均应成功退出，再进行下一步。默认 ModelSim 路径为 `D:/app/modesilm/win64/vsim.exe`，可通过 `run_sim.ps1 -ModelSim <路径>` 修改。

| 用例 | 检查内容 | 合格条件 |
| --- | --- | --- |
| SIM-01 edges | SCL 路径偏差、300 ns 传感器响应、SDA 保持时间、ID 与连续读 | 保持时间断言及读取均通过 |
| SIM-02 bus | 寄存器访问、ACK/NACK、连续读取、时钟拉伸、SDA 卡低、恢复 | 成功路径正确，错误路径按时结束并恢复 |
| SIM-03 three | 三路顺序、不同数据、UART、轮询超时、设备缺失、XSHUT 恢复、复位及安全输出 | 单路错误不阻塞其他通道，串口与缓存一致，电机恒低 |
| SIM-04 oled | 初始化、清屏、WAIT、五位距离、状态、错误隐藏旧值、帧快照 | 重建显存与期望像素相同，帧内无数据撕裂 |
| SIM-05 timing | 50 MHz 默认参数 | XSHUT 低 1 ms、启动等待 20 ms、I²C 不超过 100 kHz |
| BUILD-01 | 综合、布局布线、位流及保存工程重开构建 | 均完成，无阻止下载的错误 |
| BUILD-02 | 最终管脚、两工程保护配置、CPU/SSPI、内部时序 | 固定映射一致，无用途冲突，满足 50 MHz |

## 4. 下载与串口检查

关闭占用 COM14 的串口助手。确认模块已上电，然后执行：

```powershell
./capture_board.ps1 -Download -Port COM14 -Lines 30 -Log records/board_latest.log
python check_board.py --log records/board_latest.log --require-all
```

脚本先检查构建产物，再将 `impl/pnr/vl53l0x_three.fs` 下载至 SRAM，最后采集串口并校验原始字节。不烧写 Flash；断电后需重新下载。只采集而不下载时省略 `-Download`。手工 Programmer 下载时必须选本测试工程位流。

| 用例 | 操作 | 合格条件 |
| --- | --- | --- |
| BOARD-01 | 连续采集 30 行完整诊断记录 | 三个方向均存在，各 10 次；ID=EE、rd=1F、valid=1、err=00 |
| BOARD-02 | 执行严格原始数据校验 | 距离/计数/状态与 raw 一致，三路全部通过 |
| BOARD-03 | 重新 SRAM 下载后再次采集 | 从复位启动后仍满足 BOARD-01/02 |
| BOARD-04 | 分别在前、左、右放置并移动目标 | 相应方向的数据和 OLED 行对应，观察变化；精度另用尺量记录 |
| BOARD-05 | 断电断开单路后上电测试，再断电恢复 | 故障路显示错误并隐藏旧距离；其他路继续，恢复后重新读取 |

BOARD-04/05 的人工观察应单独记录方向、动作及结果；没有现场记录的项目不得仅凭仿真或总体验收声明补写为逐项实测通过。

## 5. 字段与 OLED 判读

`rd=1F` 表示 C2、C0、50、70 和 12 字节结果均已读取。`raw` 依次对应 0x14～0x1F；距离为字节 10/11，环境计数为 6/7，信号计数为 8/9，均高字节在前。状态为字节 0 的 bit6:3。未读字段显示横线。

| err | 含义 | 首要检查 |
| --- | --- | --- |
| 00 | 通信及例程就绪检查通过 | 结合 raw、目标位置判读，不能单独作为精度结论 |
| 01 | 地址或数据 NACK | 供电、地址、SDA/SCL 连接及波形 |
| 02 | 就绪轮询超时 | 原始状态、启动流程及初始化资料 |
| 03 | 总线异常或时钟等待超时 | 引脚、电压、通断、上拉及短路 |

OLED 页面标题为 `TOF DIAG`，四行文字分别为标题、FRONT、LEFT、RIGHT。首次结果前为 WAIT；成功后显示五位毫米值及原始状态，故障时隐藏旧距离并显示 E:xx。约每秒刷新 5 次，与测量轮次频率不同。菜单可复用 4×21 ASCII 文字渲染接口，见 README。

## 6. 归档要求

保存五套仿真日志、两份构建日志、管脚/时序检查、下载日志、原始串口日志、严格检查结果及位流 SHA256。报告写明测试日期、接线版本、工具、样本数量、失败与修复、人工确认来源及未覆盖范围。历史失败日志保留，不用新成功日志覆盖。

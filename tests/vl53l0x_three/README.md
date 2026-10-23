# 三路 VL53L0X 诊断测试工程

测试文档：[测试说明与复测步骤](TEST_PROCEDURE.md) · [最终测试报告（2026-10-23）](TEST_REPORT.md)。三路自动读取检查通过，用户已确认上板测试通过；定量精度未单独记录。

独立顶层 `vl53l0x_three_top`，GW5A-LV25UG324C2/I1，50 MHz。前、左、右采用独立 100 kHz I²C 总线，地址均为 0x29，依次测量，完成一轮后等待 1 秒。主工程尚未集成。

**这是诊断版本。** 严格移植工程附带 `GY-VL53L0X资料/ARDUINO/VL53L0X.ino` 的寄存器流程，未引入外部初始化表、ST API 或校准。现有资料缺少完整初始化/校准；仿真通过仅证明数字逻辑符合此流程，实际模块能否完成测距需要上板验证。`valid=1` 只表示通信和就绪检查通过，不证明距离在物理上有效。

## 接线

按模块实物丝印接线，三路每个模块六根线全部连接。GPIO1/XSHUT 的实物顺序与附带原理图连接器编号不同，不按原理图编号数孔。

| 模块丝印 | 前方：P6 / 网络 / FPGA | 左侧：P6 / 网络 / FPGA | 右侧：P6 / 网络 / FPGA |
| --- | --- | --- | --- |
| SCL | 7 / GPIO6 / G16 | 13 / GPIO10 / H13 | 17 / GPIO14 / J13 |
| SDA | 8 / GPIO7 / G18 | 14 / GPIO11 / H14 | 18 / GPIO15 / K14 |
| GPIO1 | 9 / GPIO8 / F17 | 15 / GPIO12 / K15 | 19 / GPIO16 / K12 |
| XSHUT | 10 / GPIO9 / F18 | 16 / GPIO13 / K16 | 20 / GPIO17 / K13 |

VIN、GND 使用专用电源接线，不分配 P6/P11 针脚；VIN=3.3 V，三个模块与 FPGA 共地。P6-1～4 留给编码器，P6-5/6 留给 ASRPRO UART。**2026-10-23 P6 接线替代旧 P11 接线，旧版位流不可用于新接线。**

板上网络及端口名见 `../../PIN_ASSIGNMENT.md`。三路 SCL/SDA 互不连接。模块有 I²C 上拉及电平转换，SCL/SDA/XSHUT 在 RTL 中仅输出 0 或 Z，无内部上下拉。XSHUT/GPIO1 直接连接传感器内部 VDD，禁止推挽驱动 3.3 V；GPIO1 为同步后的诊断输入，仍需实测模块高电平满足 FPGA 输入门限。ASRPRO 串口测试新版位流已同步本套管脚，保持测距信号高阻保护。

板载 OLED：SSD1306 128×64，地址 0x3C，SCL=R3、SDA=T3。UART TX=V8，接 USB 串口 RX，115200、8N1、3.3 V 电平，共地。复位 B16 低有效，时钟 T9。

七路电机输出 PWMA/U17、AIN1/M16、AIN2/M14、PWMB/T17、BIN1/N15、BIN2/H12、STBY/G13 恒为 0。T11 保留输入，PULL=NONE、DRIVE=OFF；它不是专用地。

## 测距及错误码

上电 XSHUT 拉低 1 ms，释放后等 20 ms。每次读取 C2（修订号）、C0（ID）、50/70（周期寄存器），写 00=01，每隔 10 ms 检查 14 bit0，最多 100 次，然后读取 14 起始 12 字节。环境、信号、距离分别为字节 6/7、8/9、10/11，均高字节在前；原始状态为字节 0 的 bit6:3，不套用缺少依据的状态语义。寄存器指针写入后 STOP，再开始读，与 Wire 例程一致。

| err（十六进制） | 含义 |
| --- | --- |
| 00 | 通信及就绪检查通过 |
| 01 | 地址或数据 NACK |
| 02 | 100 次就绪轮询超时，保留诊断读取但结果无效 |
| 03 | 总线异常或等待时钟释放超过 10 ms |

故障终止当前通道，其他通道继续，故障通道下一轮先经 XSHUT 复位。GPIO1 仅观察，不作为就绪依据。一次完整轮询超时约 1 秒，三路同时超时会使一轮超过 4 秒；OLED 刷新频率不等于测量频率。串口故障报告只显示本次确实读取的字段；未读取字段显示横线，应先检查 rd、valid 和 err。

启动串口提示初始化/校准缺失。每路每轮一行：

```text
FRONT seq=00000001 mm=01234 id=EE rev=10 pre=06 final=04 status=03 ambient=00291 signal=01110 gpio=1 valid=1 err=00 dbg=0000 pad=7 rd=1F raw=1900000000000123045604D2
```

方向 FRONT/LEFT/RIGHT，seq 为八位十六进制完成序号；ID、修订、周期、原始状态、err 为十六进制；距离、环境和信号计数为五位十进制。每通道一份待发送快照，硬件默认周期足够发送；极端加速参数下不保证保存每个事件。

## 实测诊断与串口闭环（2026-10-23）

已直接通过 JTAG 下载 SRAM，并从 COM14 采集板上数据。修复 I²C 数据保持时间后前/左恢复；用户随后纠正右侧接线，三路均恢复 ID=EE、rev=10 和原始结果读取。改线后以及重新下载启动后两次采集均为三路各 10/10 成功，共 60 条成功记录，原始字节解析及 `--require-all` 检查通过。光学准确度仍未验证，前/右偶有 20 mm 返回值；不能将通信成功标志当作物理距离有效性结论。

- `rd` 为读取掩码：bit0=C2、bit1=C0、bit2=50、bit3=70、bit4=12 字节结果。`1F` 表示全部读取完成，`00` 表示没有成功取得这些寄存器。
- `raw` 按地址递增显示 0x14～0x1F 的 12 字节原始数据。未完成结果读取时显示 24 个横线，距离/计数/状态也显示横线；未读寄存器显示 `--`。
- `dbg[15:12]` 为寄存器事务阶段：0=START、1=写地址52、2=寄存器指针、3=写数据、4=STOP、5=读START、6=读地址53、7=读取、8=最终STOP。`[11:8]` 为结果字节索引，`[5:2]` 为底层故障状态，`[1:0]` 为故障瞬间 SCL/SDA。底层状态1=等待START空闲、4=等待STOP的SCL、8=数据时钟拉伸等待、9=NACK采样。
- `pad` 是完成事件附近同步采样的 `{SCL,SDA,XSHUT}`：7=111（均高），1=001（总线均低、XSHUT高）。它是 FPGA 采样结果，不能替代万用表的实际电压测量。
- `valid=1` 仍仅代表通信和例程的就绪检查通过；原始状态不作未经资料支持的物理有效性解释。

```powershell
# 自动检查位流并下载 SRAM，然后采集30行；COM14需未被其他串口工具占用。
./capture_board.ps1 -Download -Port COM14 -Lines 30
# 只读采集：
./capture_board.ps1 -Port COM14 -Lines 30
# 独立核对串口距离/计数/状态是否与原始字节一致，并要求三路全部通信成功：
python check_board.py --log records/board_latest.log --require-all
```

最后一条命令要求三路均成功，故障时会失败，不会将单路成功误判为三路完成。最新通过证据为 `records/2026-10-23_rewired_restart_check.log`；修复及排查经过见 `records/2026-10-23_I2C_FIX_VALIDATION.md`，旧 P6 初次迁移记录是修复前历史。

`build_probe.tcl` 生成临时 `tof_bus_probe.fs`，电机恒低，仅对三路 SCL/SDA/XSHUT 输出0/Z，用于板上引脚电平对照；不是测距位流。使用后应重新运行 `build.tcl` 或 `verify_saved_project.tcl` 恢复测距工程配置，再下载测距 fs。

## OLED 与后续菜单接口

```text
TOF DIAG
FRONT:01234mm S:03
LEFT :-----  WAIT
RIGHT:-----  E:02
```

首次完成前 WAIT；故障时隐藏旧距离。6×8 ASCII，四行分别位于像素 y=0/16/32/48，约每秒刷新 5 次。

三层结构：`tof_three` 缓存三路最近结果 → `tof_page` 组合生成文字 → `tof_oled_text` 通用四行渲染。渲染器输入 `text[671:0]`，84 个 ASCII 字节，行优先，每行 21 字符，第 n 个字符在 `[8*n +: 8]`；行尾填空格。每帧开始锁存完整快照，帧内不撕裂。后续菜单可选择不同页面的同格式文字输入；本次没有按键菜单或页面切换。字体为本页需要的 ASCII 子集，未定义字符显示空白，扩展菜单文字时需补充 glyph。

字体和渲染器均在此测试目录，现有姿态 OLED 未修改。仅引用现有 `src/uart/uart_tx.v` 和 `src/imu/oled_i2c_write_byte.v`。

## 仿真、构建与下载

在本目录运行 PowerShell：

```powershell
./run_sim.ps1
& 'D:/Gowin/Gowin_V1.9.12_x64/IDE/bin/gw_sh.exe' build.tcl *> records/build.log
# 验证重新打开已保存工程也能构建：
& 'D:/Gowin/Gowin_V1.9.12_x64/IDE/bin/gw_sh.exe' verify_saved_project.tcl *> records/build_saved_project.log
```

ModelSim 默认路径 `D:/app/modesilm/win64/vsim.exe`，可用 `-ModelSim` 参数更改。五套测试（新增带时钟路径偏差及300 ns传感器响应延迟的 edges 回归）及日志见 `sim/`、`records/`；模型只实现给定例程涉及的协议，不是传感器物理模型。

Gowin IDE 打开本目录 `vl53l0x_three.gprj`；随工程保存 `impl/*process_config.json`，CPU 和 SSPI 均配置为普通 IO（true），以释放 OLED 管脚。不要用主工程替代此独立工程。50 MHz SDC 位于 `src/vl53l0x_three_top.sdc`。

**下载文件：`impl/pnr/vl53l0x_three.fs`。** 使用对应开发板的 Gowin Programmer 下载到 SRAM 验证。已连接板卡并执行 SRAM 下载及 COM14 读取，三路均通过通信与原始数据解析检查。最新构建/时序/管脚/板上结果见 `records/2026-10-23_I2C_FIX_VALIDATION.md`。

上板步骤：

1. 断电检查上述 12 根信号线、专用电源接线及公共地，确认 VIN=3.3 V；连接串口并打开 115200/8N1。
2. 下载独立 fs，复位，检查串口启动提示和 OLED 的 TOF DIAG。初始 WAIT 后应出现结果或错误码。
3. 分别仅在前、左、右放置并移动目标，核对相应方向的串口与 OLED 数值同步变化，并用尺测量对照；valid=1 仍需这一步物理核验。
4. 断电后断开一个模块再上电，检查该路错误、隐藏旧距离，其他两路继续。断电恢复接线后上电，核对恢复；不要带电拔插。
5. 若持续 E:02 或读数异常，记录完整串口日志和供电/接线情况。现有例程不含完整初始化，不能据此认定模块损坏；后续需补齐匹配的初始化资料再升级正式驱动。

来源：附带 Arduino 示例包含 Ted Meyers 及 Google Groups 参考注释；本工程仅移植该例程寄存器流程，未移植其中未调用的辅助函数，也未使用外部初始化代码。

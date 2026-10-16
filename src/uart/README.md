# UART 编码器观测

打开`tests/gowin/robot_encoder_test.gprj`，顶层选择 `robot_encoder_test_top`。
此工程只包含编码器测试的一套 CST/SDC，不要使用包含三个顶层约束的旧工程进行本次下载。
也可在 Gowin 命令行使用 `gw_sh tests/gowin/build_encoder_test.tcl`；脚本明确指定顶层和器件。

## 接线及串口助手

电机动力线断开，仅为编码器供电，并与 FPGA 共地。
A/B 高电平先实测；超过 FPGA 3.3 V IO 允许电压时需电平转换。
左 A/B 接 40-pin GPIO0/C17、GPIO1/C18；右 A/B 接 GPIO2/F14、GPIO3/G14。
串口 TX 使用板载 UART 网络 V8，不再作为步进调试输出；U8 不由 FPGA 驱动。
管脚依据随板 `02_例程源码/ch9_uart_loopback/src/uart_loopback.cst` 的 uart_tx/V8、uart_rx/U8，
与 `ch7_uart_rx` 的 UART 管脚一致，不使用任意扩展 IO 替代。
按开发板串口连接方式将板载串口连到电脑（UART 电平为 3.3 V TTL，不是 RS-232）。
如果使用外接 USB-TTL：适配器 RX 接板上 UART TX，GND 接板上 GND；不接适配器 TX 和供电脚。

串口助手：115200 baud，8 数据位，1 停止位，无校验、无流控，文本显示。
默认每 100 ms 输出一次，例如：

```
L=1456 R=-1456 DL=1 DR=0 IL=0 IR=0 FL=0 FR=0
```

L/R 是有符号累计四倍频计数；DL/DR 为最近一步的方向（1 正、0 负，静止保持）。
按当前实测安装方向，顶层设 INVERT_L=1、INVERT_R=0，使左右轮向前均为正数；无需交换A/B接线。
IL/IR 是无符号非法跳变累计次数；FL/FR 是锁存的编码器故障。
KEY0 复位清零所有计数和故障，串口恢复空闲高电平。
在轮胎上做标记，记录转一整圈前后 L/R 的差值；比较多次结果，不以1456强制判定。
分别转左右轮确认通道，对正反转确认计数符号。正常转动 IL/IR 应为0、FL/FR 应为0。

## 底层接口

`uart_tx`：CLK_HZ/BAUD 参数；data、valid、ready、tx、done 接口。
valid && ready 在时钟上升沿接受一个字节；忙时不接受；done 在停止位结束后脉冲一拍。
50 MHz/115200 使用434拍/bit，实际约115207 baud，8N1、LSB先发。
`encoder_uart_telemetry` 每次快照锁存全部字段，逐位十进制转换后发送 CRLF 行。
发送繁忙时丢弃到期的报告，不排队、不覆盖当前快照；默认10 Hz足以完成每行发送。

## 验证

在 sim 目录运行 `vsim -c -do run_uart_observation.do`。
涵盖 UART 帧、负数/32位极值、快照一致性、A/B正反转、非法跳变与复位清零。
ModelSim 仿真不代表 Gowin 时序收敛或实测 IO 电压；综合、布局布线和实际下载仍需板端验证。

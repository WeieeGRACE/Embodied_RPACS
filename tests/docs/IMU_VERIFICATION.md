# IMU 上板验证记录（2026-10-16）

已完成编译、布局布线、SRAM 下载，以及 COM14 / 115200 / 8N1 串口验证。Fluent 串口助手保持连接。

## 初始化修复

首次识别到 ID=0x24 后，写入 CMD(0x7E)=0xB6，等待 2 ms，再重新读取 ID，随后执行配置下载。自动重试也重新执行传感器复位。此修改解决了本次传感器持续供电、FPGA 重复下载条件下的初始化失败；未进一步推断芯片内部失败原因。

同时保留 PWR_CONF 写后 1 ms 等待，增加写事务完成后的 2 us 恢复间隔，并在配置流之前显式清零 INIT_ADDR 两个寄存器。SPI 最后下降沿完成接收时仍使用 rx_data <= rx_shift。

UART 增加 PWR、CTRL、SERR 原始寄存器诊断值；成功时 CTRL=37 是实际读回值。

## 验证结果

- C 配置数组与 HEX 共 8192 字节，逐字节一致。
- RTL 仿真检查全部 8192 配置字节、正常初始化、首次失败后的自动重试、6 轴连续读取、小端有符号转换和样本提交，均通过。
- 综合后 ROM 仿真扫描全部 8192 地址，与 HEX 一致。厂商参数化仿真库存在 part-select 警告，比较结果通过。
- Gowin 综合、布局布线和 bitstream 生成通过。50 MHz 约束下报告 Fmax=50.083 MHz，setup/hold 违例端点均为 0，TNS 均为 0。
- 同一 bitstream 两次 SRAM 下载后均正常。第二次采集 8 秒、161 行，全部 READY=1、ERR=0、ISTAT=1；SEQ 从 407 增至 2007，与约 200 Hz 采样一致。

实测示例：

```text
ID=36 READY=1 ERR=0 SEQ=2007 INT1=0 INT2=0 ISTAT=1 AX=195 AY=50 AZ=-4073 GX=-3 GY=7 GZ=6 PWR=0 CTRL=37 SERR=0
```

串口原始记录位于工作区根目录 ../records/imu_softreset_capture.txt 和 ../records/imu_softreset_repeat_capture.txt，Fluent 截图为 ../records/imu_fluent_connected.png。

最终文件：../../impl/pnr/robot_imu_test.fs

SHA256：24C9EC1EF707C0F41CE5EC3518F3A8DCCD6A3D97C4A3F7F65EE4BD00D7F6BA8E

## 复现

在本项目目录执行 ../gowin/build_imu_test.tcl。ModelSim 使用 ../../sim/tb_imu660ra_driver.v；默认验证正常初始化，FAIL_FIRST_INIT=1 验证失败后重试。

本次未验证物理转动、断开重连或整板冷上电。当前为 SRAM 下载，断电后不保留。

初始化复位、写恢复时间和配置地址流程参考 Bosch 官方 BMI270 SensorAPI 的 bmi2_soft_reset、bmi2_set_regs、upload_file：
https://github.com/boschsensortec/BMI270_SensorAPI/blob/master/bmi2.c

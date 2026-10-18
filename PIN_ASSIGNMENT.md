# P11 引脚分配说明

本文档记录 ACG525 开发板上的 IMU SPI、TB6612 电机接口 P11 接线方案，以及 IMU 测试工程和编码器接口的 IO 约束。本文档不修改 RTL 或 CST 约束。

## IMU 接线

IMU 供电使用 P11 的 **VCC3P3**。SPI 由 FPGA 作为主机，INT1/INT2 作为输入同步到 FPGA 后再使用。

| IMU 信号 | RTL 端口 | 相对 FPGA 的方向 | P11 针脚 | 板上网络 | FPGA 封装管脚 | 说明 |
| --- | --- | --- | --- | --- | --- | --- |
| VCC | — | 电源输出至 IMU | P11-1 | VCC3P3 | — | IMU 供电，使用 3.3 V |
| GND | — | 电源回路 | P11-3 | LCD_GPIO0 | T11 | 按已确认接法作为 IMU 地 |
| SCL/SPC | `imu_spi_sck` | FPGA → IMU | P11-5 | LCD_GPIO2 | V15 | SPI 时钟 |
| SDA/SDI | `imu_spi_mosi` | FPGA → IMU | P11-7 | LCD_GPIO4 | V16 | SPI SDI/MOSI |
| SA0/SDO | `imu_spi_miso` | IMU → FPGA | P11-9 | LCD_GPIO6 | U18 | SPI SDO/MISO；SPI 模式下按数据输出使用 |
| CS | `imu_cs_n` | FPGA → IMU | P11-11 | LCD_GPIO8 | T18 | 片选，低有效 |
| INT1 | `imu_int1` | IMU → FPGA | P11-13 | LCD_GPIO10 | N14 | 中断输入，仅同步引出 |
| INT2 | `imu_int2` | IMU → FPGA | P11-15 | LCD_GPIO12 | N16 | 中断输入，仅同步引出 |

## IMU 测试工程 IO 约束

`tests/gowin/robot_imu_test.gprj` 实际引用 `src/robot_imu_test_top.cst`。以下为该约束的全部 13 个端口，方向相对 FPGA；所有端口均为 `IO_TYPE=LVCMOS33`、`BANK_VCCIO=3.3`。

| RTL 端口 | FPGA 封装管脚 | 方向 | PULL_MODE | DRIVE | 用途 |
| --- | --- | --- | --- | --- | --- |
| `clk_50m` | T9 | 输入 | NONE | OFF | 50 MHz 时钟 |
| `reset_n` | B16 | 输入 | NONE | OFF | 低有效复位 |
| `imu_spi_sck` | V15 | 输出 | NONE | 8 | IMU SPI 时钟 |
| `imu_spi_mosi` | V16 | 输出 | NONE | 8 | IMU SPI 数据输出 |
| `imu_spi_miso` | U18 | 输入 | NONE | OFF | IMU SPI 数据输入 |
| `imu_cs_n` | T18 | 输出 | NONE | 8 | IMU 低有效片选 |
| `imu_int1` | N14 | 输入 | DOWN | OFF | IMU INT1，内部下拉 |
| `imu_int2` | N16 | 输入 | DOWN | OFF | IMU INT2，内部下拉 |
| `uart_tx` | V8 | 输出 | NONE | 8 | 串口遥测发送 |
| `led_ready` | D14 | 输出 | NONE | 8 | IMU 就绪状态 |
| `led_error` | C14 | 输出 | NONE | 8 | IMU 错误状态 |
| `led_int1` | B9 | 输出 | NONE | 8 | INT1 同步状态 |
| `led_int2` | A9 | 输出 | NONE | 8 | INT2 同步状态 |

## 编码器 IO 分配

编码器 A/B 信号均为 FPGA 输入。`src/robot_encoder_test_top.cst` 与 `src/robot_closed_loop_test_top.cst` 中的四路编码器分配一致，均配置为 `IO_TYPE=LVCMOS33`、`BANK_VCCIO=3.3`、`PULL_MODE=DOWN`、`DRIVE=OFF`。以下 GPIO 网络独立于 P11 的 LCD_GPIO 网络。

| 编码器信号 | RTL 端口 | 相对 FPGA 的方向 | 板上网络 | FPGA 封装管脚 |
| --- | --- | --- | --- | --- |
| 左编码器 A 相 | `encoder_a_l` | 编码器 → FPGA（输入） | GPIO0 | C17 |
| 左编码器 B 相 | `encoder_b_l` | 编码器 → FPGA（输入） | GPIO1 | C18 |
| 右编码器 A 相 | `encoder_a_r` | 编码器 → FPGA（输入） | GPIO2 | F14 |
| 右编码器 B 相 | `encoder_b_r` | 编码器 → FPGA（输入） | GPIO3 | G14 |

编码器 A/B 接入电平须符合 3.3 V IO 要求，必要时使用电平转换；供电沿用已验证的接法。

## 电机接线

以下为当前电机接口的完整分配。除 AIN1、BIN1 外，其余信号沿用现有测试工程的映射。

| 电机信号 | 方向 | P11 针脚 | 板上网络 | FPGA 封装管脚 | 用途 |
| --- | --- | --- | --- | --- | --- |
| PWMA | FPGA → TB6612 | P11-10 | LCD_GPIO7 | U17 | 左路 PWM |
| AIN1 | FPGA → TB6612 | P11-18 | LCD_GPIO15 | M16 | 左路方向 1 |
| AIN2 | FPGA → TB6612 | P11-14 | LCD_GPIO11 | M14 | 左路方向 2 |
| PWMB | FPGA → TB6612 | P11-12 | LCD_GPIO9 | T17 | 右路 PWM |
| BIN1 | FPGA → TB6612 | P11-16 | LCD_GPIO13 | N15 | 右路方向 1 |
| BIN2 | FPGA → TB6612 | P11-32 | LCD_GPIO23 | H12 | 右路方向 2 |
| STBY | FPGA → TB6612 | P11-31 | LCD_GPIO22 | G13 | 驱动器待机控制 |

## 接地和上电注意事项

1. 按已确认的机械接线，IMU GND 接 **P11-3 / LCD_GPIO0 / T11**。但是板卡原理图将该点标为 FPGA 信号，P11-2 才是专用 GND。P11-3 只有在实测确认与板卡地可靠连通时才能作为 IMU 地，不能仅根据 `LCD_GPIO0` 网络名判断。
2. 上电前确认 IMU 与板卡之间存在可靠的公共地，并确认没有把 IMU GND 接到 VCC 或其他 FPGA 输出。
3. 只使用 VCC3P3，不要把 IMU 的 VCC 接到 VCC5P0。
4. T11 被用作地时，FPGA 侧不得对 `LCD_GPIO0` 进行推挽输出，也不得配置内部上拉或下拉。集成 RTL/CST 时应将该点保持未驱动或高阻，避免 FPGA 与地之间形成冲突。

## 与现有测试约束的关系

现有 `src/robot_closed_loop_test_top.cst` 仍然使用旧电机分配：`AIN1=U18/LCD_GPIO6`、`BIN1=T18/LCD_GPIO8`。这会分别与新 IMU 的 `SA0/SDO` 和 `CS` 发生管脚/网络冲突。集成新接线前必须同步修改顶层端口、RTL 和约束，确保同一个 FPGA 管脚不会同时作为电机输出和 IMU 信号使用。本文档更新不修改这些文件。

## 核对依据

- P11 连接器及电源网络：`小梅哥ACG525型国产高云FPGA教学开发板/06_硬件图纸/AltiumDesigner/ACG525-CB-开发板原理图.pdf`。
- `LCD_GPIO0` 至 `LCD_GPIO15` 的 FPGA 封装管脚：`小梅哥ACG525型国产高云FPGA教学开发板/04_引脚功能信息/小梅哥ACG525型国产高云FPGA开发板引脚信息表.xlsx`，`Sheet1` 对应引脚表。
- 旧电机约束：`src/robot_closed_loop_test_top.cst`。
- IMU 测试工程及约束：`tests/gowin/robot_imu_test.gprj`、`src/robot_imu_test_top.cst`；端口方向见 `src/robot_imu_test_top.v`。
- 编码器约束及端口方向：`src/robot_encoder_test_top.cst`、`src/robot_encoder_test_top.v`；GPIO 网络名称见 `tests/docs/LEFT_WHEEL_CLOSED_LOOP.md`。

P11 接线表中的 IMU 和电机信号使用的针脚、板上网络和 FPGA 封装管脚均不重复；IMU 测试工程 IO 表是对同一组 IMU 信号及其辅助端口的约束记录。此次仅更新 Markdown，未执行 FPGA 构建。

## 主工程约束（2026-10-18）

主工程唯一顶层 `robot_system_top`，仅引用 `src/robot_system_top.cst` 和同名 SDC。
前文旧映射描述仍适用于独立测试工程；主工程已使用本文新映射：AIN1=M16、BIN1=N15、IMU MISO=U18、CS=T18。

- 时钟 T9、复位 B16、KEY1/A15、KEY2/C15、UART TX/V8、RX/U8。
- LED：`led_ready`/D14 为模块就绪且无系统故障，`led_run`/C14 为运行，`led_fault`/B9 为故障，`led_heartbeat`/A9 为复位后收到过有效心跳；后者不代表当前心跳仍正常。
- T11 显式分配为 `imu_gnd_reserved` 输入，`PULL_MODE=NONE`、`DRIVE=OFF`，状态随 IMU 样本保留。该点不驱动输出，避免 Gowin 对未用管脚默认启用上拉。
- 所有 28 个端口为 LVCMOS33、3.3 V bank；编码器和 IMU 中断输入下拉，按键和 UART RX 上拉，其余无上下拉。
- `tools/check_system_reports.ps1` 已逐项检查最终管脚报告中的位置、方向、驱动和上下拉，无重复分配。

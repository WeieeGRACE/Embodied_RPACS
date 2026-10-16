# P11 引脚分配说明

本文档记录已确认的 P11 接线方案，适用于 ACG525 开发板上的 IMU SPI 接口和 TB6612 电机接口。本次只新增文档，不修改 RTL 或 CST 约束。

## IMU 接线

IMU 供电使用 P11 的 **VCC3P3**。SPI 由 FPGA 作为主机，INT1/INT2 作为输入同步到 FPGA 后再使用。

| IMU 信号 | 相对 FPGA 的方向 | P11 针脚 | 板上网络 | FPGA 封装管脚 | 说明 |
| --- | --- | --- | --- | --- | --- |
| VCC | 电源输出至 IMU | P11-1 | VCC3P3 | — | IMU 供电，使用 3.3 V |
| GND | 电源回路 | P11-3 | LCD_GPIO0 | T11 | 按已确认接法作为 IMU 地 |
| SCL/SPC | FPGA → IMU | P11-5 | LCD_GPIO2 | V15 | SPI 时钟 |
| SDA/SDI | FPGA → IMU | P11-7 | LCD_GPIO4 | V16 | SPI SDI/MOSI |
| SA0/SDO | IMU → FPGA | P11-9 | LCD_GPIO6 | U18 | SPI SDO/MISO；SPI 模式下按数据输出使用 |
| CS | FPGA → IMU | P11-11 | LCD_GPIO8 | T18 | 片选，通常为低有效 |
| INT1 | IMU → FPGA | P11-13 | LCD_GPIO10 | N14 | 中断输入，仅同步引出 |
| INT2 | IMU → FPGA | P11-15 | LCD_GPIO12 | N16 | 中断输入，仅同步引出 |

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

现有 `src/robot_closed_loop_test_top.cst` 仍然使用旧电机分配：`AIN1=U18/LCD_GPIO6`、`BIN1=T18/LCD_GPIO8`。这会分别与新 IMU 的 `SA0/SDO` 和 `CS` 发生管脚/网络冲突。集成新接线前必须同步修改顶层端口、RTL 和约束，确保同一个 FPGA 管脚不会同时作为电机输出和 IMU 信号使用。本次没有修改这些文件。

## 核对依据

- P11 连接器及电源网络：`小梅哥ACG525型国产高云FPGA教学开发板/06_硬件图纸/AltiumDesigner/ACG525-CB-开发板原理图.pdf`。
- `LCD_GPIO0` 至 `LCD_GPIO15` 的 FPGA 封装管脚：`小梅哥ACG525型国产高云FPGA教学开发板/04_引脚功能信息/小梅哥ACG525型国产高云FPGA开发板引脚信息表.xlsx`，`Sheet1` 对应引脚表。
- 旧电机约束：`src/robot_closed_loop_test_top.cst`。

本表中的 IMU 和电机信号使用的 P11 针脚、板上网络和 FPGA 封装管脚均不重复；本次仅新增 Markdown，未执行 FPGA 构建。

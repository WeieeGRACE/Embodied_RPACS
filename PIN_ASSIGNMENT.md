# 主工程当前接线（2026-10-23）

主工程已集成 ToF、ASRPRO 与 OLED 菜单；本节及 `src/robot_system_top.cst` 为当前接线依据。
下方旧章节保留为历史测试说明。

| 用途 | 主工程端口 | FPGA 球位 / 连接 |
| --- | --- | --- |
| 50 MHz / K0 / K1 / K2 | clk_50m / reset_n / key_enable_n / key_estop_n | T9 / B16 / A15 / C15 |
| ASRPRO 接收 / 发送 | asr_rx / asr_tx | D17 / D18，P6-5 / P6-6 |
| USB 接收预留 / 事件发送 | uart_rx / uart_tx | U8 / V8，不要求心跳 |
| 左 / 右编码器 A、B | encoder_a_l,b_l / encoder_a_r,b_r | C17、C18 / F14、G14 |
| 左电机 PWM、IN1、IN2 | PWMA / AIN1 / AIN2 | U17 / M16 / M14 |
| 右电机 PWM、IN1、IN2 | PWMB / BIN1 / BIN2 | T17 / N15 / H12 |
| 电机待机 | STBY | G13 |
| IMU SCK、MOSI、MISO、CS | imu_spi_sck,mosi,miso / imu_cs_n | V15、V16、U18、T18 |
| IMU 中断 / 地预留 | imu_int1,int2 / imu_gnd_reserved | N14、N16 / T11，仅输入、无上下拉 |
| OLED SCL / SDA | oled_scl / oled_sda | R3 / T3，CPU/SSPI 设为普通 IO |
| 前 ToF SCL、SDA、GPIO1、XSHUT | tof_front_* | G16、G18、F17、F18；P6-7/8/9/10 |
| 左 ToF SCL、SDA、GPIO1、XSHUT | tof_left_* | H13、H14、K15、K16；P6-13/14/15/16 |
| 右 ToF SCL、SDA、GPIO1、XSHUT | tof_right_* | J13、K14、K12、K13；P6-17/18/19/20 |
| 就绪 / 运行 / 故障 / 语音启用 | led_ready,run,fault,heartbeat | D14 / C14 / B9 / A9 |

K0 的旧端口名 `reset_n` 保留，但仅最外层目录的按下事件触发全局复位。
K2 只负责选择；K0 在所有页面和目录均立即禁止电机输出，随后执行返回或复位。
ASRPRO 保持 Serial1 PA2/TX → D17、PA3/RX ← D18 的已验证接线。
ToF SCL/SDA/XSHUT 均只驱动 0 或释放，无内部上下拉；GPIO1 仅作输入。
供电和公共地使用专用电源/GND 接线；T11 是保留输入，不能当作 FPGA 驱动地。

---

# P6 / P11 引脚分配说明

## ASRPRO 独立串口测试引脚（2026-10-20）

先冻结以下分配，再实现独立工程 `tests/asrpro_uart/asrpro_uart.gprj`；不将 ASR 串口与电脑调试串口共接。

| RTL 端口／用途 | FPGA 管脚 | 网络／连接器 | 方向、接线及电气配置 |
| --- | --- | --- | --- |
| `asr_uart_rx` | D17 | GPIO4／P6-5 | ASRPRO TX → FPGA；输入、内部上拉、DRIVE=OFF |
| `asr_uart_tx` | D18 | GPIO5／P6-6 | FPGA → ASRPRO RX；输出、无上下拉、8 mA、空闲高 |
| `uart_tx`／`uart_rx` | V8／U8 | 板载 USB 串口 | 电脑调试；TX 无上下拉、8 mA；RX 上拉、DRIVE=OFF |
| `clk_50m`／`reset_n` | T9／B16 | 板载时钟／KEY0 | 输入、无上下拉、DRIVE=OFF；50 MHz／低有效复位 |
| `key_step_n`／`key_fault_n` | A15／C15 | KEY1／KEY2 | 输入、内部上拉、DRIVE=OFF；消抖后的状态推进／故障注入 |
| `led_ready`／`led_busy`／`led_error`／`led_rx` | D14／C14／B9／A9 | LED0／LED1／LED2／LED3 | 输出、无上下拉、8 mA；就绪／忙碌／错误／有效收帧指示 |
| 公共地 | — | P6-12 或 P6-30 专用 GND | 连接语音模块 GND；不要使用普通 GPIO 作地 |

所有上述信号均为 LVCMOS33、BANK_VCCIO=3.3，串口统一 115200 baud、8N1、无流控。P6-5/6 为底板原理图第 5 页 P6 的 GPIO4/5，不是 P11 的 LCD_GPIO4/5。D17/D18 到 GPIO4/5 的映射沿用计划书和板卡引脚表。

> **调试串口与事件日志计划（2026-10-29 用户确认）**：上板 3–5 次连续避障验收期间仍保留
> `system_event_logger` 与 V8/U8 调试串口；验收通过后删除事件日志，并**同步移除 `uart_tx`/`uart_rx`
> 顶层端口与 `src/robot_system_top.cst` 中的 V8/U8 约束，释放这两个 FPGA 球**；
> `tools/check_system_reports.ps1` 的受约束引脚期望由 44 调整为 42（预研变体已确认 IOB 104→102，
> 即释放 V8/U8 两个球；资源实测 Logic 13364/58%、CLS 8197/72%、BSRAM 9→5、Fmax 61.113 MHz）。
> 届时本表“UART 调试”相关行改为“已释放（历史）”，并同步更新 `PROJECT_PLAN.md` §16.4 与
> `records/resource_20261029/MINIMAL_PLAN.md` Step 4。

分配前已核对 `src/`、`tests/` 全部现有 CST：D17/D18 未被主工程、IMU/OLED、三路 ToF 或摄像头工程占用；GPIO0～3/C17/C18/F14/G14 继续留给编码器。共用板载时钟、按键、LED 和电脑串口属于独立位流之间的相同用途复用，不是同一顶层内重复分配。旧电机测试约束与 IMU 的 U18/T18 冲突仍按本文原说明处理，不能用于本测试工程。

本测试位流将 U17/M16/M14/T17/N15/H12/G13 七路电机输出固定为低；T11 保持输入、无上下拉。三路 ToF 的 SCL/SDA/XSHUT 保持高阻、GPIO1 保持无上下拉输入；IMU SPI 及中断保留为无上下拉输入，避免影响现有外接线。测距高阻保护管脚已同步下文 P6 分配；最新管脚和时序由独立工程检查脚本与 `records/2026-10-23_P6_VALIDATION.md` 记录。

ASRPRO 工程暂按 Serial1、PA2/TX、PA3/RX 配置；它们是芯片信号名，不代表实际模块插针编号。模块供电、电平及实物引脚须核对所购型号；本表不授权把 P6 的 5 V 接到模块或 FPGA UART。该工程仅验证收发和模拟任务状态，不驱动电机、不代表真实移动或寻找任务完成。

主工程新增板载 OLED：`oled_scl`=R3（输出）、`oled_sda`=T3（开漏双向），均为 LVCMOS33、3.3 V、无内部上下拉、驱动 8 mA。须启用 CPU/SSPI 作为普通 IO；详见 `tests/docs/SYSTEM_IMU_OLED.md`。

本文档记录 ACG525 的 IMU、电机 P11 接线、前/左/右三路 VL53L0X-V2 的 P6 接线，以及串口和编码器 IO 约束。三路测距与 ASRPRO 独立测试工程的 CST 已同步 2026-10-23 P6 分配，主工程尚未集成测距。

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

## VL53L0X-V2 测距模块（前、左、右三路，P6，2026-10-23）

三个六孔 V2 模块的 SCL、SDA、GPIO1、XSHUT 全部接线，三路独立 I²C 总线均使用 7 位地址 `0x29`。本次 P6 分配替代 2026-10-20 的旧 P11 测距分配；旧版位流对应旧接线，不得用于本表接线。

VIN、GND 使用专用电源接线，不分配 P6/P11 针脚：VIN=3.3 V，GND 与 FPGA 共地。测距模块不再需要从 IMU 的 P11-1 供电处分线。

### 信号接线表

方向相对 FPGA；P6 为通用扩展座子，与 P11 不同。避开编码器 P6-1～4、ASRPRO 串口 P6-5/6，以及座子电源/地针脚。

| 模块位置 | 模块丝印 | RTL 端口 | 相对 FPGA 的方向 | P6 针脚 | 板上网络 | FPGA 封装管脚 |
| --- | --- | --- | --- | --- | --- | --- |
| 前 | SCL | `tof_front_scl` | 开漏时钟，释放时可采样 | P6-7 | GPIO6 | G16 |
| 前 | SDA | `tof_front_sda` | 开漏双向 | P6-8 | GPIO7 | G18 |
| 前 | GPIO1 | `tof_front_gpio1` | 模块 → FPGA（输入） | P6-9 | GPIO8 | F17 |
| 前 | XSHUT | `tof_front_xshut_n` | FPGA → 模块（拉低/高阻释放） | P6-10 | GPIO9 | F18 |
| 左 | SCL | `tof_left_scl` | 开漏时钟，释放时可采样 | P6-13 | GPIO10 | H13 |
| 左 | SDA | `tof_left_sda` | 开漏双向 | P6-14 | GPIO11 | H14 |
| 左 | GPIO1 | `tof_left_gpio1` | 模块 → FPGA（输入） | P6-15 | GPIO12 | K15 |
| 左 | XSHUT | `tof_left_xshut_n` | FPGA → 模块（拉低/高阻释放） | P6-16 | GPIO13 | K16 |
| 右 | SCL | `tof_right_scl` | 开漏时钟，释放时可采样 | P6-17 | GPIO14 | J13 |
| 右 | SDA | `tof_right_sda` | 开漏双向 | P6-18 | GPIO15 | K14 |
| 右 | GPIO1 | `tof_right_gpio1` | 模块 → FPGA（输入） | P6-19 | GPIO16 | K12 |
| 右 | XSHUT | `tof_right_xshut_n` | FPGA → 模块（拉低/高阻释放） | P6-20 | GPIO17 | K13 |

### 电平、实现与核对依据

- 按六孔 V2 实物丝印接线；GPIO1/XSHUT 与附带原理图连接器编号的顺序不同，不按原理图编号数孔。
- VIN 统一 3.3 V，不接 5 V。SCL/SDA 带 MOS 电平转换和上拉；SCL/SDA/XSHUT 仅输出 0/Z，无内部上下拉。XSHUT/GPIO1 直接连接模块内部 VDD，禁止推挽驱动 3.3 V；GPIO1 为同步输入，DRIVE=OFF，需实测确认输入高电平门限。
- `tests/vl53l0x_three/` 主动驱动本表信号；`tests/asrpro_uart/` 同名信号同步迁移并保持高阻保护，GPIO1 无上下拉输入。两套新版位流适配同一接线，主工程尚未集成。
- OLED 保持 R3/T3，调试 UART TX 保持 V8；七路电机控制输出固定低，T11 保持输入、无上下拉、DRIVE=OFF。
- 12 个信号的 P6 针脚、GPIO 网络及封装管脚各不重复，与 IMU、电机、编码器、ASRPRO、OLED、串口及按键的已记录分配无冲突。检查脚本只允许两套工程中明确的同名测距信号复用，拒绝其他用途占用。
- P6 编号至 GPIO 网络依据开发板底板原理图第 5 页，GPIO6～17 至封装管脚依据引脚信息表 Sheet1；资料位置见本文“核对依据”。模块依据工程上两级 `GY-VL53L0X资料/` 的六孔 V2 照片与 `53l0x_SCH.jpg`。
- P6 初次迁移验证结果分别见两套工程 `records/2026-10-23_P6_VALIDATION.md`。测距工程后续 I²C 时序修复与实测见 [I²C 修复记录](tests/vl53l0x_three/records/2026-10-23_I2C_FIX_VALIDATION.md)：修复 SDA 保持时间并纠正右侧接线后，三路均已读到 ID 与原始结果，重新下载启动后各 10/10 次成功，管脚分配不变。2026-10-20 的旧 P11 记录仅作为历史，不代表 P6 验证结果。驱动严格沿用附带 Arduino 诊断流程，缺少完整初始化/校准，实际距离准确度仍待核验。

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

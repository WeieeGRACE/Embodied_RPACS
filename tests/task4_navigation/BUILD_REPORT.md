# 任务四独立板级构建报告

## 构建对象

本次使用保存工程 `task4_navigation_board.gprj`，器件为 `GW5A-25A / GW5A-LV25UG324C2/I1`，保存顶层为 `task4_navigation_single_leg_probe_top`。该顶层只开放 A→OPEN_WAYPOINT_1 的低速单段探针，普通任务四路线默认禁止启动；没有自动烧录。

构建入口：

```powershell
& 'D:\Gowin\Gowin_V1.9.12_x64\IDE\bin\gw_sh.exe' build_board.tcl
& 'D:\Gowin\Gowin_V1.9.12_x64\IDE\bin\gw_sh.exe' verify_board_saved_project.tcl
```

两次流程均完成解析、综合、布局布线、时序分析、位流和功耗报告。保存工程复验重新设置了 `-use_sspi_as_gpio 1`、`-use_cpu_as_gpio 1`，并保持顶层、约束和源文件一致。

## 最新结果

| 项目 | 结果 |
| --- | --- |
| 50 MHz 时钟 | 约束周期 20 ns |
| 实际 Fmax | 57.707 MHz |
| 最差 setup slack | 2.671 ns |
| Setup 违例 | 0 |
| Hold 违例 | 0 |
| 位流 | `impl/pnr/task4_navigation_board.fs` |
| 二进制 | `impl/pnr/task4_navigation_board.bin` |
| 综合网表 | `impl/gwsynthesis/task4_navigation_board.vg` |
| 位流 SHA-256 | `9F0CFC40F0E4214D8B37DE36AE9CA504577C1553F50412E4738BCDDB63491F2D` |
| 二进制 SHA-256 | `B61F05E52877DE70B73AFB1147FAD361F4119C6AF71E30EA56AD306E0B8E89B1` |

完整源文件、网表和产物哈希见 `records/board_build_sha256.json`；时序、管脚和功耗报告位于 `impl/pnr/`。三路 ToF 的 SCL/SDA 以及 OLED SDA 在管脚报告中为开漏配置；专用 SSPI/CPU 管脚复用选项已在两个 Tcl 入口中显式设置。

## 变更与验证

此前速度目标到 PI 控制器之间的宽位乘法导致 Fmax 约 38 MHz、118 个 setup 违例。本次采用窄定点 `9/128` 计数换算，并将航向修正乘法分成两级流水后，路径收敛到 57.707 MHz，最差 setup slack 为 2.671 ns；换算对负数向零截断，避免超过速度上限。RTL 闭环和量化边界测试均通过。

`sim/tb_nav_board_safe_default.v` 增加了驱动边界急停测试：未完成传感器校准时启动请求保持 TB6612 关闭；注入运行请求后，急停异步关断 PWM/STBY，释放急停不会恢复运动，复位后仍保持关闭。

## 交付限制

该位流是**单段探针/架空和标定版本**，不能用于 A→B、A→C、A→D、A→E 或巡检。全路线的 ±50 mm 墙端包络审计仍未证明安全，不能用本次板级构建替代实车路线验收。上板必须按“架空检查→静止 IMU/ToF 标定→开阔地 0.05 m/s 单段”顺序进行；任何擦碰、故障停车或净空无法证明都停止升级。


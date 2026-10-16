# 底盘标定、遥测与寄存器接口

## 职责与集成

`../src/chassis_calibration_telemetry_and_register_interface.v` 是独立的 Verilog-2001 外设，不实例化编码器、PI、安全仲裁或通信模块。现有 Gowin 工程已经引用该文件，无需将 testbench 加入综合文件列表。

本模块管理参数，不计算 PI、不转换几何单位、不驱动电机、不自动标定。外部总线适配器将软核访问转换成下面的同步事务。所有输入必须属于 clk 域；异步数据必须先通过外部握手或 FIFO 保证多字段一致性。rst_n 异步置位复位，释放由上层同步。

`control_enable` 是控制请求，必须与外部安全允许信号共同门控最终驱动。32 位 `fault_in` 为上层统一编码的当前故障位，本模块不规定各位对应哪种传感器，也不取代安全仲裁。`mode` 为 3 位透传值，其含义由上层确定。目标和有效配置在禁用、故障时保留，上电及硬复位清零目标；故障关闭使能，清除后软件需要明确重新使能。

## 同步访问协议

| 信号 | 方向/位宽 | 行为 |
| --- | --- | --- |
| req | 输入 1 | 每个上升沿为 1 接受一笔事务，连续为 1 表示连续事务 |
| write | 输入 1 | 1 写、0 读 |
| addr | 输入 12 | 模块内字节地址，32 位对齐 |
| wdata / wstrb | 输入 32 / 4 | 小端字节写使能：bit0 控制 bits7:0 |
| ack / error | 输出 1 / 1 | 请求采样沿之后一个周期的应答；错误事务也应答 |
| rdata | 输出 32 | 与 ack 同周期有效，读到采样沿之前的状态；写、空闲、错误时为 0 |

例：上升沿 N 之前放置请求，上升沿 N 后 ack/rdata 更新，下游在 N+1 沿采样。req 无需等待 ack 才能发下一笔；如果保持同一写请求为高，会每周期重复执行。复位中不接受请求。上层必须自行避免重复发出一次性命令。

未对齐、未映射、写只读寄存器返回 error=1，不产生该事务的副作用；后台计时、采样、故障锁存继续运行。wstrb=0 的合法写为空操作。普通字段保留字节写入结果，配置提交统一验证；CTRL 及 HOLD 仅使用低字节，忽略保留位。命令寄存器仅读写 bit0，读回 0；命令没有置位时为空操作。

## 控制、命令与状态寄存器

| 地址 | 名称 | 权限 | 内容 |
| --- | --- | --- | --- |
| 000 | VERSION | R | 0x00010000，接口版本 1.0 |
| 004 | CTRL | RW | bit0 使能，bits3:1 模式；使能时仍有当前/锁存故障则整笔拒绝 |
| 008 / 00C | TARGET_SHADOW_L/R | RW | signed32 目标影子，单位 count/测速窗口 |
| 010 | TARGET_COMMIT | W命令 | bit0：原子更新左右目标，产生 target_updated |
| 014 | SNAPSHOT | W命令 | bit0：立即请求快照 |
| 018 | SNAPSHOT_HOLD | RW | bit0：保持当前快照 |
| 01C | CONFIG_COMMIT | W命令 | bit0：统一验证并提交全部标定配置 |
| 020 | SOFT_RESET | W命令 | bit0：产生 soft_reset_req，不重置本模块或外部安全状态 |
| 024 | SNAPSHOT_PERIOD | RW | 周期单位为 clk 拍数；0 禁用；写入重新开始计周期 |
| 028 / 02C | TARGET_ACTIVE_L/R | R | 有效目标 |
| 030 | SNAPSHOT_VALID | R | bit0：已经采集过快照，复位清零 |
| 034 | SNAPSHOT_SEQUENCE | R | 每次成功快照递增，32 位回绕 |
| 038 | TIMESTAMP_MS | R | 从复位开始的毫秒数，32 位回绕 |

这里及以下地址均为十六进制，未列出的空洞保留且访问报错。所有写命令产生单周期事件；连续命令产生连续事件，不插入低电平。配置输出保持稳定直到 config_updated，与更新事件同周期呈现新值；目标输出同理。

目标提交要求左右目标绝对值均不超过有效 max_speed_counts。使用 33 位符号扩展判断，-2147483648 不会绝对值溢出。失败时两个目标都保持原值，error=1。提交不要求已使能，因此软件可先准备目标再使能。

## 标定配置

下表每行是一个 32 位寄存器。影子地址可读写，有效地址只读，普通端口输出为有效配置。所有寄存器复位同时恢复影子与有效默认值。

| 影子/有效地址 | 输出名称 | 有效位宽 | 默认 | 单位/约定 |
| --- | --- | --- | --- | --- |
| 100 / 180 | direction_cfg | 4 | 0 | bit0/1 左右电机反向，bit2/3 左右编码器反向；由外部应用 |
| 104 / 184 | counts_per_rev_l | 32 | 1456 | 四倍频后 count/rev |
| 108 / 188 | counts_per_rev_r | 32 | 1456 | 四倍频后 count/rev |
| 10C / 18C | wheel_diameter_l_mm | 32 | 66 | mm，待实测 |
| 110 / 190 | wheel_diameter_r_mm | 32 | 66 | mm，待实测 |
| 114 / 194 | wheelbase_mm | 32 | 240 | 左右轮中心距 mm，待实测 |
| 118 / 198 | max_speed_counts | 32 | 1000 | 非负目标幅值上限，count/窗口 |
| 11C / 19C | kp_l | signed32 | 0 | Kp 定点整数 |
| 120 / 1A0 | ki_l | signed32 | 0 | Ki 定点整数 |
| 124 / 1A4 | kd_l | signed32 | 0 | Kd 定点整数 |
| 128 / 1A8 | kp_r | signed32 | 0 | Kp 定点整数 |
| 12C / 1AC | ki_r | signed32 | 0 | Ki 定点整数 |
| 130 / 1B0 | kd_r | signed32 | 0 | Kd 定点整数 |
| 134 / 1B4 | gain_shift | 6 | 16 | 有效系数为 signed整数 / 2^gain_shift |
| 138 / 1B8 | pwm_limit | 16 | min(1000,PWM周期) | PWM 上限拍数 |
| 13C / 1BC | pwm_min | 16 | 0 | 非零 PWM 最小有效拍数 |
| 140 / 1C0 | ramp_step | 16 | 10 | 每 PWM 周期斜坡变化拍数 |
| 144 / 1C4 | stall_pwm_threshold | 16 | min(500,默认上限) | 堵转判断 PWM 幅值拍数 |
| 148 / 1C8 | stall_speed_threshold | 32 | 1 | 堵转速度阈值 count/窗口 |
| 14C / 1CC | stall_timeout_ms | 32 | 500 | 堵转持续时间 ms |
| 150 / 1D0 | integral_limit | 32 | 100000 | 外部控制器积分状态的非负幅值上限 |

提交必须满足：

- control_enable=0；direction_cfg≤15；左右每圈计数、左右轮径、轮距均非零。
- max_speed_counts∈[1,2147483647]，当前两个有效目标也必须满足新上限。
- gain_shift∈[0,31]；pwm_limit≤PWM_PERIOD_CYCLES；pwm_min≤pwm_limit。
- ramp_step∈[1,65535]；stall_pwm_threshold≤pwm_limit；stall_speed_threshold≤2147483647。
- stall_timeout_ms 非零；integral_limit≤2147483647；六个有符号增益允许全部 signed32 值。

违反任一条件时 error=1，全部有效配置保持不变，影子保留便于修正。窄字段的高位不会静默截断后通过校验。允许零 PWM 上限，但此时 pwm_min 和 stall_pwm_threshold 也必须为零。

测速窗口由外部定义，目标、实际速度和堵转阈值必须使用同一个窗口。PI 增益的实际运算、积分单位和溢出保护由外部控制器实现；本模块只保存并输出值。更改窗口或控制器单位后必须重新标定。

## 遥测快照

| 地址 | 内容 |
| --- | --- |
| 200 / 204 | signed32 speed_l/r，count/窗口 |
| 208 / 20C | signed32 position_l/r，累计 count |
| 210 / 214 | signed16 pwm_l/r 符号扩展成 signed32，PWM 拍数 |
| 218 / 21C | unsigned32 illegal_count_l/r |
| 220 | status_in 原样 32 位状态 |
| 224 | 本沿置位优先 W1C 处理后的 fault_latched |
| 228 | unsigned32 voltage_mv，mV |
| 22C | signed32 temperature_mc，千分之一摄氏度 |
| 230 | bit0 电压有效，bit1 温度有效 |
| 234 | 快照采样沿之前的 timestamp_ms |
| 238 / 23C | 快照采样沿之前的左右运行毫秒数 |

全部字段在一个沿锁存。无效电压/温度仍保存输入值，但软件必须检查有效位。复位时快照内容和 valid 均为零。

周期 N 从写 PERIOD 的沿之后开始，第 N 个沿请求采样；手动采样不重置周期。手动和周期同沿合并成一次请求。写 PERIOD 与旧周期到期同沿时，旧周期的最后一次采样仍发生，再采用新周期。

HOLD 写入在同一沿即生效：置保持并碰到采样则遗漏一次，释放保持并碰到采样则立即捕获。保持期间不覆盖快照，每次请求使 MISSED_SAMPLES 饱和递增；释放不补采遗漏数据。

多字读取流程：写 HOLD=1 → 读取 VALID、SEQUENCE 和遥测字段 → 写 HOLD=0。首次采样可先禁用周期、写 HOLD=0、触发 SNAPSHOT，再写 HOLD=1 读取。默认不自动保持，单纯读取 SEQUENCE 不会冻结数据。

## 故障与计时

| 地址 | 名称 | 权限/含义 |
| --- | --- | --- |
| 300 | FAULT | R/W1C，32 位故障记录 |
| 304 | FAULT_LIVE | R，当前 fault_in |
| 308 | MISSED_SAMPLES | R，保持期间遗漏采样请求数，32 位饱和 |
| 30C / 310 | RUNTIME_L/R | R，左右运行毫秒数，饱和 |

每沿执行 `fault_latched=(fault_latched & ~清除掩码) | fault_in`，置位优先。有效非零 W1C 事务同时产生 fault_clear_req 和 fault_clear_mask；掩码受字节使能限制，输出请求并不保证外部安全模块已解除故障。持续故障不会被清掉。非零 fault_in 同沿强制关闭使能，优先于 CTRL 写入；仍有锁存故障时不能重新使能。清除故障不清目标、不清模式、不自动使能。

时钟每 CLK_HZ/1000 拍形成毫秒节拍。running_l/r 在该节拍沿采样，为 1 时相应运行计数加一；这是毫秒分辨率统计，短于 1 ms 的运行脉冲可能不计入。运行标志由外部提供，不以 control_enable 推断。TIMESTAMP_MS 自由运行并回绕；运行计数和遗漏计数饱和，复位清零。

参数约束：CLK_HZ≥1000 且可被 1000 整除；PWM_PERIOD_CYCLES∈[1,32767]；RUNTIME_WIDTH∈[1,32]。默认分别为 50 MHz、2500 拍、32 位。RUNTIME_WIDTH 缩小供仿真快速覆盖饱和；端口读回始终零扩展为 32 位。非法参数在仿真报错，参数检查排除在综合之外。

## ModelSim 本地复现

在 sim 目录运行，不覆盖原有 run.do、work 库或 modelsim.ini：

```powershell
Set-Location 'D:\Gowin\Project\Contest\Embodied_RPACS\Embodied_RPACS\Embodied_RPACS\sim'
& 'D:\app\modesilm\win64\vsim.exe' -c -modelsimini chassis_registers_modelsim.ini -l chassis_registers_simulation.log -do run_chassis_registers.do
$LASTEXITCODE
```

GUI 切换到本 sim 目录，执行 `do wave_chassis_registers.do`。批处理保存 `chassis_registers.wlf`；脚本使用专用 ini 和 work_chassis_registers 仿真库。testbench 不加入 Gowin 综合。

2026-10-12 本机 ModelSim SE-64 10.7 实测：**2079 项检查通过，退出码 0，Errors=0、Warnings=0，结束时间 28626 ns**。随机字节写参考模型共 600 轮，固定初始种子 0x134579ab；独立默认参数实例检查上电值。主实例 CLK_HZ=1000、RUNTIME_WIDTH=4 加速计时和饱和验证。

覆盖复位、地址错误和只读保护、连续事务、全部字节通道、零使能写、目标原子提交、负数极值、各类配置拒绝、几何零值、零 PWM 上限、快照符号扩展及有效位、保持和释放、周期 0/1/3、同时手动/周期采样、故障与使能/W1C 竞争、脉冲命令、运行计数饱和、运行中复位。遗漏计数接近 2^32 的边界用 testbench 层次赋值注入，运行计数饱和则使用合法的小位宽参数自然运行覆盖。

失败退出码也经过实际验证：

```powershell
& 'D:\app\modesilm\win64\vsim.exe' -c -modelsimini chassis_registers_modelsim.ini -l chassis_registers_expected_failure.log -do 'set inject_failure 1; do run_chassis_registers.do'
$LASTEXITCODE # 预期为 1
```

`chassis_registers_expected_failure.log` 中的 Fatal 是刻意注入，用于证明脚本不会将断言失败当作通过；正常功能结果见 `chassis_registers_simulation.log`。脚本还检查 completed 标志，testbench 带 1 ms 仿真超时。

本次验证为独立 RTL 功能仿真，未执行 Gowin 综合、布局布线和上板测试；参数不掉电保存，软件需在启动时加载实测参数。

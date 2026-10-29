# 最小避障演示配置方案（MINIMAL_PLAN）

- **记录编号**：resource_20261029（task-2 第二份交付）
- **目标**：在 **不改变演示所必需功能** 的前提下，用最少资源让 `robot_system_top` 满足「任务 3 实时避障演示」要求：三路 ToF + IMU yaw + 编码器 + OLED 面板 + 按键/语音启动。
- **依据**：
  - 用户要求：“工程目录有 plan.md，用最少的资源能做到演示要求就可以”。
  - `D:\Gowin\Project\Contest\Embodied_RPACS\PROJECT_PLAN.md` §8 任务 3：测距/滤波/有效性/减速/停车/绕障状态机/安全状态（CLEAR/SLOW/STOP/SENSOR_FAULT）均为 PL 实现；§13 台架与整车验证要求“姿态显示、直线行驶、定点旋转、自动减速、停车和绕障”。
  - `records/resource_20261029/RESOURCE_AUDIT.md` 的资源基线、逐模块成本与 `12036/53%` 坏基线结论。
- **约束**：本文件为**规划**。审计者全程只读：未改 `src/`、`sim/`、`build.tcl`、`README.md`、`impl/`，未运行综合/PnR，未改 git 状态；本文件之外的写入均无。
- **前置条件（重要）**：Lead 的故障修复（当前工作区 `src/asr/system_motion.v`、`src/obstacle/obstacle_route.v`、`src/obstacle/obstacle_sensors.v` 有 11:53–11:56 的未提交改动）**先落地并完成回归**；裁剪开始前先重构建一次、重取基线（`impl/pnr/Embodied_RPACS.rpt.txt` + `_syn_rsc.xml`）。**本文件所有估算以 11:40 基线（Logic 17661 / Reg 6859 / CLS 9858 / Fmax 53.020）为参照，故障修复后允许 ±0.1–0.3k 偏移。**

---

## 0. 结论速览（TL;DR）

| 结论 | 内容 |
| --- | --- |
| 推荐配置 | **Minimal-A**：删 DISPLAY 组 5 个诊断页 + 删主菜单“下一项预览”行；**IMU 模块不改**（删页后综合自动裁剪 roll/pitch 数据通路）；**保留** OBSTACLE 面板 / MANUAL / SETTINGS / 语音 / 日志。 |
| 改动面 | 仅 **2 个必改 RTL 文件**（`src/ui/system_pages.v`、`src/ui/system_menu.v`）+ 可选 top 清理 1 个；**4 个仿真 TB 需同步**（`tb_system_oled`、`tb_robot_system`、`tb_system_50mhz`、`tb_system_obstacle`）。 |
| 估算收益 | **−2.7～−3.6k Logic**（其中 **849 LUT 为实测锚点**：`tof_page` 595 + `attitude_angle_to_digits` 254；其余为估算）→ Logic ≈ **14.1–15.0k（61–65%）**，CLS ≈ **7.9–8.4k（68–73%）**，FF ≈ **6.15–6.4k**。 |
| 时序 | 当前关键路径（roll 陀螺积分 wrap 链）会被一并裁掉；预计 Fmax **53–58 MHz（估算，需实测）**，不要指望必然提升。 |
| IMU 做法推荐 | **不要先改 `imu_attitude_fixed.v`**：删页后 roll/pitch 不再被消费，综合会级联裁剪其 atan/sqrt/divider 数据通路与 roll/pitch 组合逻辑（预计 −1.0～−1.4k）。仅当实测裁剪不足时，才用 **参数裁剪（`YAW_ONLY` 参数 + generate）**；**不推荐**改端口删单元。 |
| MANUAL / 预览行 / 日志 | 预览行：**删**（~−0.2～0.3k，纯 UI 提示）。MANUAL：**保留**（收益仅 ~0.08–0.15k，但演示定位、任务 2 基础要求与 `tb_robot_system` 4 条手动动作测试都依赖它）。USB 日志：**本轮保留**（上板验收证据依赖），验收通过后删（预研实测 −56 Logic/−107 CLS/−193 FF，BSRAM 9→5、释放 V8/U8）。 |
| 落地后余量 | CLS 余量从 1662 提升到约 **3.1–3.6k**；届时合入 `tests/uart_bridge` 完整链路（881 CLS）约到 77–81% CLS，属于可接受范围。 |

---

## 1. 演示需求 ↔ 模块保留矩阵

| 演示要求（PROJECT_PLAN §8/§13） | 当前实现 | 最小配置处理 |
| --- | --- | --- |
| 三路 ToF 测距/滤波/有效性 | `obstacle_sensors` + 3×`tof_driver`（1836 LUT） | **保留**（核心） |
| IMU yaw（90° 转向、航向保持） | `imu_attitude_fixed` yaw 积分 + `asr_yaw_unwrap` | **保留 yaw**；roll/pitch 显示用数据通路随页面裁剪自动消失 |
| 编码器/测速/PI/PWM/差速 | `dual_ab_quadrature_encoder`、`speed_measurement…`、`dual_speed_pi_pid_controller`、`pwm_and_startup_ramp`、`tb6612_driver_interface` | **保留**（核心） |
| 绕障状态机 + 减速/停车/恢复 | `obstacle_route`、`obstacle_command_buffer`、`motion_control_and_mode_fsm` | **保留**（核心） |
| 模式仲裁/暂停/退出 | `system_dispatch` | **保留** |
| OLED 面板（状态/档位/三路距离） | `system_pages` group 5 + `system_oled_text` | **保留** |
| 按键启动/暂停/退出、语音启动 | `system_keys`、`system_menu`（K1/K2/K0）、`system_menu` group5 K2、ASR 通道 | **保留**；根菜单由 4 项变 3 项 |
| 语音开关 + 三档速度 | `system_menu` group 4（SETTINGS 页） | **保留** |
| 手动定量动作（演示定位/任务 2 基础） | `system_menu` group 2（MANUAL 页）+ motion FSM | **保留**（建议） |
| 现场诊断显示（状态码/姿态角/ToF 五位 mm/编码器计数） | `system_pages` group 1（DISPLAY 5 页）+ `tof_page` + `attitude_angle_to_digits` | **删除**（非演示必需） |
| USB 事件日志/心跳 | `system_event_logger`（321 LUT + 2 逻辑 BSRAM） | **本轮保留**，上板验证后可删 |

> 结论：演示必需功能全部落在“保留”列；被删的只有**现场诊断显示**与**菜单预览提示**，不触碰任何运动/安全/避障/语音路径。

---

## 2. 推荐方案与取舍（含 Lead 提出的第 4 问）

### 2.1 三个候选实施级别

| 方案 | 改动内容 | 改动文件 | 估算收益 | 风险 | 建议 |
| --- | --- | --- | --- | --- | --- |
| **Minimal-A（推荐）** | 删 DISPLAY 组页内容 + 菜单 3 项 + 删预览行；`system_pages` **端口保持不变**，由综合自动裁剪上游；IMU 模块零改动 | `system_pages.v`、`system_menu.v`（+top 可选） | −2.7～3.6k Logic | 低-中（主要是 TB 更新与菜单序号） | **先做这个** |
| Clean-B | 在 A 基础上彻底清理：删 `system_pages` 无用端口、删 top 的 `tof_page`/`tof_snapshot`/死连线、删 `attitude_angle_to_digits`/`tof_page` 文件引用、删 pages 内死函数/寄存器 | A 的 3 个 + `robot_system_top.v`、`system_motion.v`（端口） | 与 A 相同（0 额外收益） | 中（多文件、仿真 compile 列表） | A 验证通过后的清理项，不单独作为优化 |
| 显式裁姿态（3C，不推荐先做） | 删 atan/sqrt/divider 实例、改 `imu_attitude_fixed`/`system_motion` 端口、重写 attitude 状态机 | 3+ 文件 + `tb_imu_attitude_fixed`/`tb_attitude_pipeline` | 仅比 A 多 ~0.1–0.3k | 高（姿态 FSM 重写、7 套件之一需重写） | 仅当 A 的自动裁剪不达预期时考虑 |

### 2.2 为什么“只删页面、不专门裁 IMU”就够（关键结论）

`roll_mdeg`/`pitch_mdeg`（以及 `pos_l/pos_r/speed_l/speed_r`、`status` 等）当前的**唯一消费者就是 `system_pages` 的诊断页**：
- `system_motion` 只把它们送出到 top，top 只接到 `pages`；
- 90° 转向/航向保持只用 **yaw**（`yaw_wrapped` → `asr_yaw_unwrap` → `obstacle_route`/motion FSM），与 roll/pitch 无关；
- 因此删掉诊断页后，这些信号在 RTL 里变成“无负载”，50 MHz 时序驱动综合会做标准 dead-logic 级联裁剪：`atan_unit`/`sqrt_unit`/`divider_unit` 的数据通路（保留其 `done` 供 FSM 时序）、roll/pitch 误差/融合/回绕网络、`imu_attitude_bias_divider` 的 x/y 通道寄存器会一起被裁掉。
- **反过来说：“删页面但保留 roll/pitch 计算”在物理上做不到**——除非给这些逻辑加 `syn_preserve` 人为保留（纯粹浪费资源）。所以真正的选择只有“让综合自动裁（A）”还是“显式重写（3C）”，而 A 的收益几乎等于 3C（3C 只是把 FSM/端口也精简掉，多收 0.1–0.3k）。

### 2.3 Lead 第 4 问的直接回答

| 比较项 | A：只删页面（自动裁剪） | B：参数裁剪（YAW_ONLY） | C：改端口删单元 |
| --- | --- | --- | --- |
| 收益 | 约 1.0–1.4k（attitude 侧） | 与 A 相同或略多 0.05–0.15k | 比 A 多 ~0.1–0.3k |
| 改动 | 0 行 attitude | +5～15 行（参数 + generate + 顶层传参） | 删单元 + 改 3 个端口文件 + 重写 TB |
| 仿真影响 | `attitude_pipeline`/`imu_attitude` **完全不用改** | 默认参数保持原行为，**现有 TB 也不用改**；另需新增 YAW_ONLY=1 的用例 | 两个 TB 必须重写/删套件 |
| 风险 | 低（依赖综合，必须实测验证） | 低-中 | 高 |
| 推荐 | **首选，先做** | **若 A 实测不足，作为 fallback** | 不推荐 |

> 结论：**推荐 A；B 作为兜底；C 不划算。** A 需要一次真实构建来确认自动裁剪幅度（看综合日志的 unused/swept 警告 + 资源差值），这是本方案唯一的“未知量”。

---

## 3. 精确改动清单

### 3.1 Step 1 —— 删主菜单“下一项预览”行（独立、最小、先做）

**文件：`src/ui/system_pages.v`**

| 位置 | 现状 | 改为 |
| --- | --- | --- |
| L105-110（非 page 分支准备） | `r0=…;r1=r0;r2=r0;r3=r0;current_label=label(group_id,item);`、`next_item=item==last_item ? 0:item+1;` | 保留；`next_item` 可一并删除（L106、L110） |
| **L118** | ` r2=label(group_id,next_item);` | **删除该行**（`r2` 已在 L108 初始化为 21 空格） |

- 效果：菜单第 3 行（row 2）由“下一项名称”变为空行；按键/功能不变。
- TB 同步：`sim/tb_system_oled.sv` L100-101 的 `check_row(2,"MANUAL")` 改为 `check_row(2,"")`（或删掉该行检查）。
- 估算收益：**−0.2～0.3k LUT**（一个 168-bit×7-bit 字面量 mux 实例）。
- 单独回归：`run_system_oled` + `run_system` + 构建，占用半天以内。

### 3.2 Step 2 —— 删除 DISPLAY 组 5 个诊断页（主体收益）

#### A. `src/system_menu.v`（根菜单 4 项 → 3 项；菜单序号映射）

| 行 | 现状 | 改为 |
| --- | --- | --- |
| L12-15 `last_item` | `0:3;1:4;2:4;5:0;default:1` | `0:2;2:4;4:1;5:0;default:0`（删 group1；根菜单 3 项；group4=1） |
| L25（K0 从 OBSTACLE 退出） | `group_id<=0;item<=3;page<=0;` | `group_id<=0;item<=2;page<=0;` |
| L27（K0 从子菜单返回） | `item<={1'b0,group_id}-4'd1;group_id<=0;` | `item<= group_id==2 ? 4'd0 : group_id==4 ? 4'd1 : 4'd2; group_id<=0;` |
| L34-36（根菜单 K2 选择） | `group_id<=item==2?4'd4:(item==3?4'd5:{1'b0,item}+4'd1); item<=0; if(item==3)page<=1;` | `group_id<= item==0 ? 4'd2 : item==1 ? 4'd4 : 4'd5; item<=0; if(item==2)page<=1;` |
| L49 `show_root` | `group_id<=0;item<=3;page<=0;` | `group_id<=0;item<=2;page<=0;` |
| L38-46（页内 K2 命令） | group5→0x16；group2→0x10–0x13/0x02；group4→语音/档位提交 | **不变**（group2/4/5 保留；group1 页原本无 K2 命令） |

> 新根菜单（K1 循环、K2 进入）：**item0 = MANUAL（→group 2）→ 进入手动动作列表（page=0）**；item1 = SETTINGS（→group 4）；item2 = OBSTACLE AVOID（→group 5，直接打开面板 page=1）。语音进入避障/退出仍用 `show_avoid`/`show_root`，只改其根 item 落点（2）。

#### B. `src/ui/system_pages.v`（页面与标签）

| 行/区域 | 现状 | 改为 |
| --- | --- | --- |
| L16-45 `label()` | group0: DISPLAY/MANUAL/SETTINGS/OBSTACLE AVOID（4 项）；group1: 5 个诊断页名 | group0 改 3 项：`MANUAL`/`SETTINGS`/`OBSTACLE AVOID`；**删除 group1 整段**；group2/4/5 标签保留 |
| L105-110 `last_item` | `0:3;1:4;2:4;5:0;default:1` | `0:2;2:4;4:1;5:0;default:0` |
| L111-119 非 page 分支 | `r0=group_id==0?"RPACS MENU":group_id==4?"SETTINGS":group_id==5?"OBSTACLE AVOID":label(3'd0,{1'b0,group_id}-4'd1);` | `r0= group_id==0 ? "RPACS MENU" : group_id==2 ? "MANUAL" : group_id==4 ? "SETTINGS" : "OBSTACLE AVOID";`（不再经 label 间接取 group0 标签） |
| **L123-147** | `case(group_id) 1:case(item) … endcase`（5 个诊断页：系统状态/IMU 姿态/ToF 距离/编码器速度/语音运动） | **整段删除**（这是本方案最大收益点之一） |
| **L175** | `if(page && group_id==1 && item==2)text=tof_text;` | **删除**（TOF DISTANCE 页与 `tof_text` 输入不再被消费） |
| **L63-104** | 姿态/速度格式化数据通路：`roll_q/pitch_q/yaw_q`、`att_conv/att_value`、`ad` 实例、`roll_text/pitch_text/yaw_text`、`speed_sel/sel_d/speed_conv`、`left/right_speed_text` | **建议直接删除**（这些信号只服务于已删的诊断页；不删也会被综合裁剪，但直接删可**保证**收益、便于评审；`attitude_angle_to_digits.v` **文件保留**，`tb_attitude_pipeline`/`tests/imu_attitude_oled` 仍用它） |
| L46-52、L53-62、L77-80 | `hex32`（仅编码器页用）、`signed_speed`（仅速度页用）、`angle`（仅姿态页用） | 删除（同上；`hexchar` 仍被 OBSTACLE 面板使用，**保留**） |
| 端口（Minimal-A） | `roll_mdeg/pitch_mdeg/yaw_mdeg/tof_text/pos_l/pos_r/speed_l/speed_r/status/imu_error/sensor_error/motion_fault/command/response/imu_ready/calibrating/attitude_valid` | **保持不变**（让综合裁剪上游）；Clean-B 才删端口 + 同步 top |

> 保留的页面内容：group 2 MANUAL（5 项动作）、group 4 SETTINGS（语音开关/档位，2 项）、group 5 OBSTACLE 面板（状态/档位/三路距离/按键提示）、根菜单 3 项。

#### C. `src/robot_system_top.v`（Minimal-A 可不动；推荐 Step 2b 再清理）

| 行 | 现状 | 建议 |
| --- | --- | --- |
| L100-105 | `wire [671:0] tof_text,text; reg [671:0] tof_snapshot; always…; tof_page tof_display(…)` | Step2 后可删（tof_page 失去消费者）；保留也不产生资源成本（自动裁剪） |
| L113 `.tof_text(tof_snapshot)` | pages 端口仍在 | 若删 tof 相关，则改 `.tof_text({672{1'b0}})`（Minimal-A 端口保留）；Clean-B 直接删端口 |
| 其它 pages 连接 | `roll_mdeg/pitch_mdeg/yaw_mdeg/pos_*/speed_*` | 可保留（自动裁剪）；若首轮实测裁剪不足 → 改为常量（`.roll_mdeg(32'sd0)` 等）强制裁剪，见 Step 2b |

**不动的文件（重要）**：`build.tcl`、`Embodied_RPACS.gprj`、`src/robot_system_top.cst/.sdc`、`verify_project.tcl`、`tools/*.ps1`、`sim/compile_system.do`。**引脚/时钟/器件/顶层设置全部不变**，`tools/check_system_reports.ps1` 的 44 引脚与单 owner 检查应原样通过。

### 3.3 Step 3（条件触发）—— 若自动裁剪不足，显式 yaw-only

**触发条件**：Step 2 构建后 `imu_attitude_fixed` 的 logic 仍 > 2.2k（当前 3272），或综合日志显示 atan/sqrt/divider 未被裁剪。

- **3B（推荐 fallback，参数裁剪）**：
  - `src/imu/imu_attitude_fixed.v`：新增 `parameter integer YAW_ONLY = 0`；用 `generate if (YAW_ONLY)` 把 `IDLE→ROLL_START→…→BLEND` 的加速度校正分支替换为“直接 `yaw_mdeg<=wrap_angle(yaw_mdeg+step_z)`、`update_valid<=1`、`attitude_valid<=1`”的简化序列；保留陀螺零偏校准（可保留三轴以复用现有校准 FSM，或只留 gz）。
  - `src/asr/system_motion.v`：实例化处加 `.YAW_ONLY(1)`。
  - 默认值 0 保证 `tb_imu_attitude_fixed`、`tb_attitude_pipeline` 等现有 TB 行为不变；主工程用 1。
  - 估算额外收益：0.1–0.2k；风险低-中。
- **3C（不推荐）**：删实例 + 删 `roll_mdeg/pitch_mdeg` 端口 + 改 `system_motion`/top + 重写/删除 attitude 相关 TB（`sim/tb_imu_attitude_fixed.sv`、`sim/tb_attitude_pipeline.sv`、`run_attitude_pipeline.do`、`tools/run_system_sim.ps1` 的套件列表）。收益仅比 3B 多 0.1–0.3k，风险与工作量明显更高。

#> **2026-10-29 题目要求变更**：恢复 IMU 姿态页（ROLL/PITCH/YAW）+ MANUAL SPIN 360 后，
> post-PnR 实测 Logic 15910/23040 (70%)、CLS 9614/11520 (84%)、Fmax 56.049 MHz。
> 资源预算按“满足题目要求后的最小演示配置”更新；PASS 中途截断兜底（场景 11）另计。

## 3.4 Step 4（可选，上板验证通过后）—— 删 USB 事件日志

- `src/robot_system_top.v`：删 `system_event_logger` 实例（L86-89）与 `debug_state/debug_valid/event_errors` 连线；按用户确认采用**删端口+约束**方案：删 `uart_tx`/`uart_rx` 顶层端口、`src/robot_system_top.cst` 的 `IO_LOC "uart_tx" V8` 与 `IO_LOC "uart_rx" U8`、`tools/check_system_reports.ps1` 的引脚期望（44→42）。
- `src/asr/system_motion.v`：删 `debug_state/debug_valid/event_errors` 端口与组装（L128-131、L286-289）。
- 收益（预研变体实测）：**−56 Logic（13420→13364）、−107 CLS（8304→8197）、−193 FF、BSRAM 9→5、
  Fmax 58.287→61.113 MHz**；原“−0.42～0.45k LUT”估算偏高，实际主要节省是 4 个 BSRAM、FF 与布线压力。
- **功能轮基线复测（2026-10-29 22:17，`.tmp_nolog2` 临时工程，同一 build.tcl 参数）**：把“删日志 +
  删调试串口 + 删 `system_motion` 调试端口”应用到当前功能版源码，实测 **Logic 15498/23040 (68%)、
  Register 6543、CLS 9404/11520 (82%)、BSRAM 5/56、DSP 8.5/28、Fmax 57.115 MHz、0 违例、
  42 个受约束引脚**；相对主位流（15910/70%、6746、9614/84%、9、56.049 MHz）即 **Logic −399、
  CLS −236、FF −193、BSRAM −4、Fmax +2.1 MHz**，CLS 余量从 1880 提升到约 2116。交付位流未改动；
  验收后仍需按 `verify_project.tcl` 重建并同步改仿真 TB 的 uart 端口。
  证据：`records/resource_20261029/nolog_variant_features_{build,resources}.log`。
- **执行时机（2026-10-29 用户确认）**：上板 3–5 次连续避障验收通过后执行。README 的板级验收与
  `records/obstacle_20261025/board_smoke_final.log` 依赖 USB 心跳/事件行，验收前删会失去串口取证能力。
  执行时采用“删端口+约束”方案：移除 `uart_tx`/`uart_rx` 顶层端口与 `src/robot_system_top.cst`
  的 V8/U8 约束，释放两个 FPGA 球；`tools/check_system_reports.ps1` 引脚期望 44→42；
  同步更新 `PIN_ASSIGNMENT.md`、`PROJECT_PLAN.md` §16.4 与 `TECHNICAL_REPORT.md`。
- **执行方式（2026-10-29 复核）**：**不要直接合并旧分支 `post-acceptance-no-logger`（`d09f097`）**——
  它基于功能轮之前的 main，与当前 main 在 `src/asr/system_motion.v`、`tests/p6_uart_debug/README.md`
  及历史日志处冲突（`git merge-tree` 已实测）。验收通过后应在最新 main 上按本节步骤重新删减、
  重建位流并复测资源，旧分支仅作改动清单参考；收益数字以验收后重建为准。

### 3.5 Step 5（不建议）—— 删 MANUAL 组

- 收益仅 **−0.08～0.15k**；代价：`sim/tb_robot_system.sv` 的 4 条手动动作测试（L262 `manual_action(0..3)`、L230-240 任务）全部要删/改，演示失去“按键定量前进/后退/90°”定位能力，且任务 2（运动控制基础）验收受影响。
- 如用户明确“只要任务 3 演示、不要手动页”，可作为最后手段；否则**保留**。

---

## 4. 仿真影响与必须同步更新的文件

### 4.1 七个套件影响总表

| # | 套件（`tools/run_system_sim.ps1`） | 脚本 | 受影响 | 需要更新的内容 |
| --- | --- | --- | --- | --- |
| 1 | `system` | `sim/run_system.do` → `tb_robot_system.sv` + `tb_system_event_logger.sv` | **是** | `tb_robot_system.sv`：L51 `menu_index` 改 `2:0;4:1;5:2`；L247 期望 `group_id==2 && !page`（原 group1）；L250 期望 `group_id==2 && page`（原 group1/page）；L297-298 删除 `choose(1,2)` 与 `dut.text[…]` 的 TOF 页 stale 检查（页面已删，改为保留 L296/L299 的 ToF 故障与运动检查）。`tb_system_event_logger.sv` **不改** |
| 2 | `system_oled` | `run_system_oled.do` → `tb_system_oled.sv` | **是（最大）** | 见 §4.2 逐行 |
| 3 | `system_50mhz` | `run_system_50mhz.do` → `tb_system_50mhz.sv` | **是** | L133 `repeat(2)` → `repeat(1)`（根菜单 SETTINGS 由 item2 变 item1）；其余不变 |
| 4 | `system_dispatch` | `run_system_dispatch.do` → `tb_system_dispatch.sv` | 否 | 直接驱动 `system_dispatch` 的 `menu_valid/menu_command`，不经过 `system_menu`/`system_pages`；命令码未变，无需改 |
| 5 | `live_speed` | `run_live_speed.do` → `tb_live_speed.sv` | 否 | 纯测速/滤波，不涉及 UI |
| 6 | `system_obstacle` | `run_system_obstacle.do` → `tb_system_obstacle.sv` | **轻微** | L51 `menu_index` 同步（`2:0;4:1;5:2`）；L349 `choose(4,0)` 等由该函数自动适配；其余（攻障/语音/路线）不变 |
| 7 | `attitude_pipeline` | `run_attitude_pipeline.do` → `tb_attitude_pipeline.sv` | **否**（方案 A/3B 默认参数） | 仅方案 3C 需要重写或删除该套件 |

### 4.2 `sim/tb_system_oled.sv` 逐行更新（最大改动）

| 行 | 现状 | 改为 |
| --- | --- | --- |
| L6 | `wire [671:0] text,tof_text;` | 可保留（Minimal-A 下 pages 端口仍在）；Clean-B 时改 `wire [671:0] text;` |
| L10 | `tof_page page(…);` | 诊断页删除后无意义，建议删除（或保留无害） |
| L17 | `.tof_text(tof_text)` | Minimal-A 保留；Clean-B 删 |
| L100 | `check_row(0,"RPACS MENU");…check_row(1,">DISPLAY");check_row(2,"MANUAL");` | 新根菜单：`check_row(0,"RPACS MENU");check_row(1,">MANUAL");check_row(2,"");check_row(3,"K0 RST K1 NEXT K2 OK");` |
| L104-107 | MANUAL 页（group2 item3 RIGHT 90 DEG）切换/tear-free 测试 | **保留**（MANUAL 页未删；group2/item3 不变） |
| L108-111 | SETTINGS 页测试（group4） | **保留**（不变） |
| L112 | `group_id=1;item=1;…ROLL/PITCH/YAW` | **删除**（IMU ATTITUDE 页已删） |
| L113 | `item=3;…L:/R:` 编码器页 | **删除** |
| L114-117 | TOF DIAG 页（`tof_page` 文本、stale 掩码） | **删除**；`dump(...system_menu_tof.hex)` 一并删除 |
| L119 | `group_id=0;item=3;…">OBSTACLE AVOID"` | 改 `item=2`（根菜单第 3 项） |
| L120-125 | OBSTACLE 面板全部检查 | **保留**（不变） |
| L101/L110 的 dump | 写 `records/main_20261023/system_menu_root.hex` / `..._voice.hex` | 保留（重新生成，内容随菜单变）；`system_menu_tof.hex` 删除 |
| L128 | 看门狗 | 不变 |

### 4.3 必须同步更新的文件清单（实施时核对）

**RTL（必改）**：`src/ui/system_pages.v`、`src/ui/system_menu.v`
**RTL（Step 2b/条件触发才改）**：`src/robot_system_top.v`（删 tof_page/tof_snapshot 或 tie0）、`src/imu/imu_attitude_fixed.v` + `src/asr/system_motion.v`（仅 Step 3B）
**仿真（必改）**：`sim/tb_system_oled.sv`、`sim/tb_robot_system.sv`、`sim/tb_system_50mhz.sv`、`sim/tb_system_obstacle.sv`
**仿真（不改）**：`sim/tb_system_dispatch.sv`、`sim/tb_live_speed.sv`、`sim/tb_attitude_pipeline.sv`、`sim/tb_system_event_logger.sv`、`sim/compile_system.do`、各 `sim/run_*.do`
**工具/构建（不改）**：`build.tcl`、`Embodied_RPACS.gprj`、`src/robot_system_top.cst`、`src/robot_system_top.sdc`、`verify_project.tcl`、`tools/run_system_sim.ps1`、`tools/check_system_reports.ps1`、`tools/run_legacy_regression.ps1`
**产物**：`records/main_20261023/system_menu_root.hex`、`system_menu_voice.hex` 由 TB 重新生成；`system_menu_tof.hex` 删除

> 提示：`tb_system_oled.sv` 断言的是**整行 OLED 像素**（`check_row` 用 golden 字模逐列比对），因此删页后必须让该 TB 的期望与新页面完全一致；这是本方案最容易出错的地方，建议**先改 TB 期望、再改 RTL**（或同 commit 改完立即跑 `run_system_oled.do`）。

---

## 5. 资源预测（全部为估算，除非标注“实测锚点”）

### 5.1 基线

- 参照基线（11:40 构建）：**Logic 17661（14866 LUT + 2795 ALU）/ Register 6859 / CLS 9858（86%）/ BSRAM 9 / DSP 8.5 / Fmax 53.020 MHz / 0 违例**。
- 故障修复构建后须重取基线；下列收益按“相对新基线”使用。

### 5.2 分步收益

| 步骤 | 内容 | Logic 节省（估算） | CLS 节省（估算） | Register | DSP | BSRAM | 依据 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Step 1 | 删主菜单“下一项预览”行 | **−0.20～0.30k** | −0.11～0.17k | 0 | 0 | 0 | 估算（一个 168-bit×7-bit label mux） |
| Step 2 | 删 DISPLAY 组（页内容+标签+菜单 3 项；含自动裁剪） | **−2.5～3.3k** | −1.4～1.8k | −0.45～0.7k | −2（可能） | 0 | 其中 **849 LUT 为实测锚点**（`tof_page` 595 + `attitude_angle_to_digits` 254，二者唯一消费者被删，必消失）；**attitude 自动裁剪 1.0–1.4k 与 pages 内部 0.6–1.0k 为估算** |
| **合计 Minimal-A** | Step 1+2 | **−2.7～3.6k** | −1.5～2.0k | −0.45～0.7k | 6.5–8.5 | 9（不变） | — |

### 5.3 最小配置后的预测（估算）

| 指标 | 现状 | Minimal-A 后（估算） | 说明 |
| --- | --- | --- | --- |
| Logic | 17661（77%） | **14.1–15.0k（61–65%）** | 按 1.79 logic/CLS 折算 CLS；实际以重综合为准 |
| CLS | 9858（86%） | **7.9–8.4k（68–73%）** | 余量约 3.1–3.6k CLS |
| Register | 6859（29%） | ≈6.15–6.4k（26–27%） | pages/attitude 无用寄存器被裁 |
| DSP | 8.5 | 6.5–8.5 | x/y 陀螺常数乘法（2×MULTALU27X18）可能随 roll/pitch 一起被裁；平方器仍用于 `accel_norm_valid` 控制，可能保留 |
| BSRAM | 9 | **9（不变）** | 字体 pROM、IMU 配置 ROM、日志 debug FIFO 均保留 |
| Fmax | 53.020 MHz | **53–58 MHz（估算，需实测）** | 当前最差路径（roll 陀螺积分 wrap，slack 1.14 ns）将消失；新最差路径预计是 **yaw wrap 链或速度换算乘法链**（后者 slack 1.995 ns），不能保证提升 |

### 5.4 可选后续（上板验证后）

| 步骤 | 内容 | 额外收益（估算） | 之后 Logic | 之后 CLS | BSRAM |
| --- | --- | --- | --- | --- | --- |
| Step 4 | 删 USB 事件日志 | 实测 −56 Logic、−107 CLS、−193 FF、BSRAM −4 | 13364（58%） | 8197（72%） | 9→5 |
| Step 5（不建议） | 删 MANUAL 组 | −0.08～0.15k | ≈13.6–14.5k | ≈7.5–8.0k | 5 |

> 若后续还要合入 `tests/uart_bridge` 完整链路（实测 1174 Logic / 881 CLS）：Minimal-A 后 CLS 约 8.0–8.4k + 881 ≈ **8.9–9.3k（77–81%）**，空间充足；而**当前基线（86%）直接合会到 93%**，故“先裁剪、后合入”的顺序不可颠倒。

---

## 6. 实施顺序与每步回归（每步可独立回退）

> 前提：**Lead 的故障修复已提交并通过 7 套仿真 + 构建**；本方案在修复后的干净基线上执行。每步一个独立 commit（Lead 执行），便于回退与归因。

### Step 0 —— 重取基线（只读）
1. 故障修复构建完成后，记录新 `impl/pnr/Embodied_RPACS.rpt.txt`（Logic/Reg/CLS/BSRAM/DSP）+ `Embodied_RPACS_tr_content.html`（Fmax/TNS）+ `_syn_rsc.xml` 逐模块值。
2. 与本文件 §5.1 基线对照；若偏差 >0.3k，按新基线等比修正后续估算（本方案结论不变）。

### Step 1 —— 删预览行
1. 改 `src/ui/system_pages.v`（删 L118；必要时删 `next_item`）。
2. 改 `sim/tb_system_oled.sv` 根菜单 row2 期望。
3. 回归：`vsim … run_system_oled.do`、`run_system.do`；构建；记录 Δ1（对照基线）。
4. 上板可选：菜单显示正常（第 3 行空白）。

### Step 2 —— 删 DISPLAY 组
1. 按 §3.2 改 `system_pages.v` + `system_menu.v`（+ 可选 top 清理）。
2. 同批改 4 个 TB：`tb_system_oled`（§4.2）、`tb_robot_system`（L51/L247/L250/L297-298）、`tb_system_50mhz`（L133）、`tb_system_obstacle`（L51）。
3. **先跑 `run_system_oled.do`**（最敏感），再 `run_system.do`、`run_system_50mhz.do`、`run_system_obstacle.do`；其余 3 套应直接通过。
4. 构建；**检查综合日志**：应出现 `system_pages`/`tof_page`/`attitude_angle_to_digits_pipeline` 相关标注 "unused/swept"（若未被裁，综合会保留，说明自动裁剪不达预期）；记录 Δ2。
5. **Step 2b（条件）**：若 Δ2 明显小于 2.5k 或 attitude 仍 >2.2k：在 `robot_system_top.v` 把 pages 的 17 个无用输入 tie 常量（`32'sd0` 等/`{672{1'b0}}`），再构建比较 Δ2b；若仍不足 → 进入 Step 3。

### Step 3 —— 条件触发：参数裁剪 YAW_ONLY（推荐 fallback）
1. 仅在 Step 2/2b 后 attitude 仍 >2.2k 时执行 §3.3 的 3B。
2. 回归：`run_attitude_pipeline.do`（默认参数路径应不变）、`run_system.do`、`run_system_obstacle.do`；构建记录 Δ3。
3. 不要使用 3C（改端口删单元）。

### Step 4 —— 上板演示验收
按 §8 清单做板级冒烟；通过后**再**考虑删 USB 日志（Step 4）。

### 每步统一验收门槛（全部满足才进入下一步）
- `./tools/run_system_sim.ps1`：7 套日志尾行 `Errors: 0, Warnings: 0`；
- `& 'D:/Gowin/Gowin_V1.9.12_x64/IDE/bin/gw_sh.exe' build.tcl`：0 error、无新增 swept/black-box/未用关键告警；
- `./tools/check_system_reports.ps1`：PASS（44 引脚、单 owner、open-drain、0 时序违例、位流新鲜）；
- `impl/pnr/Embodied_RPACS_tr_content.html`：Setup/Hold 违例 0、TNS 0、**Actual Fmax ≥ 50 MHz**（建议 ≥52 MHz 留余量）；
- 资源：记录 `rpt.txt`/`_syn_rsc.xml` 的实测 Δ，与本文件估算对比（**以实测为准**）。

---

## 7. 风险、回退与注意事项

| 风险 | 影响 | 缓解/回退 |
| --- | --- | --- |
| 菜单序号 off-by-one（根菜单 4→3 项、K0 返回映射） | 上板按键进错页面/退出行为异常 | TB 断言 + 上板逐键测试（K1/K2 循环、K0 返回、OBSTACLE 直接开面板、语音进入/退出）；RTL 改动集中在 `system_menu.v` 一个文件，回退容易 |
| 综合自动裁剪不达预期 | 实际收益低于估算 | Step 2b tie 常量强制裁剪 → Step 3B 参数裁剪；**不签字估算，只认报告** |
| `tb_system_oled` 整行像素断言大面积失效 | 仿真红、拖时间 | 按 §4.2 先改期望再改 RTL；可先只跑该套件定位 |
| `system_pages` 端口保留但上游被裁 → 综合告警 | 告警噪声/困惑 | 属正常死逻辑裁剪；Clean-B 时删端口与死代码 |
| 演示功能回退（面板/设置/手动/语音） | 演示失败 | 保留矩阵（§1）已明确保留项；每步上板冒烟覆盖 |
| 在故障修复未回归时裁剪 | 无法归因、可能掩盖故障 | 严格执行 Step 0 前置条件 |
| 误改引脚/时钟/器件 | 板级不工作 | 明确不动 `build.tcl`/`.gprj`/`.cst`/`.sdc`/`verify_project.tcl` |

**回退策略**：每步独立 commit；Step 1/2 的 RTL 改动仅 2 个文件 + 4 个 TB，回退成本低；Step 3B 独立文件可单独回退；`git revert` 即可恢复原页面（原代码逻辑未删除的部分都在 git 历史中）。

---

## 8. 演示验收清单（上板）

| # | 验收项 | 期望 |
| --- | --- | --- |
| 1 | 上电/复位 | 主菜单 **3 项**：MANUAL / SETTINGS / OBSTACLE AVOID；USB 每秒心跳（本轮保留日志） |
| 2 | K1/K2 导航 | K1 循环 3 项；K2 进入 MANUAL 动作列表 / SETTINGS / OBSTACLE 面板；K0 逐级返回 |
| 3 | MANUAL | 前进 20cm / 后退 20cm / 左转 90° / 右转 90° / STOP 五项仍可执行（`tb_robot_system` 同项已过） |
| 4 | SETTINGS | 语音 ON/OFF、三档速度确认生效 |
| 5 | OBSTACLE 面板 | K2 进入面板；未开语音显示 `ENABLE VOICE FIRST`；三路距离四位十六进制（不可用 `----`）；状态/档位正确 |
| 6 | 语音启动 | “避障模式”播报完成后前进；K2 暂停/恢复；K0 退出并返回主菜单 |
| 7 | 实时避障 | 前方障碍减速/停车/选择通行侧/转向/沿障/越过后回线；面板状态正确；电机安全 |
| 8 | 三路 ToF + IMU + 编码器联动 | 距离刷新正常、90° 转向角度正确、速度闭环正常 |
| 9 | 安全 | K0 立即关断；故障锁存与复位；无 USB 控制心跳依赖 |
| 10 | 时序/资源 | Fmax ≥ 50 MHz、0 违例；Logic/CLS 记录在案（与 §5 估算对照） |

---

## 9. 附录

### 9.1 与 RESOURCE_AUDIT 的对应关系

- 错误数字排除：`12036/53%` 是 `font_byte` 无驱动导致的坏网表（RESOURCE_AUDIT §9），本方案全部以 17661/77% 为基线。
- 逐模块成本、子系统分桶、Tier A 结构优化（ToF FSM/测速/PI/斜坡共享，约 1.2–1.4k）见 `RESOURCE_AUDIT.md` §4–§7；**本最小演示方案未包含这些结构改造**——若 Minimal-A 后仍需更多余量（例如为任务 4/uart_bridge/视觉预留），再按 RESOURCE_AUDIT 的 Tier A 逐项实施（每项需独立回归）。
- 时序关键路径数据（roll 链 slack 1.140 ns、速度换算链 1.995 ns）见 RESOURCE_AUDIT §10。

### 9.2 当前源码行号索引（实施时以实际文件为准）

| 文件 | 关键位置 |
| --- | --- |
| `src/ui/system_pages.v` | `label()` L16-45；`hex32` L49-52；`signed_speed` L53-62；姿态/速度转换 regs L63-104；`last_item` L109；非 page 分支 L111-119；`case(group_id) 1:` 诊断页 L123-147；`text` 组装 L174-175 |
| `src/ui/system_menu.v` | `last_item` L12-15；K0 处理 L23-27；K1 处理 L28-31；K2 根选择 L32-37；页内 K2 命令 L38-46；`show_avoid` L48；`show_root` L49 |
| `src/robot_system_top.v` | `tof_text/tof_snapshot/tof_page` L100-105；`pages` 实例 L106-113；`display` 实例 L114-116 |
| `sim/tb_system_oled.sv` | 根菜单期望 L100-101；MANUAL 页 L104-107；SETTINGS L108-111；诊断页 L112-118；OBSTACLE L119-125 |
| `sim/tb_robot_system.sv` | `menu_index` L50-52；`root/choose` L45-57；菜单键测 L242-250；手动动作 L230-240、L262；TOF 页文本检查 L297-298 |
| `sim/tb_system_50mhz.sv` | 菜单开语音 L132-138 |
| `sim/tb_system_obstacle.sv` | `menu_index` L50-52；`choose` L53-61；设置页惯用序列 L349 |

### 9.3 只读复现命令（不触发构建）

```powershell
# 基线资源 / 时序
Select-String -Path impl/pnr/Embodied_RPACS.rpt.txt -Pattern 'Logic|Register|CLS|BSRAM|DSP|Created Time'
Select-String -Path impl/pnr/Embodied_RPACS_tr_content.html -Pattern 'Actual Fmax|Violated Endpoints|Endpoints TNS'
# 逐模块资源
Get-Content impl/gwsynthesis/Embodied_RPACS_syn_rsc.xml
# 方案涉及的源码位置（只读查看）
Select-String -Path src/ui/system_pages.v -Pattern 'label|last_item|group_id==1|tof_text'
Select-String -Path src/ui/system_menu.v -Pattern 'last_item|item<=|show_root|show_avoid'
# 仿真影响（只读查看）
Select-String -Path sim/tb_system_oled.sv -Pattern 'check_row|group_id|item'
Select-String -Path sim/tb_robot_system.sv -Pattern 'menu_index|group_id!='
```

---

*本计划由 resource-auditor 于 2026-10-29 完成，全程只读、未改代码、未跑构建。执行前请以 Lead 故障修复后的新基线重取数字；执行后请用新的 `Embodied_RPACS.rpt.txt` / `_syn_rsc.xml` 核对本文件全部估算。*

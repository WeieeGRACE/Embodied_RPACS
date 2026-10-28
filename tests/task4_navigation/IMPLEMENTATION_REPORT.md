# 任务四导航模块实施与验证报告

## 当前交付状态

事件分段导航 RTL 已接入独立板级工程，并完成 ModelSim 仿真、闭环单段验证、Gowin 综合和布局布线。保存工程重开复验成功，最新探针位流为 `impl/pnr/task4_navigation_board.fs`。本轮没有烧录 FPGA，也没有执行实车路线。

可供实车验证的只有 A→OPEN_WAYPOINT_1 单段探针，速度上限 50 mm/s（0.05 m/s），方向沿起点 A 的 +Y，预期墙面为地图中的 MID_H；这不是 A→B 或巡检的路线放行。完整 A—E/巡检路线继续禁止启动。

沿用提供的场地及底盘基线：3 m × 3 m 场地、600 mm 名义板段、板厚 3 mm，轴中点前方 320 mm、后方 0 mm、两侧各 120 mm，轮距 240 mm。静态场地尺寸不作为本轮要求用户重新测量的项目。A—E 及墙段位置继续按现有地图、路书使用。

## 实现内容

- `src/nav_event_segment_controller.v`、`src/nav_segment_table.v`、`src/nav_tof_wall_event.v`：固定路段表、距离窗口、硬停止里程、前向 ToF 停车阈值、墙事件确认与状态推进。墙事件须匹配当前地图关联墙段、使用新鲜有效样本、满足连续确认和里程窗口；事件不直接报告 `DONE`。
- `src/nav_map_wall_association.v`、`src/nav_pose_range_guard.v`：从当前地图射线关联墙面；融合编码器和 IMU 的短时位姿增量并监控 ToF 年龄。`20 mm` 样本丢弃，不更新有效距离或有效年龄。
- `src/task4_navigation_board_top.v`：在独立工程内接入 IMU SPI、编码器、三路 ToF I²C、速度反馈、PI/PWM、方向保护、急停、OLED 和 UART 段记录。常规参数 `ENABLE_SINGLE_LEG=0`；保存工程使用独立受限 wrapper `task4_navigation_single_leg_probe_top`。
- 速度单位换算采用 `9/128` 的定点比例，避免宽位常数乘法导致 50 MHz 关键路径超时；与当前 10 ms 编码器速度窗口和轮周参数相比，比例误差约 0.1%，正负值向零截断。航向修正乘法分成两级流水，新增延迟相对 100 Hz 导航周期可忽略。
- 位流上电后不自动运动。启动必须通过传感器有效/校准检查；急停异步拉低驱动，释放急停不会自动续行。

## 仿真结果

使用 `.\run_sim.ps1` 完成全部 RTL、板级保护和几何入口。ModelSim 结果见 `records/`：

| 验证项 | 结果 |
| --- | --- |
| 位姿、里程、航向融合、ToF `20` 丢弃和数据年龄 | PASS |
| 校准/几何互锁、近障停车、急停故障锁存 | PASS |
| 墙事件去抖、重复抑制、里程窗口、转弯后新鲜测距 | PASS |
| 地图墙面关联、近障墙拒绝、`20 mm` 拒绝 | PASS |
| 板级无校准启动故障、驱动边界急停、复位安全重入 | PASS |
| 物理轮速/IMU/ToF 单段闭环 | PASS |

单段闭环由左右轮命令推进真实模型位姿，并据此产生编码器、陀螺和 ToF 输入，没有把理想位置或脚本成功信号送给导航器。路线终点模型结果：位置误差 `32.28 mm`，航向误差 `0.692°`，映射墙段 ID `10`，停止距离 `417 mm`。扰动包括左右轮速度差、20 ms 执行延迟、缓加减速及确定性 ToF 噪声。该仿真没有替代板上 PI/PWM 和真实传感器闭环验证。

## 碰撞预测与路线放行

- A→OPEN_WAYPOINT_1 单段：5 个定向场景和固定种子 1,000 个共享墙端扰动场景全部通过；预测最小净空 `18.931 mm`。计算使用 3 mm 板厚、给定车体外廓、起点姿态误差，以及 13.5 mm 反馈反应距离和 20 mm 制动距离等当前动态假设。
- A→B、A→C、A→D、A→E 和 PATROL：名义搜索虽然找到路线候选，但 ±50 mm 墙端保守包络与 1,000 个随机场景均未通过，因此 `board_enabled=false`。这是“目前没有证明符合该误差包络”，不是断言现实场地不可达；不得据此缩小车体或安全余量。
- 检查报告和轨迹预览：`records/collision_prediction_report.json`、`records/geometry_report.json`、`records/routes_preview.svg`、`records/map_v1_preview.svg`。

## 板级构建

保存工程为 `task4_navigation_board.gprj`，器件 `GW5A-LV25UG324C2/I1`。综合、布局布线、位流生成和重新打开保存工程复验均成功。50 MHz 时钟 Fmax `57.707 MHz`，最差 setup slack `2.671 ns`，Setup 违例 0，Hold 违例 0。三路 ToF 的 SCL/SDA 和 OLED SDA 已在布局布线管脚报告中确认为开漏 IO。

最新位流为 `impl/pnr/task4_navigation_board.fs`，SHA-256：

```text
9F0CFC40F0E4214D8B37DE36AE9CA504577C1553F50412E4738BCDDB63491F2D
```

保存工程位流二进制 `impl/pnr/task4_navigation_board.bin` 的 SHA-256 为 `B61F05E52877DE70B73AFB1147FAD361F4119C6AF71E30EA56AD306E0B8E89B1`。全部源文件、约束、综合网表及位流哈希见 `records/board_build_sha256.json`；详细时序见 `impl/pnr/task4_navigation_board_tr_content.html`。

## 上板步骤与限制

详细接线信号、上电顺序、指示灯、启动方式和故障处理见 [`FIELD_TEST_GUIDE.md`](FIELD_TEST_GUIDE.md)。本次允许的验证范围仅为：复位/急停/传感器检查、静态 IMU 与 ToF 检查，以及 A→OPEN_WAYPOINT_1 的单段 0.05 m/s 验证。实测 ToF 年龄/误差和命令到停稳距离后，才能决定是否继续开放任何路段；不要求重新测量用户已提供的场地静态尺寸。

完整路线在得到各段几何净空证明并通过实车低速阶段测试前保持禁用。最终每条路线仍按 10 次至少 9 次无碰撞完成、位置误差不超过 100 mm、航向误差不超过 5°验收；安全停车计作未完成。


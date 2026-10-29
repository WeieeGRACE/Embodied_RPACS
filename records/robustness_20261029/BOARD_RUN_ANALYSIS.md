# 2026-10-29 上板连续避障运行日志分析（用户实测）

- 固件：主工程诊断构建 SHA256 `F9B573A648AFEDA94A59EED067BA84CA3182837355B9E6E47A0C0E237F7E85CD`
  （带 `route_reason`/`tof_reused` 诊断字段与 PAUSED 面板行）
- 串口：CH9102（抓取期间由 COM14 重新枚举为 **COM23**），115200 8N1，只读
- 原始抓取：`board_run_demo.log`（脚本因端口被其他程序抢占未落盘），
  完整尾段 `board_run_demo_full.log`（16:18:16–16:20:40，共 357 条 `D` 行）
- 16:16:56–16:18:16 段仅存于会话输出，关键解码行摘录见下表

## debug_state 解码（system_motion.v）

```
[63:56] start_diag  [55:48] route_reason  [47:40] menu_command  [39:32] ack
[31:24] aux_type    [23:16] aux_code     [15:13] tof_reused    [12:8] route_state
[7] voice [6] ready [5] tof_all_valid [4] fault [3] busy [2] wait [1] auto [0] avoid
```

route_state：1 IDLE / 1 CRUISE_START / 2 CRUISE / 3 BRAKE_SELECT / 4 TURN_OUT /
5 WAIT_OUT / 6 SHIFT / 7 TURN_PARALLEL / 8 WAIT_PARALLEL / 9 PASS / 10 BRAKE_EDGE /
11 EXTRA / 12 TURN_BACK / 13 WAIT_BACK / 14 RETURN / 15 TURN_HOME / 16 WAIT_HOME /
17 BLOCKED / 20 WAIT_HOME 之后的巡航恢复 barrier / 21 BRAKE_SHIFT / 22 SHIFT_EXTRA / 23 SAMPLE_SHIFT

## 三次运行结果

| # | K2 启动 | 结果 | 关键证据 |
|---|---|---|---|
| 1 | 16:17:10.45 | **PAUSED（reason=0x85）** | 16:17:18.4 检测障碍→BRAKE_SELECT；16:17:22.31 TURN_OUT 完成进入 **WAIT_PARALLEL(8)**；此后 `tof_reused` 的 bit2（RIGHT 通道）持续置位（0x88/0xA8/0xC8/0xE8），RIGHT 一直停在精确 20 mm 的复用状态；4 次 1000 ms barrier 重试全部超时 → 16:17:26.311 `D 0085160006019163`（state=17 BLOCKED、**reason=0x85**）；随后 `D 0000160006018061`：auto_run=0 → 面板 PAUSED |
| 2 | 16:17:59.85 | **完整绕障成功** | 16:18:00.845 CRUISE → 16:18:07.73 障碍 → 16:18:11.686 WAIT_PARALLEL（84 ms 通过）→ 23 SAMPLE_SHIFT → 6 SHIFT → 21 BRAKE_SHIFT → 22 SHIFT_EXTRA → 7 TURN_PARALLEL → 13 WAIT_BACK → 9 PASS → 10 BRAKE_EDGE → 11 EXTRA → 12 TURN_BACK → 16 WAIT_HOME → 14 RETURN → 15 TURN_HOME → 20 → 1 CRUISE_START → **16:18:47.20 恢复 CRUISE**，全程约 47 s |
| 3 | 16:19:19.3 | **完整绕障成功** | 16:19:19.51 CRUISE → 19:26.60 障碍 → 19:30.589 WAIT_PARALLEL（95 ms 通过）→ 23 → 6 → 21 → 22 → 7 → 13 → 9 → 10 → 11 → 12 → 16 → 14 → 20（16:20:07.52）→ 1/2（**16:20:07.67 恢复 CRUISE**），约 48 s |

运行 2 期间有 `S M 00 / S N 00 / S I 00` 事件行（动作开始、ASR 状态），无 `E `（故障）行。

## 结论

1. 旧版“转 90° 后 FAULT、必须 K0 复位”的故障已消除：本次 1 次失败也只是
   `BLOCKED`/PAUSED（`auto_run` 被置 0，K2 可恢复），3 次运行 0 次故障锁存。
2. 剩余问题定位明确：**转向后 barrier（WAIT_PARALLEL）等不到该侧通道的“新有效样本”**。
   RIGHT 通道连续返回精确 20 mm（min-range 截断/测距失效），固件按策略丢弃并复用旧值，
   因此 `post_turn` 永远不置位；4×1000 ms 重试后进入 PAUSED。三次运行中 1 次命中
   （本次约 1/3，属间歇性）。
3. 失败与成功时 WAIT_PARALLEL 停留时间差距极大（4.0 s 失败 vs 0.085–0.184 s 成功），
   说明修复点应放在该 barrier 对“20 mm 卡住通道”的处理策略上，而不是提高重试次数。

## 修复方向（本轮实现）

- 传感器层：连续 2 次精确 20 mm 视为“本次测距已完成、目标不可测/过近”，
  可作为“新鲜测距”供 barrier 使用（但绝不把它当作可用的几何距离）；取消“连续 4 次
  就 XSHUT 重初始化”的无效动作（本次日志证明 4 s 内多次重初始化后通道仍持续 20 mm）。
- 路线层：转向后若障碍侧通道持续 20 mm，立即进入固定里程兜底绕行
  （横移 + 额外横移 + 固定直行通过 + 回线），不再停在 WAIT_PARALLEL。
- 仿真：新增“转向后侧向通道持续 20 mm”场景，要求路线仍能恢复到 CRUISE；
  重建后重新上板连续跑 3–5 次确认。

## 用户补充反馈与最终处理（2026-10-29 16:24–16:45）

- 用户反馈：“刚刚 IMU 地线掉了跑得很稳，现在插回去又不行了”。
  这与“地环路影响 VL53L0X 量程下限截断”的判断一致。本工程机械接线把 IMU 地接在
  P11-3 / LCD_GPIO0 / **T11**，而 PIN_ASSIGNMENT.md 早已注明 T11 是 FPGA 信号脚、
  “P11-2 才是专用 GND”。建议把 IMU 地改接 P11-2 或 P6-12/P6-30 专用 GND，
  三路 ToF 与 IMU 星形一点接地，不经 T11；改线后再跑 3–5 次确认。
- 用户随后反馈：“现在改了之后基本上都能跑完了，如果能让他更稳定就好了”。
  据此完成第二轮固件修复（20 mm 截断容错 + 固定里程兜底，详见
  TECHNICAL_REPORT.md §3.6/§4.5 与 `records/fault_injection/REPORT.md` §9）：
  连续 2 次精确 20 mm 视为“测距完成但不可用”，作为 barrier 的新鲜事件放行；障碍侧
  持续 20 mm 时路线直接改走固定里程兜底，不再 4 s 后 PAUSED；删除上板证明无效的
  XSHUT 重初始化。
- 第三轮（最终源码）进一步删除 `obstacle_sensors.valid` 的“距离 > 85 mm”掩码：
  真实近距是可信几何，不再让整条路线因“读到近距”而判为未就绪；路线层把
  `clip` 与 `side_mm <= AVOID_MM` 统一为“侧向不可用”，共用固定里程兜底。
- 新位流：`impl/pnr/Embodied_RPACS.fs`，SHA256
  `B90071E6C58283F0471ABC5F04AC0EA643FDF9CEC84A3B502F6BFBEE98AE2A10`
  （Logic 13420/59%、CLS 8304/73%、Fmax 58.287 MHz、0 违例）。
  仿真：11 个故障注入场景 0 故障 0 PAUSED（场景 10 断言进入兜底，场景 7 断言近距兜底）、
  7 套主工程仿真 PASS。

## 2026-10-29 18:10–18:20 板测进展（P6-21 无线串口方案）

- Gowin 下载器（板载 FT2CH，VID_0403/PID_6014）已识别，`programmer_cli --scan` 找到
  GW5A-25A（ID 0x0001281B）；已成功 SRAM 烧写 P6-21 诊断位流
  `tests/p6_uart_debug/impl/pnr/Embodied_RPACS_p6uart.fs`
  （SHA256 `EC3ACEFF698DAB306DA2F04CD8CB6085D1EB61BD6BE4EBE7E26621531B196EDC`，
  Logic 13420/59%、CLS 8284/72%、Register 5690）。
- WCH-Link（单个 USB 棒，`WCH-Link SERIAL (COM24)`）按方案接线：RX → P6-21、
  GND → P6-12/P6-30；115200 8N1 只读。
- 烧写后 COM24 仍无数据，随后 Gowin 扫描报 `No Gowin devices found`：用户确认
  **机器人电池没电**，板子已断电。待充电后重新扫描/烧写并采集 3–5 次连续避障。
- 备注：板载 USB-UART（CH9102F，V8/U8）只在插 Type-C 数据线时枚举；FTDI 的 COM15
  是 FT2CH 第二通道，未接 FPGA UART，不能用于抓取。

## 2026-10-29 20:33–20:42 板测（PASS 中途截断问题定位）

- 20:33 板子上电、Gowin 下载器与 WCH-Link 同时在线；后台守护自动烧写 P6-21 诊断位流
  （`tests/p6_uart_debug/impl/pnr/Embodied_RPACS_p6uart.fs`，即当时的 18:14 构建），
  随后 COM24 只读采集 360 s，抓到 3367 行、0 条 `E ` 故障行。
- 用户连续尝试 5 次：**2 次完整跑完**（约 47 s / 48 s，20:41:10、20:42:23 恢复 CRUISE），
  **3 次在 PASS 阶段 BLOCKED**（20:37:41、20:39:09、20:39:57），表现为路线暂停
  `reason=0x04`、`route_state=17`，随后 `auto_run=0` 面板 PAUSED。
- 解码 `D` 行时间线（`records/robustness_20261029/board_wchlink_run_20261029_2046.log`）：
  3 次失败都满足 `prev state=9 (PASS)`、`tof_reused=4`（RIGHT 通道 20 mm 截断确认）。
  即 RIGHT 通道在 `WAIT_BACK` 时还正常，进入 PASS 后才转为 20 mm 截断；此时边缘检测
  （`side_sample`）不再产生事件，路线一直直行到前向阈值 `front_block` 才进入 BLOCKED。
- 根因与修复：`PASS`/`SHIFT` 的固定里程兜底原来只在进入该段时已知侧向不可用才启用。
  现改为运行中途 `side_unusable` 也生效——`(fallback_pass || side_unusable) &&
  pass_traveled>=FALLBACK_PASS_UM`（PASS）、`(fallback_shift || side_unusable) &&
  shift_traveled>=FALLBACK_SHIFT_UM`（SHIFT）。新增故障注入场景 11 `PASS_CLIP` 覆盖：
  进入 PASS 后钳 RIGHT=20 mm，修复版 `pass_reached=1`、`pass_clip_done=1`，
  53.2 DUT-s 恢复 CRUISE，0 故障 0 PAUSED。
- 20:33 还发现串口采集脚本的一个使用问题：`read_serial_log.ps1` 的相对 `-OutFile` 会按
  调用者的当前目录解析，脚本改用绝对路径后采集正常（见 `README`/第 6 节示例）。
- 本轮最终固件与 P6-21 诊断位流已重建（含 SPIN 360、IMU ATTITUDE 页与该 PASS 兜底），
  等待用户重新烧写后复测。

## 2026-10-29 22:52–22:59 板测（PASS 前向坏数据 → 时间滤波）

详细逐次时间线与复现命令见 `records/robustness_20261029/board_run_20261029_2252_analysis.md`。

- 采集：功能版 P6-21 诊断位流（`B3A02502…`）+ WCH-Link COM24 只读 15 分钟；原始日志
  `records/robustness_20261029/board_wchlink_live_2248.log`（约 178 KB、2080 条 `D` 快照）。
- 结果：7 次运行，**0 条 `E` 行、0 次故障锁存**；5 次在 PASS 段 `BLOCKED reason=0x04`
  （前向 ToF ≤85 mm），1 次完整跑完（48 s），1 次人为退出。失败发生在 PASS 开始后
  0.05–5.2 s 不等，成功那次 PASS 走了 5.2 s 才检测到箱尾 → 前向读数存在短时低值。
- 根因：旧逻辑对“前向有效距离 ≤85 mm”单次采样立即判障，箱角扫过前视场等单点坏数据被当成
  真障碍；与几何余量无关，所以**不加大**参考工程的 300/300 mm 距离。
- 修复：`obstacle_route.v` 增加 `FRONT_LEG_MS=400` 时间滤波——让位/超车各腿与 WAIT_* 屏障
  的前向判定必须连续 400 ms 低于 85 mm 才生效；巡航段仍 130 mm 立即刹车。新增故障注入
  场景 12（PASS 中 3 段 200 ms 前向 60 mm 毛刺）覆盖，13/13 场景 0 故障 0 PAUSED。
- 重建：主位流 SHA256 `F5DD42E8…`（6,443,628 字节，Logic 15910/70%、CLS 9614/84%、
  Fmax 56.049 MHz），P6-21 诊断位流 SHA256 `08DFDED3…`；等待下一次上板复测。

## 2026-10-30 00:03–00:10 距离日志验证（PASS 前向滤波）

详见 `records/robustness_20261029/board_run_20261030_0003_analysis.md`。要点：

- 距离日志诊断固件（`EE00D375…`）连续两次会话共 **4 轮完整绕障**，PASS 段不再 `BLOCKED`；
- 唯一一次停车在 SHIFT 段：前向连续 77–82 mm（>400 ms）、左侧墙 145 mm，属真实障碍的安全停车；
- 0 条 `E` 行、0 次故障锁存；原始日志 `records/robustness_20261029/board_p6dist_live.log`。


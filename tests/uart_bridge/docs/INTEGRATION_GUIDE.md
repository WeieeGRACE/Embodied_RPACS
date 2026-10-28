# 链路接入指南（把 `uart_bridge` 合进整车工程）

> 面向"链路 + 电机闭环"联调。今晚（2026-10-29）只做到独立测试工程 + 电机侧适配器的
> 仿真验证，**没有改动主工程任何文件**（PLAN 决策 D3/R12）。本文给出接入所需的全部
> 信号映射、安全约束、资源核算与验收清单。

---

## 1. 链路核心对外接口（已冻结）

`uart_bridge_core` 只暴露"目标速度 + 状态"，不含任何 PWM：

| 信号 | 方向 | 位宽 | 含义 |
| --- | --- | --- | --- |
| `target_v_left` / `target_v_right` | out | signed 16 | 目标轮速 mm/s，已按 `SET_LIMIT` 裁剪 |
| `setvel_strobe_out` | out | 1 | 每接受一条 `SET_VEL` 出一拍脉冲；**速度值与脉冲同拍更新，消费方要在下一拍采样**（`host_link_motion_adapter` 已这样做） |
| `stop_latched_out` | out | 1 | STOP **或硬件急停**锁存（必须 `CLEAR_FAULT` 解除） |
| `estop_latched_out` | out | 1 | 硬件急停锁存（`ext_fault[2]`），松开按钮不自动恢复 |
| `link_ok_out` | out | 1 | 心跳新鲜（≤300 ms） |
| `link_lost_out` | out | 1 | 失联锁存（上电即 1，显式使能原则） |
| `fault_out` | out | 8 | 故障位图（见协议 §4.3） |
| `*_cnt` 诊断 | out | 16 | good/crc/len/tail/sync/timeout/unsupported/telem/ack |
| `hb_seq_out` / `cmd_seq_out` / `hb_age_ms_out` | out | — | 遥测统计用 |

**永不靠近**：PWM、TB6612 方向脚、使能脚。链路只写"目标速度"，安全与执行仍由原有 PL 逻辑负责。

---

## 2. 两条接入路线

### 路线 A（推荐，改动最小）：用适配器映射到现有动作命令

主工程运动层 `src/motor/motion_control_and_mode_fsm.v` 是**动作命令式**：
`cmd_mode`（FORWARD=2 / BACKWARD=3 / ROTATE=6 / BRAKE=8）+ `cmd_reverse` + 速度幅值。

新增的 `tests/uart_bridge/src/host_link_motion_adapter.v` 做映射：

| 链路请求 (v_l, v_r) | 适配器输出 |
| --- | --- |
| `v_l == v_r > 0` | mode=2 FORWARD, speed=v, reverse=0 |
| `v_l == v_r < 0` | mode=3 BACKWARD, speed=\|v\|, reverse=1 |
| `v_l == -v_r` | mode=6 ROTATE（原地）, reverse=方向 |
| `v_l == v_r == 0` | mode=8 BRAKE |
| 差速（其余情况） | **近似**为主分量（直线或原地转），并累计 `differential_approx_cnt` |

**已知限制**：现有运动层没有"圆弧/差速"模式，真差速命令无法精确执行。若上层（建图）
只下发"直行 x mm/s"或"原地转 y °/s"，路线 A 足够；若需要边走边转的平滑轨迹，走路线 B。

**已仿真验证**（`sim/tb_motion_adapter.sv`，19 项全过）：
前进/后退/左转/右转的模式、方向、幅值换算（mm/s → µm/s）、差速近似计数、
STOP 只发一次 BRAKE、停车期间拒绝新命令、解除后可恢复、零速即 BRAKE。

### 路线 B（精确差速，需要改运动层）：给 FSM 增加 `VELOCITY` 模式（mode 10）

在 `motion_control_and_mode_fsm.v` 中新增：

1. `localparam VELOCITY=10;`
2. `cmd_mode == VELOCITY` 时：`cmd_end` 忽略，`mag_l/mag_r` 分别取 `cmd_speed_l/r`，
   `neg_l/neg_r` 取各自符号（而不是共用一个 `cmd_reverse`）；
3. 目标生成改为 `target_l = neg_l ? -vel_l : vel_l`（保留现有加减速斜坡与 `runtime_speed_cap`）；
4. 停车/故障路径不变（BRAKE 优先），`cmd_timeout_ms` 沿用；
5. 回归：主工程的 `tools/run_system_sim.ps1`（七套件）+ 现有动作命令必须零差异。

适配器把 `USE_VELOCITY_MODE` 置 1 即可输出 mode 10。该改动**未在今晚实施**（涉及主工程
回归，需专门一轮）。

---

## 3. 信号接线表（路线 A）

```text
uart_bridge_core                    host_link_motion_adapter            motion_control_and_mode_fsm
  target_v_left  ────────────────► req_v_left
  target_v_right ────────────────► req_v_right
  setvel_strobe  ────────────────► setvel_strobe
  stop_latched_out ──────────────► stop_latched
  link_ok_out / link_lost_out ───► link_ok / link_lost
                                   cmd_valid      ───────────────────► cmd_valid
                                   cmd_mode[3:0]  ───────────────────► cmd_mode
                                   cmd_end[1:0]   ───────────────────► cmd_end
                                   cmd_reverse    ───────────────────► cmd_reverse
                                   cmd_speed_l/r  ───────────────────► cmd_speed_l/r
                                   cmd_goal=0, cmd_timeout_ms=3000
                                   cmd_speed_limit/accel/decel ──────► 对应输入
                     cmd_ready ◄───────────────────────────────────── cmd_ready
```

### 与语音通道的仲裁（PLAN §9.4）

主工程现有 ASR 语音通道通过 `system_motion.v` 生成动作命令。接入后需要保证：

```text
优先级：硬件急停/STOP（锁存） > LINK_LOST（锁存） > 上位机 SET_VEL > 语音命令
实现建议：把上位机与语音两条"提案"接到一个 2 选 1 仲裁器（select = 上位机有新鲜命令），
         停止/失效信号旁路仲裁，直接送 motion FSM 的 stop_request / external_fault。
         motion FSM 自身的 cmd_valid/cmd_ready 握手仍是唯一入口（一次只接一条命令）。
```

`uart_bridge_core` 的 `fault_out[0]`（STOP_LATCHED）与 `fault_out[1]`（LINK_LOST）
应直接驱动 `stop_request`，确保任何软件异常都不会续行。

---

## 4. 资源核算（用户明确关注）

三个变体都是**实测**数值（`gw_sh build*.tcl` 后读 `impl/pnr/*.rpt.txt`）：

| 版本 | Logic | Register | CLS | 其他 |
| --- | --- | --- | --- | --- |
| `uart_bridge_top`（完整） | 1077 | 808 | 829 | — |
| `uart_bridge_lite_top`（精简） | 998 | 747 | 747 | 无 ACK、8 位计数器 |
| `uart_bridge_imu_top`（链路+IMU） | 5405 | 2529 | 3640 | BSRAM 4、DSP 10 |

**主工程当前实测**（`impl/pnr/Embodied_RPACS.rpt.txt`，2026-10-28 23:40）：

| 资源 | 占用 | 余量 |
| --- | --- | --- |
| Logic | 19664/23040（86%） | 3376 |
| Register | 7486/23685（32%） | 16199 |
| CLS | 10522/11520（92%） | **998** |

**结论**：CLS 余 998 ⇒ 完整版链路（829 CLS / 1077 Logic）**可以直接并入**，剩 169 CLS；
精简版剩 251 CLS。适配器另估 ~50 CLS / ~75 Logic（未单独综合，保守估计）。

> 早前"CLS 差 12、放不下"的结论基于 `PLAN.md` 的过期基线（CLS 10728），已作废。

若后续还要在主工程里继续加功能（适配器+仲裁+以太网 RTL），建议同时走下面任一条：

1. **以 `tests/task4_navigation` 为基座重新划分**（CLS 仅 53%，PLAN §9.3 同结论）；
2. **再精简链路**（当前已实现两项，剩余可裁剪项按收益排序）：
   - 收发共用 CRC 引擎（省 ~40 CLS，需加仲裁状态）；
   - 遥测负载 23→15 字节（去掉 pitch/roll/enc_r，省 ~80 CLS/FF）；
   - 命令解码只支持 SET_VEL/STOP（省 ~60 CLS）。
   预计可再降到 **约 650 CLS**，仍保留安全锁存与失联保护。

---

## 5. 接入验收清单

| # | 检查 | 方法 | 期望 |
| --- | --- | --- | --- |
| 1 | 仿真回归不退化 | 主工程 `tools/run_system_sim.ps1`（七套件） | 全 PASS |
| 2 | 链路回归不退化 | `tests/uart_bridge/run_sim.ps1`（四套件） | 全 PASS |
| 3 | 停车优先 | 上位机发 SET_VEL 后按板载 STOP / 语音"停止" | 立即停，链路命令无效 |
| 4 | 失联停车 | 拔 USB 串口 | ≤300 ms 内停，恢复需 `CLEAR_FAULT` |
| 5 | 速度上限 | `SET_LIMIT=50` 后下发 300 | 实际不超过 50 mm/s，ACK=REJECT_OVER_LIMIT |
| 6 | 方向正确 | 架空轮子，下发 ±100 mm/s | 左右轮转向与速度符合 |
| 7 | 长期稳定 | 连续 10 min 心跳+遥测 | 误码 0、无假失联 |
| 8 | 急停 100% | 运行中反复按急停 | 每次都在毫秒级停车、需显式复位 |
| 9 | 语音不受影响 | 语音通道逐命令回归 | 行为与接入前一致 |
| 10 | 位流 SHA | 记录接入前后位流哈希 | 可回溯 |

---

## 6. 今晚已验证 / 未验证

| 项目 | 状态 |
| --- | --- |
| 链路核心（帧/命令/安全/遥测） | ✅ 仿真 60 项 + 上板 19 项 + 600 s 长稳 |
| 适配器映射与握手 | ✅ `tb_motion_adapter` 19 项（对 FSM 命令契约建模验证） |
| 真实 motion FSM 联调 | ⛔ 未做（需改主工程/换基座，留待联调轮） |
| 电机闭环/方向/限速 | ⛔ 未做（今晚明确不控制车运动） |
| IMU 姿态入遥测 | ✅ 独立变体已上板（pitch −3.6°/roll −1.15° 真实值） |
| 以太网 v2 | 协议层就绪（`UdpTransport`），RTL 未做 |

---

## 7. 参考

- 协议冻结：`docs/PROTOCOL_v1.md`
- 验证报告：`records/2026-10-29_uart_bridge_validation.md`
- 适配器：`src/host_link_motion_adapter.v`、仿真 `sim/tb_motion_adapter.sv`
- 主工程运动层：`src/motor/motion_control_and_mode_fsm.v`（命令契约见其端口注释）
- 语音通道参考：`src/asr/system_motion.v`（issue_payload 打包与 FSM 例化）

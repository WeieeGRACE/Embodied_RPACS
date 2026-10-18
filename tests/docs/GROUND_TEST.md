# 低速落地测试（2026-10-14）

## 状态与前置条件

按用户现场确认，左右单轮及双轮正反向架空测试已完成。本阶段进入低速落地验收；这不代表新增固件或真实看门狗已完成实机测试。
历史急停期间左右编码器各出现两次非法跳变的报告保留；若新测试复现，应停止当前项并定位，不修改相序、不屏蔽保护。

沿用原接线、50 MHz、10 ms 测速窗口、4 窗滤波、KP=32/KI=1/KD=0、20 kHz PWM、625/2500（25%）限幅和每 PWM 周期最多一拍斜坡。KEY1 有效按下一次后运行 8 秒，松开或再次按下不影响当前动作；KEY2 急停锁存，KEY0 复位。故障复位或动作结束后必须先释放 KEY1 至少 10 ms，再按才能触发下一次动作。

## 工程与下载

| 动作 | 工程（同名顶层加 `_top`） | TL | TR | SRAM 下载文件 |
| --- | --- | ---: | ---: | --- |
| 前进 | ../gowin/robot_dual_forward_test.gprj | 10 | 10 | ../../impl/pnr/robot_dual_forward_test.fs |
| 后退 | ../gowin/robot_dual_reverse_test.gprj | -10 | -10 | ../../impl/pnr/robot_dual_reverse_test.fs |
| 左原地旋转 | ../gowin/robot_rotate_left_test.gprj | -10 | 10 | ../../impl/pnr/robot_rotate_left_test.fs |
| 右原地旋转 | ../gowin/robot_rotate_right_test.gprj | 10 | -10 | ../../impl/pnr/robot_rotate_right_test.fs |
| 左转向 | ../gowin/robot_turn_left_test.gprj | 5 | 10 | ../../impl/pnr/robot_turn_left_test.fs |
| 右转向 | ../gowin/robot_turn_right_test.gprj | 10 | 5 | ../../impl/pnr/robot_turn_right_test.fs |
| 看门狗（前进） | ../gowin/robot_watchdog_test.gprj | 10 | 10 | ../../impl/pnr/robot_watchdog_test.fs |

新增构建命令：在工程目录用 Gowin `gw_sh ../gowin/build_rotate_left_test.tcl`，其余使用 `../gowin/build_rotate_right_test.tcl`、`../gowin/build_turn_left_test.tcl`、`../gowin/build_turn_right_test.tcl`、`../gowin/build_watchdog_test.tcl`。
先检查对应时序报告，再断开电机动力线下载，选择 SRAM Program。保持原单轮/双轮文件，不将专用文件覆盖为旧文件名。
新动作先无动力检查按键、方向及关断，再短时架空检查，不重做已完成的基础架空测试。

## 看门狗接线与主机

板载 UART TX=V8、RX=U8；115200、8N1、无流控。外接 USB-TTL 时 RX 接 V8、TX 接 U8、GND 共地，电平 3.3 V，不接供电脚。普通动作工程无需 UART RX。

主机每 100 ms 发一个二进制 `A5` 字节（不是字符字符串 `A5`）。只接受有效起始位和高停止位的 8N1 帧。第一次有效心跳前禁止启动；第一次心跳后即开始计时，运行或空闲时超过 500 ms 未刷新都锁存超时，直到 KEY0 复位。
错误字节和停止位错误不刷新。持续正常心跳期间按下一次 KEY1 即可启动；超时恢复发送不会恢复驱动。

关闭串口助手，在交互 PowerShell 中运行：

```powershell
./tools/send_heartbeat.ps1 -Port COM3
```

换成实际端口。脚本同时接收并保存完整遥测；空格暂停/恢复心跳，Q 退出。暂停心跳后无需保持 KEY1，8 秒动作期间看门狗仍会独立关断；主机发送日志只能提供软件时间参考，精确关断延时用示波器测 UART 最后有效帧与 PWM/STBY。

原 20 个遥测字段顺序和语义不变；仅看门狗工程追加 `HB`（自复位以来收到过有效心跳）和 `WD`（超时锁存）。`HB=1` 不表示当前心跳仍正常。看门狗故障为 `FC=5`；急停 `FC=1` 优先。8 秒动作计时独立存在，不替代看门狗。

## 落地步骤

1. 使用平坦、空旷测试区域，保留实体断电开关和限流供电；记录电源设置与固件文件哈希。检查轮子无卡滞。
2. 前进、后退先各短按一次检查方向，松开 KEY1 后确认仍运行；停稳后再运行至 8 秒自动停止，各重复三次；记录偏移、速度、饱和、电流、温升及停止滑行。
3. 在前后退运行中分别测试 KEY2 急停和松开 KEY1。松开 KEY1 后动作仍应继续，约 8 秒自动停止；急停释放后仍应关断，重新按 KEY1 不得运行；KEY0 复位后先松开再按才可运行。
4. 看门狗工程先验证无心跳不能启动、正常心跳能运行。在启动后约 0.5 秒暂停心跳，预期最后有效心跳后约 500 ms 关断，早于本次 8 秒限时；恢复心跳不得自行运行。复位后重复三次。
5. 左右原地旋转、左右转向同样按下一次后运行至 8 秒自动停止，各三次。转向为同向不同速；不得通过交换动力线实现换向。每次切换动作先停稳，下载前断开动力。

若方向/反馈符号不一致、持续饱和、明显抖动、供电下降、异常发热或编码器故障，立即松开 KEY1，必要时实体断电；不通过提高限幅强行启动。当前未接硬件过流检测，PWM 限幅不等于过流保护。

## 验收与记录

正常段反馈跟随目标，无持续饱和、编码器故障或异常复位。PWM 不超过 625。左右 PWM 可不同，不要求最终累计位置完全相等。记录按键释放后仍运行的状态及 8 秒自动停止时间。
禁用/急停/超时后 EN/ST/PL/PR 归零，两路物理 PWM 与方向脚均关闭；机械可以惯性滑行。分别记录电气关断时间、8 秒自动停止时间、停稳时间和滑行距离，串口 10 Hz 报告不能证明亚毫秒响应。
急停、超时保持锁存，恢复输入不能自动运行。测试结果填写 `../records/落地测试记录模板.md`，每次保存原始完整日志和固件 SHA256。此文档不预填实机通过结论。

## 自动验证

在 sim 运行 `vsim -c -l ground_tests.log -do run_ground_tests.do`。包含四种不同左右目标的电机模型、实际 PWM/方向/限幅、禁用、编码器故障、急停、限时重启；看门狗覆盖无心跳、错误字节、错误停止位、连续刷新、精确计数边界、两路关断、FC=5、恢复不重启和复位再使能；扩展 UART 逐位解码检查完整字段、边界及原子快照。
旧双轮回归 `run_dual_closed_loop_test.do`、左轮/UART/PI/滤波回归 `run_closed_loop_test.do` 和右轮回归 `run_right_closed_loop_test.do` 用于检查默认行为兼容。仿真和时序报告不能替代落地实测。

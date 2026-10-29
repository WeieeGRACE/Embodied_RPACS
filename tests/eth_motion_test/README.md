# 链路 v2（以太网）+ 电机控制测试位流

> 用途：**明天动车测试**用。今晚只做了仿真与综合，**没有烧写、没有通电驱动电机**。
>
> 位流：`impl/pnr/eth_motion_top.fs`（`eth_link_top` + 速度→PWM→TB6612 + 安全门控）
> SHA256 `AD7C8ADB6B30979D7E5E551B610B5B98C78521D04EF6B532A9AFBA4F31F8D25F`
> 资源：Logic 1928/23040（8%）、Register 1116、CLS 1401/11520（12%）

---

## 1. 这块位流做了什么

```
Pi ──UDP(SET_VEL/STOP/…)──> FPGA 链路核 ──target_v_left/right(mm/s)──┐
                                                                    ▼
                                                        eth_motion_bridge
                                              （限幅 + 速度→占空比 + 使能门控）
                                                                    │
                                                                    ▼
                                         主工程 tb6612_driver_interface
                                     （方向死区 + 安全 inhibit，未改动）
                                                                    │
                                        PWMA/AIN1/AIN2/PWMB/BIN1/BIN2/STBY
```

- `SET_VEL(l, r)` 的 mm/s 线性映射成 TB6612 占空比：`duty = v × 20`（300 mm/s = 满占空比，20 kHz PWM）。
- 遥测仍按 50 Hz 回传（fault/链路状态/目标速度）。
- **安全全部硬连线**，不依赖上位机：

| 触发 | 结果 |
| --- | --- |
| 心跳丢失（300 ms） | `watchdog_timeout` → **STBY=0** |
| 任一故障位（含 CMD_TIMEOUT） | `fault` → **STBY=0** |
| STOP 锁存 | `enable=0` → **STBY=0**，需 CLEAR_FAULT 解锁 |
| 急停键 KEY2（C15，低有效）按下 | `emergency_stop` → **STBY=0** |
| 复位未释放 | inhibit → **STBY=0** |

## 2. 今晚已完成的验证（无硬件动作）

| 项 | 结果 |
| --- | --- |
| 仿真 `sim/tb_eth_motion_bridge.sv` | **15/15 PASS**：空载占空比 0、150 mm/s≈48% 占空比、方向正确、后退反向、超速限幅、链路丢失/故障/STOP/急停四路都能拉低 STBY、恢复后能重新使能 |
| 综合 | 通过（见上） |
| 烧写 | **未烧写**（保持板上是无电机的 `eth_link_top`） |

仿真命令：

```powershell
cd tests\eth_motion_test
D:\app\modesilm\win64\vsim.exe -c -do "do sim/run_motion.do"
```

## 3. 明天测试步骤（建议按顺序）

### 3.0 准备

```powershell
# PC：烧电机位流（SRAM，掉电失效）
cd tests\eth_motion_test
programmer_cli --device GW5A-25A --run 2 `
    --fsFile (Resolve-Path impl\pnr\eth_motion_top.fs) --cable "Gowin USB Cable(FT2CH)"
```

```bash
# Pi
sudo bash ~/pi_setup_eth.sh          # IP + 静态 ARP
python3 ~/eth_fault_test.py          # 链路安全语义先过一遍（无电机动作）
```

**第一轮必须架起车、轮子离地**（或电机电源先断开，用万用表/示波器量 PWMA/AIN1/AIN2/STBY）。

### 3.1 轮子离地

| # | 操作 | 期望 |
| --- | --- | --- |
| 1 | 心跳+CLEAR_FAULT 后发 `SET_VEL(150,150)` 持续 1 s | 两轮同向转；示波器 PWMA 占空比 ≈48% |
| 2 | `SET_VEL(-150,-150)` | 反向转（AIN1/AIN2 互换） |
| 3 | `SET_VEL(-120,120)` | 原地转，两轮反向 |
| 4 | `SET_VEL` 从 0 逐级加到 300 | 占空比线性上升，无突变/异响 |
| 5 | 停发命令（主机 pause） | 500 ms 后 CMD_TIMEOUT → STBY 掉、轮停 |
| 6 | 拔网线 | 300 ms 内 STBY 掉、轮停 |
| 7 | 按 KEY2 急停 | 立即 STBY=0 |
| 8 | `STOP` 命令 | 停 + 锁存；再发 SET_VEL 被 `REJECT_STOP`；CLEAR 后恢复 |

### 3.2 落地（低速）

| # | 操作 | 期望 |
| --- | --- | --- |
| 9 | 限速 100 mm/s 直线 2 m | 直行，不跑偏（机械差异允许微调） |
| 10 | 限速 100 mm/s 原地转 90° | 两轮反向，转向正确 |
| 11 | 中途拔网线 | 立即停车（安全链路生效） |

## 4. 已知限制（明天别误判为故障）

1. **没有编码器反馈/PID**：本测试位流只验证「链路 → PWM → H 桥」通路；速度闭环、里程、姿态保持都不在本版范围（遥测里编码器字段恒 0）。
2. **没有 ToF 避障**：障碍物安全由人工/上位机负责。
3. **CMD_TIMEOUT=500 ms**：上位机必须以 ≥10 Hz 持续发命令（建议 20 Hz），否则会主动停机。
4. **心跳 300 ms**：Pi 上跑相机时实测曾出现 330 ms 延迟导致 LINK_LOST；若明天频繁误停，把 `eth_link_bridge` 的 `HB_TIMEOUT_MS` 调成 500 重编即可。
5. **左右轮方向/极性**：若某轮方向反了，优先调换该电机的两根线，其次在 RTL 里取反。
6. 板载按键：KEY2=C15 急停、KEY0=B16 复位；都是**点动**键，松手即恢复（急停是"按住才停"）。

## 5. 文件

```text
tests/eth_motion_test/
├── README.md（本文件）
├── build.tcl                       综合脚本
├── src/eth_motion_bridge.v         速度→占空比 + 使能门控（新增）
├── src/eth_motion_top.v            顶层（链路桥接 + 电机驱动）
├── src/eth_motion_top.cst          RGMII + 电机 + 急停/复位/LED 引脚
├── sim/tb_eth_motion_bridge.sv     15 项自检
├── sim/run_motion.do
└── impl/pnr/eth_motion_top.fs      待烧位流
```

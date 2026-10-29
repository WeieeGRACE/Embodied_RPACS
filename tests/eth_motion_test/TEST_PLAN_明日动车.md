# 明日动车测试清单（2026-10-30）

> 位流：`tests/eth_motion_test/impl/pnr/eth_motion_top.fs`
> SHA256 `AAC99B3BB3288D3A145AD69E698289DDB75AB8B5C80E9748C4D01713C59CCBE3`
> 今晚只做了仿真（15/15）与综合，**没有烧写、没有通电驱动电机**。

---

## A. 开测前（10 分钟）

| # | 项目 | 做法 | 结果记录 |
| --- | --- | --- | --- |
| A1 | 车架起 / 电机电源先断 | 轮子悬空；或把电机电源线拔掉 | ☐ |
| A2 | 烧位流 | `programmer_cli --device GW5A-25A --run 2 --fsFile <abs>/eth_motion_top.fs --cable "Gowin USB Cable(FT2CH)"` | ☐ |
| A3 | Pi 网络 | `sudo bash ~/pi_setup_eth.sh`（IP+静态ARP），`cat /sys/class/net/eth0/speed` = 1000 | ☐ |
| A4 | 链路自检 | `python3 ~/eth_link_test.py` → 12/12 PASS | ☐ |
| A5 | 静态引脚检查（电机电源仍断） | 万用表/示波器量 STBY（G13）：无命令时应为 **0**；发心跳+CLEAR 后应为 **1** | ☐ |

> A5 很重要：STBY 从 0→1 说明"安全门控 + 驱动芯片"这条链是通的，再给电机上电。

## B. 轮子离地（电机上电，30 分钟）

| # | 操作 | 期望 | 结果/备注 |
| --- | --- | --- | --- |
| B1 | 心跳 10 Hz + `CLEAR_FAULT`，然后 `SET_VEL(150,150)` 1 s | 两轮**同向**转；PWMA 占空比 ≈48% | ☐ |
| B2 | `SET_VEL(-150,-150)` | 反向转（AIN1/AIN2 互换） | ☐ |
| B3 | `SET_VEL(-120,120)` | 原地转，两轮反向 | ☐ |
| B4 | 0→300 mm/s 分 5 级 | 占空比线性、无异响/抖动 | ☐ |
| B5 | 停止发命令 0.8 s | CMD_TIMEOUT → STBY 掉、轮停 | ☐ |
| B6 | 拔网线 2 s 再插回 | 300 ms 内 STBY 掉、轮停；插回后需 `CLEAR_FAULT` 才能再动 | ☐ |
| B7 | 按住 KEY2（C15） | 立即停（STBY=0） | ☐ |
| B8 | `STOP` 命令 | 停 + 锁存；再发 `SET_VEL` → `REJECT_STOP`；`CLEAR_FAULT` 后恢复 | ☐ |
| B9 | 方向确认无误 | 前进 = 车头方向（如需改：调换该电机两根线） | ☐ |

## C. 落地低速（电机已确认方向，30 分钟）

| # | 操作 | 期望 | 结果/备注 |
| --- | --- | --- | --- |
| C1 | 100 mm/s 直线 2 m | 基本直行（允许小幅跑偏） | ☐ |
| C2 | 100 mm/s 原地转 90° | 转向正确、无卡顿 | ☐ |
| C3 | 行驶中拔网线 | **立即停车**（安全链路生效） | ☐ |
| C4 | 行驶中按 KEY2 | 立即停车 | ☐ |
| C5 | 200 mm/s 直线 2 m | 稳定，无丢步 | ☐ |

**测完请把上表 ☐ 改成 ✓/✗ 并记录异常现象**，把 `SET_VEL` 值、ACK、遥测 fault、STBY 电平填进备注。

## D. 需要记录的数据

- 每个用例的：下发命令、ACK 状态、遥测 `fault`、STBY 电平、实际轮速（视频/转速计）
- Pi 侧可全程留日志：
  ```bash
  python3 ~/eth_soak.py --duration 600 --csv ~/car_test.csv --json ~/car_test.json
  ```
- FPGA 侧如需调试文本（无线串口）：烧 `tests/eth_link_protocol/impl/pnr/eth_link_dbg_top.fs`
  的 L12 调试变体不适合开车（没有电机），开车时用电机位流即可。

## E. 出问题先查这些

| 现象 | 优先排查 |
| --- | --- |
| STBY 一直是 0 | KEY2 是否被按住（低有效）；`fault` 是否非 0（先 CLEAR_FAULT）；心跳是否在发 |
| 命令全部被拒 | ACK 状态：`REJECT_LINK`=心跳断/未清锁存；`REJECT_STOP`=STOP 锁存未清 |
| 一个轮子不转/反向 | 该电机接线；AIN/BIN 极性；驱动板供电 |
| 速度抖/异响 | 电机电源电压不足；PWM 20 kHz 干扰；机械卡滞 |
| 链路频繁断 | 网线/水晶头；心跳 300 ms 偏紧 → 改 20 Hz 心跳或重编 `HB_TIMEOUT_MS=500` |

---

## F. 驾驶助手（已部署到 Pi：`~/eth_drive.py`）

明天不用手写 SET_VEL 循环，直接用这个（20 Hz 命令 + 斜坡 + 自动安全停车）：

```bash
# 轮子离地：前进 150 mm/s 3 秒（0.5 s 斜坡）
python3 ~/eth_drive.py --left 150 --right 150 --seconds 3

# 后退
python3 ~/eth_drive.py --left -150 --right -150 --seconds 3

# 原地转（左退右进）
python3 ~/eth_drive.py --left -120 --right 120 --seconds 2

# 落地低速直线 60 s
python3 ~/eth_drive.py --left 100 --right 100 --seconds 60

# 只停车 / 只清故障
python3 ~/eth_drive.py --stop
python3 ~/eth_drive.py --clear
```

内置安全：
- 任何 `SET_VEL` 被拒 / 无 ACK / 故障位非 0（CMD_TIMEOUT 除外）→ **立即 STOP 并退出**（退出码 2）；
- 松开 Ctrl-C 会自动 STOP；
- 速度超过 300 mm/s 直接拒绝执行；
- 结束自动 STOP + CLEAR_FAULT（不会把车留在停车锁存状态）。

今晚已在无电机位流上验证过完整流程：建链 → 斜坡 11 步 → 巡航 2 s → 减速 → STOP(OK) → 完成。
# 避障稳定性修复轮：改动前基线（2026-10-29）

本文件在修改任何 RTL 之前提交，作为"改动之前的本地提交"基线记录。
自本文件起，本轮（90° 转向后故障修复 + 资源优化 + 技术报告）的所有改动都在其后提交。

## 代码基线

- 仓库根：`D:/Gowin/Project/Contest/Embodied_RPACS/Embodied_RPACS/Embodied_RPACS`
- HEAD：`3556192 chore: remove temporary debug do file`（与 `origin/main` 同步，工作区干净）
- 上一轮交付：`f22c6e0 fix: green all seven suites and verified final bitstream on board`

## 位流与构建基线

- 构建命令：`gw_sh build.tcl`（Gowin 1.9.12，GW5A-25A / GW5A-LV25UG324C2/I1，50 MHz）
- 构建日志：`records/baseline_20261029/build_baseline.log`
- 位流：`impl/pnr/Embodied_RPACS.fs`
  - 大小：6,389,380 字节
  - SHA256：`90DA6F4FE7A77C2DAFEAF08AE951E7C8B9E6A043934A0A7343C998B8C8A9B5E8`

### 资源（post-PnR，2026-10-29 11:40）

| 资源 | 占用 | 比例 |
| --- | --- | --- |
| Logic | 17661/23040（14866 LUT + 2795 ALU） | 77% |
| Register | 6859/23685 | 29% |
| CLS | 9858/11520 | 86% |
| BSRAM | 9/56 | 17% |
| DSP | 8.5/28 | 31% |
| I/O Port | 44/222 | 20% |

注意：上一轮 README/提交信息中的 "Logic 12036/53%" 与本次实测 17661/77% 不一致，
本轮会重新以构建产物为准核准并在技术报告中说明。

## 已验证状态

- 上一轮提交声明 7 个主工程仿真套件全部 PASS（system / system_oled / system_50mhz /
  system_dispatch / live_speed / system_obstacle / attitude_pipeline）。
- 上板仅验证过静置心跳（`records/obstacle_20261025/board_smoke_final.log`，45 s 无错误事件），
  尚未完成连续避障上板演示。

## 本轮用户报告

- 避障演示运行中途常出现故障提示（OLED 避障面板显示 `FAULT RESET REQUIRED`）；
- 多发于 90° 转向完成后：遇障左转完成、或左转 + 前进固定距离 + 右转完成时；
- 用户要求：稳定并行采集传感器不报错、简化复杂逻辑、优化板级资源、写技术报告并推送远端。

## 本轮工作顺序

1. 故障注入仿真：证明当前哪些锁存路径会在转向后触发（`records/fault_injection/`）。
2. RTL 修复：瞬时故障可恢复、真实故障仍安全停车；90° 转向后路线可重试继续。
3. 资源优化：按 PROJECT_PLAN 以"最小资源满足避障演示"为原则裁剪不必要功能。
4. 回归：7 个仿真套件 + 全量构建 + 上板验证。
5. 技术报告 + 本地提交 + 推送远端。

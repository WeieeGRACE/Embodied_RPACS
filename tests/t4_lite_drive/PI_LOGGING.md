# Pi 本地数据采集（断网/丢串口都不丢数据）

> 目的：场地测试时 Pi 的 WiFi（到 PC）可能断、无线串口也可能掉；采集必须在 **Pi 本地**完成，
> 联网后再把数据拉回 PC 并清理 Pi 空间。

## 1. 组件

| 文件 | 作用 |
| --- | --- |
| `tools/t4_pi_logger.py` | 前台采集进程：UDP 遥测（默认 5001）+ 可选无线串口 + 可选深度相机距离；自动滚动、自动清理旧会话 |
| `tools/t4_logs.py` | 会话管理：`start / stop / status / list / size / purge` |
| `tools/pull_logs.ps1` | PC 端：把 Pi 的会话拉回来，**校验文件数/字节一致后**再删除 Pi 上的会话 |
| `tools/synthetic_telem.py` | 自检用：向 127.0.0.1:5001 发合成遥测，验证采集链路 |

## 2. 数据布局

```
~/t4_logs/
  logger.out                 # 采集进程输出
  session_YYYYmmdd_HHMMSS/
    telem.jsonl              # 每帧遥测：t(epoch_ms) enc_l enc_r yaw v_l v_r fault ls cs hb
    telem.1.jsonl …          # 超过 --max-mb 后滚动
    events.log               # 链路中断/恢复、fault 变化、串口行、相机距离、操作标记
    meta.json                # 本次会话参数
```

## 3. 用法

```bash
# Pi：开始采集（接无线串口时加 --serial /dev/ttyACM0；要相机距离加 --camera）
python3 ~/t4_tools/t4_logs.py start --serial /dev/ttyACM0 --camera --max-mb 200 --keep 6
python3 ~/t4_tools/t4_logs.py status          # 是否在跑 + 最近会话
python3 ~/t4_tools/t4_logs.py list            # 所有会话及大小
python3 ~/t4_tools/t4_logs.py size <session>  # 文件数/字节（PC 拉取校验用）
python3 ~/t4_tools/t4_logs.py stop
python3 ~/t4_tools/t4_logs.py purge --keep 2  # 只留最近 2 个会话
```

```powershell
# PC：拉回并清理（先 DryRun 看有什么）
powershell -ExecutionPolicy Bypass -File pull_logs.ps1 -DryRun
powershell -ExecutionPolicy Bypass -File pull_logs.ps1            # 拉取→校验→清理 Pi
powershell -ExecutionPolicy Bypass -File pull_logs.ps1 -KeepLocal # 只拉不清
```

## 4. 容量与滚动

- 默认单会话上限 **200 MB**，保留最近 **6** 个会话（约 1.2 GB 上限）。
- 遥测 50 Hz、每行约 110 B → **约 19 MB/小时**；200 MB 可存 ~10 小时。
- 若加 `--camera`，只记录 1 Hz 的中值距离（几 KB/分钟），不存图。
- 空间不足时：`t4_logs.py purge --keep 2` 或 PC 侧 `pull_logs.ps1` 拉走即清理。

## 5. 断线行为

| 情况 | 行为 |
| --- | --- |
| Pi 与 PC 的 WiFi 断 | 采集不受影响（数据在本地），恢复后 `pull_logs.ps1` 拉取 |
| Pi↔FPGA 网线断/FPGA 重启 | `events.log` 记 `LINK_GAP start/end`；恢复后继续采集 |
| 无线串口掉 | 串口线程自动重连，`events.log` 记 `serial error/open` |
| 采集进程崩溃 | `logger.out` 有 traceback；会话文件保留可拉取 |

## 6. 自检记录（2026-10-31 凌晨）

- `t4_logs.py start --duration 8` + 合成遥测 199 帧 → `telem.jsonl` **199 行**，`events.log` 正确记录 LINK_GAP/FAULT。
- `pull_logs.ps1`：远端 3 文件/21202 B ↔ 本地 3 文件/21202 B **一致**，随后远端清理成功。

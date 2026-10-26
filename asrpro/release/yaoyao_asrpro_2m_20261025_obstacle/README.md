# 幺幺 ASRPRO 避障语音版（2026-10-25）

当前入口 `asrpro.hd`，源文件 `voice_runtime.cpp` 和 `tools/build_project.py`。
天问Block实际加载、保存和重开验证：101个积木，23条识别词，15条反馈音频。
完整配套固件：`release/yaoyao_asrpro_2m_20261025_obstacle/fw.bin`。

## 口令

保留唤醒词“幺幺耍起”，前后移动、左右转、加速/快一点、减速/慢一点、停车/停止/停止任务及查询状态。
新增“避障模式”（识别ID24，命令0x16）和“退出避障模式”（ID25，命令0x17）。
退出与三条停车口令均设为唤醒词类别，可在休眠时识别；内建自动回复为空。
上电、唤醒、休眠原播报保持。每次识别续期8589934ms；避障模式每秒收到模式状态也续期。
FPGA的语音开关仍须先在设置中打开；停车、查询、退出不受该开关限制。

## 双向协议

115200/8N1，Serial1 PA2/TX → FPGA D17，PA3/RX ← FPGA D18。
全部是原始六字节 `AA 55 TYPE CODE CHECK 0D`，`CHECK=(TYPE+CODE)&255`。

| TYPE | CODE与语义 |
|---|---|
| 01 | 02停车、03查询、10前进、11后退、12左转、13右转、14加速、15减速、16避障、17退出避障（十六进制） |
| 02 | 00接受、01忙碌、02无法执行 |
| 03 | 原状态00～06、10移动、11完成；20避障暂停、21等待播报、22巡航、23绕障 |
| 04 | FPGA请求“开启避障模式”，CODE为8位请求号 |
| 05 | ASRPRO正常播完后的确认，CODE原样回传请求号 |
| 06 | 01模式存续/唤醒续期，00已退出 |
| 07 | 01开始绕障、02完整绕障完成、03退出完成 |

避障口令不本地播报“收到”，FPGA接受该口令的通用“已接受”也抑制，改由TYPE04触发专用播报。
不能用固定delay或SDK普通结束回调推断成功：普通回调在中断、失败、清队列时也调用。
本版本使用项目隔离SDK内的 `yy_prompt_play`，带真实结束原因及32位本地cookie；只有正常完成（原因0）发送TYPE05。
旧回调、重复完成、播放失败、停止、退出、接管或10秒超时均不能触发启动确认。
FPGA同时验证自己的请求号、等待状态、测距新鲜度与底盘状态。请求号跨软复位递增，电源重新配置后重新初始化。

TYPE03状态播报去重；避障状态20～23静默显示，TYPE07事件逐次播报，不被查询或相同事件历史抑制。
9300开启避障模式、9301遇到障碍开始绕障、9302已绕过障碍物、9303已退出避障模式。
UART接收与音频为独立线程，音频24项队列，完成8项队列，串口发送互斥保护。
AEC开启，`PAUSE_VOICE_IN_WITH_PLAYING=0`，播报期间继续识别。

## 可复现构建

本机天问目录默认为 `D:/app/twenblock`，可设置 `TIANWEN_HOME`。
词表变更需登录天问重新生成模型与音频；不得混用旧包。

```powershell
python -X utf8 tools/build_project.py
node tools/cdp.mjs tools/verify_current.js records/current_verification.json
# 从验证JSON保存code/xml/report为records/generated.cpp、asrpro.hd和records/blockly_validation.json。
# 使用天问生成当前模型并保存到records/current_model.zip。
python -X utf8 tools/prepare_firmware.py
python -X utf8 tools/build_firmware.py
python -X utf8 tools/check_project.py
python -X utf8 tools/package_firmware.py
python -X utf8 tools/verify_release.py
```

隔离构建目录 `records/build/firmware_obstacle_20261025`，SDK补丁由 `tools/patch_sdk.py` 自动应用，不修改安装目录。
`.hd`可编辑与生成代码；链接本版本的增强播放接口须使用上述隔离SDK构建流程，不能直接用未补丁的SDK代替。
首次生成代码后可先用 `prepare_firmware.py --code-only` 准备SDK并验证，再导入模型。
重复导入已有resources会拒绝；保留旧交付，重建时先检查本次工作目录。

发布检查包含：词表ID/类别、空回复25、全部实际音频、分区校验和与CRC16、代码和模型内容一致、AEC配置及固件大小。
官方v2打包器的分区对齐问题沿用现有修正并逐分区复核。烧录只使用发布目录 `fw.bin`。
本次生成资源来自天问当前模型任务；内置下载代理报错后下载同一任务资源URL，并验证23条词表与15条反馈音频。

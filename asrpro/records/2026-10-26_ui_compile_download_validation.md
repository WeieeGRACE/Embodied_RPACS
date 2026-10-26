# ASRPRO 天问编译下载链路验证（2026-10-26）

## 故障与修复

原界面在链接阶段报 `undefined reference to yy_prompt_play`：工程使用带结束原因和cookie的扩展播放接口，而天问安装目录中的预编译 `core.a` 只有普通播放器接口。固件没有生成，因此当时还未开始串口烧写。

在隔离SDK中重建兼容播放器库后，将播放器源文件、头文件和 `core.a` 安装到天问 ASRPRO SDK。原文件已备份在 `D:/app/twenblock/asrpro/.codex_asrpro_yy_prompt_play_backup_20261026_125737_146841`。安装脚本会先比对关键SDK源，版本不匹配时停止。

## 验证结果

- 隔离SDK完整构建通过，当前生成代码用同一工具链链接成功。
- 天问界面再次运行“2M编译下载”，生成了当前 `asr.cpp`、`sample_asr.elf`、`user_code/[0]code.bin` 和 `fw.bin`；没有再次出现 `yy_prompt_play` 链接错误。
- 直接使用安装目录的 `core.a` 重新交叉链接通过；190项主机回归测试通过，涵盖正常播放完成确认及失败、中断、过期确认抑制。
- 生成的 `fw.bin` 为1,605,632字节。烧写器已打开；界面提示连接ASR-PRO，当前烧写按钮禁用，因此本次没有写入设备。

## 复现

从 ASRPRO 工程目录运行：

```powershell
python -X utf8 tools/build_sdk_compat.py
python -X utf8 tools/install_sdk_compat.py --check
python -X utf8 tools/install_sdk_compat.py --apply
```

安装后 `install_sdk_compat.py --check` 可再次确认文件与SDK版本。恢复时，将备份目录中的 `prompt_player.c`、`prompt_player.h` 和 `core.a` 原样复制回对应位置。

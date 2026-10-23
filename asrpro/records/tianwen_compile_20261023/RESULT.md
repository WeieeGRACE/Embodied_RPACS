# 天问当前工程编译验证

日期：2026-10-23（Asia/Shanghai）。本次不修改工程源码，不下载硬件。

- 已启动天问 Block 2025 夏季正式版，确认设备为 ASRPRO，并通过软件的最近工程入口载入当前 `asrpro.hd`。
- 当前工程实际加载 92 个积木，实际 Blockly 生成代码保存在 `generated.cpp`。这些是本次重新生成的结果，未使用 10 月 20 日的旧代码替代。
- 使用安装版 RISC-V GCC 和 ASRPRO SDK 头文件编译当前生成代码，退出码为 1。首个有效错误位于 `generated.cpp:205`：`Serial1.begin();`，错误为 `no matching function for call to 'HardwareSerial::begin()'`。SDK 声明为 `void begin(uint32_t)`，需要一个波特率参数。原始输出见 `cross_compile.log`。
- 源工程 `asrpro.hd:369` 的串口初始化积木为 `yaoyao_40`，其中 `CONTENT` 下的波特率积木 `yaoyao_41` 在第 375 行配置了 115200，但本次天问生成代码未包含该值。应检查该积木在当前版本中的连接/加载兼容性；尚未修改或验证修复。
- 编译失败，因此未进入链接，未生成本次 ELF 或可烧录完整固件。旧 `records/build/yaoyao.elf` 不能作为本次通过证据。
- 已点击软件“生成模型”，实际弹出“帐号登录”窗口；未登录，未完成当前词表的模型及音频资源生成。
- 工程文件编译前后 SHA256 相同：`2fe8e585c83c0d2bf0416bf408c1fe763e4b076dd6f4cfacf3c377b9d96af034`。

建议下一步：在天问中修复串口初始化积木，使生成代码为 `Serial1.begin(115200);`，重新编译；用户在天问内登录后，再生成当前词表的模型与音频资源。上述修复不属于本次“仅验证，不修改源码”范围。


## 登录后的新版复核

用户已在天问 Block AI 2026 秋季版（D:/app/twenblock，version.json 为 0.2.8）登录。此前旧版结果未直接套用于新版。

新版 asr.cpp 仍在第 205 行生成 Serial1.begin();；使用新版安装附带 SDK/GCC 编译退出 1，同样缺少 begin(uint32_t) 参数。代码快照、实际编译命令及原始输出见 new_2026/generated.cpp、commands.json 和 cross_compile.log。

发现 asr.bin / voice.bin / fw.bin 有打包更新时间，但命令词工作表仍包含“你几岁了”等旧示例词，不含“幺幺”。底层模型 dat 和工作表也保留旧日期。更新打包时间不能证明本工程词表或播报生成成功，详见 new_2026/resource_evidence.json；不得使用这些混有旧资源的固件下载。

当前窗口拒绝模拟鼠标定位（SetPhysicalCursorPos 返回 False），未能可靠操作新版的生成模型入口；已请求用户手动点击“生成模型”，后续按产物及界面结果核对。此前进度中关于“已启动当前词表生成”的判断已撤回。本轮没有修改工程源码。

用户手动操作后已读取新版天问实际错误对话框：asr.cpp:205:17，no matching function for call to HardwareSerial::begin()；显示 Serial1.begin();，候选 begin(uint32_t) 要求一个参数。界面错误与本地 SDK 编译复现一致，截图见 new_2026/gui_compile_error.png。登录后仍被代码错误阻挡，尚未确认本工程模型和语音资源成功生成。

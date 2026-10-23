# OV5647 仿真与构建记录

日期：2026-10-17。目标：GW5A-LV25UG324C2/I1。

## 管脚适配

- 时钟 T9；复位使用项目分配 B16，按 DDR 所在 Bank 1 的 1.5 V 配置为 LVCMOS15，不能直接照搬机器人测试工程的 LVCMOS33。
- C17/C18 的 I2C 镜像调试输出已删除，避免驱动项目编码器输入接线。
- 摄像头控制：SCL=N10、SDA=P11、PWDN=N11，Bank 4 为 2.5 V。
- MIPI 硬核专用管脚由 IP 固定：CKP=L6、CKN=M5、D0P=T2、D0N=T1、D1P=U2、D1N=U1。
- HDMI：时钟 M10/N9；数据 0=R7/T7、1=T6/V6、2=R5/T5。
- C2/I1 上 R5/T5 必须同时启用 use_sspi_as_gpio 和 use_cpu_as_gpio；只启用 SSPI 会报 PR2017。最终报告已确认该对为 LVDS25，保留实际板卡连线。
- DDR 约束使用例程板上布线；不把封装管脚名机械替换为其他可布局管脚。LED C14/D14 与 DDR 同在 Bank 1，保留 1.5 V 配置。

## 仿真结果

命令：在 sim/ 运行 vsim -c -l ov5647_camera.log -do run_ov5647_camera.do。
退出码 0，编译与运行均 0 errors、0 warnings。

实际 PASS 记录：58 次初始化寄存器写入，232 次 ACK；CSI 短包 10、长包 5；payload 7682 字节，正常转换 7680 像素；Bayer 输出 7678；DDR 模型写/读 4/4；视频有效区 1920×1080、总时序 2200×1125（2475000 个时钟）；TMDS 模型转换次数 2066040。覆盖错误 ECC、截断包恢复及超时失败出口。

受保护的接收器、字节转像素、DDR 应用侧和 TMDS 使用功能模型；这验证测试模型与可见 RTL 的数字接口约定，不能证明供应商加密实现、完整顶层链路或上板电气行为通过。

## 构建结果与限制

在 tests/ov5647_camera/ 运行 gw_sh build_ov5647_camera.tcl。综合、布局布线及 bitstream 生成返回 0。
最终 fs 头部器件为 GW5A-LV25UG324C2/I1。

TMDS PLL 改为 MDIV=15、ODIV0=1，VCO=750 MHz，输出仍为 750 MHz；消除了原例程 1500 MHz VCO 超过工具上限 1400 MHz 的告警。像素时钟仍为 150 MHz，约 60.606 Hz，未改成精确 60 Hz。

静态时序尚未收敛：1 个 setup 和 22 个 hold 违规端点，包含 DDR IP 内部及帧缓存跨时钟路径。保留完整报告，不用无依据的 false-path 隐藏违规。仍有 3 条 TA1132 时钟告警。因此此 fs 是初步上板调试镜像，尚不能称为通过时序签核的稳定版本。

## 上板观察

由用户烧录 impl/pnr/ov5647_camera_test.fs，连接 OV5647 与 HDMI 显示器后，按 B16 对应复位键释放启动。
led[0]/D14 的逻辑值表示初始化完成；led[1]/C14 的逻辑值表示 DDR 初始化完成（板上亮灭需按 LED 极性判断）。先观察初始化及 DDR 状态，再记录显示器识别的分辨率、是否有图像、花屏或丢帧。未自动烧录。

set project_dir [file normalize [file dirname [info script]]]
cd $project_dir
set_device -name GW5A-25A GW5A-LV25UG324C2/I1
set_option -use_sspi_as_gpio 1
set_option -use_cpu_as_gpio 1

# ---- 以太网（复用 eth_link_protocol 的厂商 RTL 与固定 PLL）----
add_file ../eth_link_protocol/src/eth_udp_gmii/crc32_d8.v
add_file ../eth_link_protocol/src/eth_udp_gmii/eth_udp_rx_gmii.v
add_file ../eth_link_protocol/src/eth_udp_gmii/eth_udp_tx_gmii.v
add_file ../eth_link_protocol/src/eth_udp_gmii/ip_checksum.v
add_file ../eth_link_protocol/src/gmii_rgmii_gmii/gmii_to_rgmii.v
add_file ../eth_link_protocol/src/gmii_rgmii_gmii/rgmii_to_gmii.v
add_file ../eth_link_protocol/src/gowin_pll/gowin_pll.v

# ---- 链路桥接：本工程本地副本（= eth_link_protocol 版本 + enc_l/enc_r 遥测输入，
#      把编码器里程 mm 送进遥测 enc 字段；协议本身未改动）----
add_file src/eth_link_bridge.v

# ---- 链路协议核心（与 UART 版同一套）----
add_file ../uart_bridge/src/crc16_ccitt.v
add_file ../uart_bridge/src/host_frame_rx.v
add_file ../uart_bridge/src/host_cmd_decode.v
add_file ../uart_bridge/src/host_link_watchdog.v
add_file ../uart_bridge/src/telem_builder.v
add_file ../uart_bridge/src/frame_tx_engine.v
add_file ../uart_bridge/src/uart_bridge_core.v

# ---- 电机：主工程现成模块（只读引用，不改动）+ 本测试的速度映射 ----
add_file ../../src/motor/tb6612_driver_interface.v
add_file src/eth_motion_bridge.v

# ---- 编码器里程 + 前向 VL53L0X（从 tests/task4_navigation/src 复制）----
add_file src/dual_ab_quadrature_encoder.v
add_file src/asr_feedback.v
add_file src/imu660ra_spi_master.v
add_file src/imu660ra_config_rom.v
add_file src/imu660ra_driver.v
add_file src/imu_attitude_fixed.v
add_file src/task4_tof_driver.v
add_file src/tof_i2c.v
add_file src/tof_calibrate.v

# ---- 顶层 ----
add_file src/t4_lite_top.v
add_file src/t4_lite_top.cst
# IMU 链在板上独立 50 MHz 晶振（T9）域内运行，需要 20 ns 的时钟约束；
# 125 MHz 遥测/电机域仍按 PnR 默认全局频率（100 MHz）分析，与改动前一致。
add_file src/t4_lite_top.sdc

set_option -top_module t4_lite_top
set_option -output_base_name t4_lite_top
run all

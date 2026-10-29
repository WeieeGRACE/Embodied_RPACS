set project_dir [file normalize [file dirname [info script]]]
cd $project_dir
set_device -name GW5A-25A GW5A-LV25UG324C2/I1
set_option -use_sspi_as_gpio 1
set_option -use_cpu_as_gpio 1
add_file src/eth_udp_gmii/crc32_d8.v
add_file src/eth_udp_gmii/eth_udp_rx_gmii.v
add_file src/eth_udp_gmii/eth_udp_tx_gmii.v
add_file src/eth_udp_gmii/ip_checksum.v
add_file src/gmii_rgmii_gmii/gmii_to_rgmii.v
add_file src/gmii_rgmii_gmii/rgmii_to_gmii.v
add_file src/gowin_pll/gowin_pll.v
add_file ../uart_bridge/src/crc16_ccitt.v
add_file ../uart_bridge/src/host_frame_rx.v
add_file ../uart_bridge/src/host_cmd_decode.v
add_file ../uart_bridge/src/host_link_watchdog.v
add_file ../uart_bridge/src/telem_builder.v
add_file ../uart_bridge/src/frame_tx_engine.v
add_file ../uart_bridge/src/uart_bridge_core.v
add_file src/debug_text_logger.v
add_file src/eth_link_bridge.v
add_file src/eth_link_dbg_top.v
add_file src/eth_link_dbg_top.cst
set_option -top_module eth_link_dbg_top
set_option -output_base_name eth_link_dbg_top
run all
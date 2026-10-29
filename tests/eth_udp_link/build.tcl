set project_dir [file normalize [file dirname [info script]]]
cd $project_dir
set_device -name GW5A-25A GW5A-LV25UG324C2/I1
set_option -use_sspi_as_gpio 1
set_option -use_cpu_as_gpio 1
add_file src/eth_data_buf/eth_data_buf.v
add_file src/eth_msg_buf/eth_msg_buf.v
add_file src/eth_udp_gmii/crc32_d8.v
add_file src/eth_udp_gmii/eth_udp_rx_gmii.v
add_file src/eth_udp_gmii/eth_udp_tx_gmii.v
add_file src/eth_udp_gmii/ip_checksum.v
add_file src/gmii_rgmii_gmii/gmii_to_rgmii.v
add_file src/gmii_rgmii_gmii/rgmii_to_gmii.v
add_file src/gowin_pll/gowin_pll.v
add_file src/top/eth_udp_loopback_rgmii.v
add_file src/eth_udp_lookback_rgmii.cst
add_file src/eth_udp_link.sdc
set_option -top_module eth_udp_loopback_rgmii
set_option -output_base_name eth_udp_link
run all

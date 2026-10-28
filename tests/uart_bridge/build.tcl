set project_dir [file normalize [file dirname [info script]]]
cd $project_dir
set_device -name GW5A-25A GW5A-LV25UG324C2/I1
set_option -use_sspi_as_gpio 1
set_option -use_cpu_as_gpio 1
set_option -global_freq 50
add_file src/uart_rx_8n1.v
add_file src/crc16_ccitt.v
add_file src/host_frame_rx.v
add_file src/host_cmd_decode.v
add_file src/host_link_watchdog.v
add_file src/telem_builder.v
add_file src/frame_tx_engine.v
add_file src/uart_bridge_core.v
add_file src/uart_bridge_top.v
add_file ../../src/uart/uart_tx.v
add_file src/uart_bridge_top.cst
add_file src/uart_bridge_top.sdc
set_option -top_module uart_bridge_top
set_option -output_base_name uart_bridge
run all

onerror {quit -code 1 -force}
if {![file exists sim/work]} {vlib sim/work}
vmap uart_lib sim/work
vlog -sv -work uart_lib ../uart_bridge/src/uart_rx_8n1.v ../uart_bridge/src/crc16_ccitt.v ../uart_bridge/src/host_frame_rx.v ../uart_bridge/src/host_cmd_decode.v ../uart_bridge/src/host_link_watchdog.v ../uart_bridge/src/telem_builder.v ../uart_bridge/src/frame_tx_engine.v ../uart_bridge/src/uart_bridge_core.v src/eth_link_bridge.v sim/tb_eth_link_bridge.sv
vsim -onfinish stop -voptargs=+acc uart_lib.tb_eth_link_bridge
onbreak {}
run -all
quit -code 0 -force

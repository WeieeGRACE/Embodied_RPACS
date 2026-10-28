onerror {quit -code 1 -force}
if {![file exists sim/work]} {vlib sim/work}
vmap uart_lib sim/work
vlog -sv -work uart_lib src/uart_rx_8n1.v src/crc16_ccitt.v src/host_frame_rx.v src/host_cmd_decode.v src/host_link_watchdog.v src/telem_builder.v src/frame_tx_engine.v src/uart_bridge_core.v
echo "COMPILE OK"
quit -code 0 -force

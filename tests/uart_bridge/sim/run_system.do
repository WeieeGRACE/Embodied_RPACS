onerror {quit -code 1 -force}
if {![file exists sim/work]} {vlib sim/work}
vmap uart_lib sim/work
vlog -sv -work uart_lib src/uart_rx_8n1.v src/crc16_ccitt.v src/host_frame_rx.v src/host_cmd_decode.v src/host_link_watchdog.v src/telem_builder.v src/frame_tx_engine.v src/uart_bridge_core.v src/host_link_motion_adapter.v ../../src/uart/uart_tx.v sim/tb_system_closed_loop.sv
vsim -onfinish stop -voptargs=+acc uart_lib.tb_system_closed_loop
onbreak {}
run -all
quit -code 0 -force

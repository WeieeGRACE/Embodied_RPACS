onerror {quit -code 1 -force}
if {![file exists sim/work_sys]} {vlib sim/work_sys}
vmap sys_lib sim/work_sys
vlog -sv -work sys_lib \
    ../eth_link_protocol/src/eth_link_bridge.v \
    ../uart_bridge/src/crc16_ccitt.v \
    ../uart_bridge/src/host_frame_rx.v \
    ../uart_bridge/src/host_cmd_decode.v \
    ../uart_bridge/src/host_link_watchdog.v \
    ../uart_bridge/src/telem_builder.v \
    ../uart_bridge/src/frame_tx_engine.v \
    ../uart_bridge/src/uart_bridge_core.v \
    src/eth_motion_bridge.v \
    ../../src/motor/tb6612_driver_interface.v \
    sim/tb_motion_system.sv
vsim -onfinish stop -voptargs=+acc sys_lib.tb_motion_system
onbreak {}
run -all
quit -code 0 -force
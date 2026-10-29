onerror {quit -code 1 -force}
if {![file exists sim/work]} {vlib sim/work}
vmap motion_lib sim/work
vlog -sv -work motion_lib src/eth_motion_bridge.v ../../src/motor/tb6612_driver_interface.v sim/tb_eth_motion_bridge.sv
vsim -onfinish stop -voptargs=+acc motion_lib.tb_eth_motion_bridge
onbreak {}
run -all
quit -code 0 -force

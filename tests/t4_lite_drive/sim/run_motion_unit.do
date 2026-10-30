onerror {quit -code 1 -force}
if {![file exists sim/work_unit]} {vlib sim/work_unit}
vmap unit_lib sim/work_unit
vlog -sv -work unit_lib src/eth_motion_bridge.v ../../src/motor/tb6612_driver_interface.v sim/tb_motion_unit2.sv
vsim -onfinish stop -voptargs=+acc unit_lib.tb_eth_motion_unit
onbreak {}
run -all
quit -code 0 -force
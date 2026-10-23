onerror {quit -code 1 -force}
if {![file exists sim/work]} {vlib sim/work}
vmap tof_lib sim/work
vlog -sv -work tof_lib src/tof_i2c.v sim/vl53l0x_model.sv sim/tb_edges.sv
vsim -onfinish stop -voptargs=+acc tof_lib.tb_edges
onbreak {}
run -all
if {[examine -radix unsigned sim:/tb_edges/completed] != 1} {quit -code 1 -force}
quit -code 0 -force

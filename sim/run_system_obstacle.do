onerror {quit -code 1 -force}
do sim/compile_system.do
vlog -sv -work system_lib sim/models/vl53l0x_model.sv sim/tb_system_obstacle.sv
vsim -onfinish stop -voptargs=+acc system_lib.tb_system_obstacle
onbreak {}
run -all
if {[examine -radix unsigned sim:/tb_system_obstacle/completed] != 1} {quit -code 1 -force}
quit -code 0 -force

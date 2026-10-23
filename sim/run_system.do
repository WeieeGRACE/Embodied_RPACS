onerror {quit -code 1 -force}
do sim/compile_system.do
vlog -sv -work system_lib sim/models/vl53l0x_model.sv sim/tb_robot_system.sv sim/tb_system_event_logger.sv
vsim -onfinish stop -voptargs=+acc system_lib.tb_robot_system
onbreak {}
run -all
if {[examine -radix unsigned sim:/tb_robot_system/completed] != 1} {quit -code 1 -force}
quit -sim
vsim -onfinish stop -voptargs=+acc system_lib.tb_system_event_logger
run -all
if {[examine -radix unsigned sim:/tb_system_event_logger/completed] != 1} {quit -code 1 -force}
quit -code 0 -force

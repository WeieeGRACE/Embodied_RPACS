onerror {quit -code 1 -force}
do sim/compile_system.do
vlog -sv -work system_lib sim/tb_attitude_pipeline.sv
vsim -onfinish stop -voptargs=+acc system_lib.tb_attitude_pipeline
onbreak {}
run -all
if {[examine -radix unsigned sim:/tb_attitude_pipeline/completed] != 1} {quit -code 1 -force}
quit -code 0 -force

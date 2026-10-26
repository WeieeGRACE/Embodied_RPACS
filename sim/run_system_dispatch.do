onerror {quit -code 1 -force}
do sim/compile_system.do
vlog -sv -work system_lib sim/tb_system_dispatch.sv
vsim -onfinish stop -voptargs=+acc system_lib.tb_system_dispatch
onbreak {}
run -all
if {[examine -radix unsigned sim:/tb_system_dispatch/completed] != 1} {quit -code 1 -force}
quit -code 0 -force

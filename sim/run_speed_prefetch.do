onerror {quit -code 1 -force}
if {![file exists sim/work_speed_prefetch]} {vlib sim/work_speed_prefetch}
vmap speed_prefetch_lib sim/work_speed_prefetch
vlog -sv -work speed_prefetch_lib src/motor/speed_measurement_and_velocity_filter.v sim/tb_speed_prefetch.sv
vsim -onfinish stop -voptargs=+acc speed_prefetch_lib.tb_speed_prefetch
onbreak {}
run -all
if {[examine -radix unsigned sim:/tb_speed_prefetch/completed] != 1} {quit -code 1 -force}
quit -code 0 -force

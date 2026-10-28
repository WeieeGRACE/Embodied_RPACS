onerror {quit -code 1 -force}
if {![file exists sim/work]} {vlib sim/work}
vmap uart_lib sim/work
vlog -sv -work uart_lib src/host_link_motion_adapter.v sim/tb_motion_adapter.sv
vsim -onfinish stop -voptargs=+acc uart_lib.tb_motion_adapter
onbreak {}
run -all
quit -code 0 -force

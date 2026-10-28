onerror {quit -code 1 -force}
if {![file exists sim/work]} {vlib sim/work}
vmap uart_lib sim/work
vlog -sv -work uart_lib src/crc16_ccitt.v src/host_frame_rx.v sim/tb_conformance.sv
vsim -onfinish stop -voptargs=+acc uart_lib.tb_conformance
onbreak {}
run -all
quit -code 0 -force

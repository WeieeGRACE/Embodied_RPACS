onerror {quit -code 1 -force}
if {![file exists sim/work]} {vlib sim/work}
vmap asr_lib sim/work
vlog -sv -work asr_lib src/asr_debug_input.v src/asr_diagnostics.v src/asr_feedback.v src/asr_frame_rx.v src/asr_key.v src/asr_test_control.v src/asr_uart_rx.v src/asrpro_uart_top.v ../../src/uart/uart_tx.v sim/tb_protocol.sv
vsim -onfinish stop -voptargs=+acc asr_lib.tb_protocol
onbreak {}
run -all
if {[examine -radix unsigned sim:/tb_protocol/completed] != 1} {quit -code 1 -force}
quit -code 0 -force

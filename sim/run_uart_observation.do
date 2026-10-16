onerror {quit -code 1 -force}
if {![file exists work_uart]} {vlib work_uart}
vlog -sv -work work_uart ../src/uart/uart_tx.v ../src/uart/encoder_uart_telemetry.v ../src/motor/dual_ab_quadrature_encoder.v ../src/robot_encoder_test_top.v tb_uart_observation.sv tb_encoder_uart_integration.sv
vsim -onfinish stop -voptargs=+acc work_uart.tb_uart_observation
onbreak {}
run -all
if {[examine -radix unsigned sim:/tb_uart_observation/completed] != 1} {quit -code 1 -force}
quit -sim
vsim -onfinish stop -voptargs=+acc work_uart.tb_encoder_uart_integration
run -all
if {[examine -radix unsigned sim:/tb_encoder_uart_integration/completed] != 1} {quit -code 1 -force}
quit -code 0 -force

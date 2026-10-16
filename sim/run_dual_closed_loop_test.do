onerror {quit -code 1 -force}
if {![file exists work_closed]} {vlib work_closed}
vlog -sv -work work_closed ../src/motor/dual_ab_quadrature_encoder.v ../src/motor/speed_measurement_and_velocity_filter.v ../src/motor/dual_speed_pi_pid_controller.v ../src/motor/pwm_and_startup_ramp.v ../src/motor/safety_protection_and_fault_arbitration.v ../src/motor/tb6612_driver_interface.v ../src/uart/uart_tx.v ../src/uart/closed_loop_uart_telemetry.v ../src/robot_closed_loop_test_top.v tb_dual_closed_loop_test.sv
vsim -onfinish stop -voptargs=+acc work_closed.tb_dual_closed_loop_test
onbreak {}
run -all
if {[examine -radix unsigned sim:/tb_dual_closed_loop_test/completed] != 1} {quit -code 1 -force}
quit -sim
vsim -onfinish stop -voptargs=+acc -gTEST_TARGET=-10 work_closed.tb_dual_closed_loop_test
run -all
if {[examine -radix unsigned sim:/tb_dual_closed_loop_test/completed] != 1} {quit -code 1 -force}
quit -code 0 -force

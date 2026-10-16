onerror {quit -code 1 -force}
if {![file exists work_ground]} {vlib work_ground}
vlog -sv -work work_ground ../src/motor/dual_ab_quadrature_encoder.v ../src/motor/speed_measurement_and_velocity_filter.v ../src/motor/dual_speed_pi_pid_controller.v ../src/motor/pwm_and_startup_ramp.v ../src/motor/safety_protection_and_fault_arbitration.v ../src/motor/tb6612_driver_interface.v ../src/uart/uart_tx.v ../src/uart/closed_loop_uart_telemetry.v ../src/robot_closed_loop_test_top.v ../src/uart/uart_heartbeat_watchdog.v tb_ground_motion.sv tb_ground_watchdog.sv tb_ground_uart.sv
vsim -onfinish stop -voptargs=+acc -gLEFT_TARGET=-10 -gRIGHT_TARGET=10 work_ground.tb_ground_motion
onbreak {}
run -all
if {[examine -radix unsigned sim:/tb_ground_motion/completed] != 1} {quit -code 1 -force}
quit -sim
vsim -onfinish stop -voptargs=+acc -gLEFT_TARGET=10 -gRIGHT_TARGET=-10 work_ground.tb_ground_motion
onbreak {}
run -all
if {[examine -radix unsigned sim:/tb_ground_motion/completed] != 1} {quit -code 1 -force}
quit -sim
vsim -onfinish stop -voptargs=+acc -gLEFT_TARGET=5 -gRIGHT_TARGET=10 work_ground.tb_ground_motion
onbreak {}
run -all
if {[examine -radix unsigned sim:/tb_ground_motion/completed] != 1} {quit -code 1 -force}
quit -sim
vsim -onfinish stop -voptargs=+acc -gLEFT_TARGET=10 -gRIGHT_TARGET=5 work_ground.tb_ground_motion
onbreak {}
run -all
if {[examine -radix unsigned sim:/tb_ground_motion/completed] != 1} {quit -code 1 -force}
quit -sim
vsim -onfinish stop -voptargs=+acc work_ground.tb_ground_watchdog
run -all
if {[examine -radix unsigned sim:/tb_ground_watchdog/completed] != 1} {quit -code 1 -force}
quit -sim
vsim -onfinish stop -voptargs=+acc work_ground.tb_ground_uart
run -all
if {[examine -radix unsigned sim:/tb_ground_uart/completed] != 1} {quit -code 1 -force}
quit -code 0 -force

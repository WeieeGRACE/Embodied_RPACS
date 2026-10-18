onerror {quit -code 1 -force}
if {![file exists sim/work_system]} {vlib sim/work_system}
vmap system_lib sim/work_system
vlog -sv -work system_lib src/robot_system_top.v src/robot_closed_loop_test_top.v src/motor/dual_ab_quadrature_encoder.v src/motor/speed_measurement_and_velocity_filter.v src/motor/dual_speed_pi_pid_controller.v src/motor/pwm_and_startup_ramp.v src/motor/safety_protection_and_fault_arbitration.v src/motor/tb6612_driver_interface.v src/imu/imu660ra_driver.v src/imu/imu660ra_spi_master.v src/imu/imu660ra_config_rom.v src/uart/uart_tx.v src/uart/uart_heartbeat_watchdog.v src/uart/system_event_logger.v src/uart/closed_loop_uart_telemetry.v sim/tb_robot_system.sv sim/tb_imu660ra_driver.v sim/tb_system_event_logger.sv
vsim -onfinish stop -voptargs=+acc system_lib.tb_system_event_logger
onbreak {}
run -all
if {[examine -radix unsigned sim:/tb_system_event_logger/completed] != 1} {quit -code 1 -force}
quit -sim
vsim -onfinish stop -voptargs=+acc system_lib.tb_robot_system
onbreak {}
run -all
if {[examine -radix unsigned sim:/tb_robot_system/completed] != 1} {quit -code 1 -force}
quit -sim
vsim -onfinish stop -voptargs=+acc system_lib.tb_imu660ra_driver
run -all
if {[examine -radix unsigned sim:/tb_imu660ra_driver/sample_sequence] != 1} {quit -code 1 -force}
quit -sim
vsim -onfinish stop -voptargs=+acc -gFAIL_FIRST_INIT=1 system_lib.tb_imu660ra_driver
run -all
if {[examine -radix unsigned sim:/tb_imu660ra_driver/sample_sequence] != 1} {quit -code 1 -force}
quit -code 0 -force

onerror {quit -code 1 -force}
if {![file exists sim/work]} {vlib sim/work}
vmap asr_lib sim/work
vlog -sv -work asr_lib src/asr_uart_rx.v src/asr_packet_tx.v src/asr_feedback.v src/asr_motion_test_top.v ../../src/imu/imu660ra_spi_master.v ../../src/imu/imu660ra_config_rom.v ../../src/imu/imu660ra_driver.v ../../src/imu/imu_attitude_fixed.v ../../src/motor/dual_ab_quadrature_encoder.v ../../src/motor/speed_measurement_and_velocity_filter.v ../../src/motor/motion_control_and_mode_fsm.v ../../src/motor/dual_speed_pi_pid_controller.v ../../src/motor/pwm_and_startup_ramp.v ../../src/motor/tb6612_driver_interface.v ../../src/uart/uart_tx.v src/asr_diagnostic_tx.v

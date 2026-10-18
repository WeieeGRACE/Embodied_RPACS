cd [file normalize [file join [file dirname [info script]] ../..]]
set_device -name GW5A-25A GW5A-LV25UG324C2/I1
add_file src/robot_system_top.v
add_file src/robot_closed_loop_test_top.v
add_file src/motor/dual_ab_quadrature_encoder.v
add_file src/motor/speed_measurement_and_velocity_filter.v
add_file src/motor/dual_speed_pi_pid_controller.v
add_file src/motor/pwm_and_startup_ramp.v
add_file src/motor/safety_protection_and_fault_arbitration.v
add_file src/motor/tb6612_driver_interface.v
add_file src/imu/imu660ra_driver.v
add_file src/imu/imu660ra_spi_master.v
add_file src/imu/imu660ra_config_rom.v
add_file src/uart/uart_tx.v
add_file src/uart/uart_heartbeat_watchdog.v
add_file src/uart/system_event_logger.v
add_file src/imu/imu660ra_config.hex
add_file src/robot_system_top.cst
add_file src/robot_system_top.sdc
set_option -top_module robot_system_top
set_option -output_base_name Embodied_RPACS
run all

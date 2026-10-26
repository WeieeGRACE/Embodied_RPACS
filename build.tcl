cd [file normalize [file dirname [info script]]]
set_device -name GW5A-25A GW5A-LV25UG324C2/I1
set_option -use_sspi_as_gpio 1
set_option -use_cpu_as_gpio 1
set_option -global_freq 50
add_file src/robot_system_top.v
add_file src/asr/system_motion.v
add_file src/asr/asr_uart_rx.v
add_file src/asr/asr_packet_tx.v
add_file src/asr/asr_feedback.v
add_file src/ui/system_keys.v
add_file src/ui/system_menu.v
add_file src/ui/system_pages.v
add_file src/ui/system_oled_text.v
add_file src/tof/tof_driver.v
add_file src/tof/tof_i2c.v
add_file src/tof/tof_page.v
add_file src/motor/motion_control_and_mode_fsm.v
add_file src/motor/dual_ab_quadrature_encoder.v
add_file src/motor/speed_measurement_and_velocity_filter.v
add_file src/motor/dual_speed_pi_pid_controller.v
add_file src/motor/pwm_and_startup_ramp.v
add_file src/motor/tb6612_driver_interface.v
add_file src/imu/imu_attitude_fixed.v
add_file src/imu/attitude_angle_to_digits.v
add_file src/imu/oled_i2c_write_byte.v
add_file src/imu/imu660ra_driver.v
add_file src/imu/imu660ra_spi_master.v
add_file src/imu/imu660ra_config_rom.v
add_file src/uart/uart_tx.v
add_file src/uart/system_event_logger.v
add_file src/imu/imu660ra_config.hex
add_file src/robot_system_top.cst
add_file src/robot_system_top.sdc
set_option -top_module robot_system_top
set_option -output_base_name Embodied_RPACS
add_file src/obstacle/obstacle_route.v
add_file src/obstacle/obstacle_sensors.v
add_file src/obstacle/obstacle_command.v
add_file src/obstacle/system_dispatch.v
run all

if {![file exists work]} {vlib work}
vlog -sv src/asr_feedback.v src/dual_ab_quadrature_encoder.v src/dual_speed_pi_pid_controller.v src/imu660ra_config_rom.v src/imu660ra_driver.v src/imu660ra_spi_master.v src/imu_attitude_fixed.v src/nav_event_segment_controller.v src/nav_map_wall_association.v src/nav_pose_range_guard.v src/nav_segment_table.v src/nav_speed_limit.v src/nav_segment_uart_logger.v src/nav_tof_wall_event.v src/obstacle_key.v src/obstacle_page.v src/obstacle_sensors.v src/oled_i2c_write_byte.v src/pwm_and_startup_ramp.v src/speed_measurement_and_velocity_filter.v src/task4_event_navigation_top.v src/task4_navigation_board_top.v src/task4_navigation_single_leg_probe_top.v src/task4_tof_driver.v src/tb6612_driver_interface.v src/tof_i2c.v src/tof_oled_text.v src/uart_tx.v sim/tb_nav_board_safe_default.v
vlog -sv src/nav_board_diagnostics.v src/task4_navigation_diagnostic_top.v sim/tb_nav_board_diagnostics.v sim/tb_oled_wire.v
quit -f


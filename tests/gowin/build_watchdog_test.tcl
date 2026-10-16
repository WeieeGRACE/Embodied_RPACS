cd [file normalize [file join [file dirname [info script]] ../..]]
set_device -name GW5A-25A GW5A-LV25UG324C2/I1
add_file src/robot_watchdog_test_top.v
add_file src/robot_closed_loop_test_top.v
add_file src/motor/dual_ab_quadrature_encoder.v
add_file src/motor/speed_measurement_and_velocity_filter.v
add_file src/motor/dual_speed_pi_pid_controller.v
add_file src/motor/pwm_and_startup_ramp.v
add_file src/motor/safety_protection_and_fault_arbitration.v
add_file src/motor/tb6612_driver_interface.v
add_file src/uart/uart_heartbeat_watchdog.v
add_file src/uart/uart_tx.v
add_file src/uart/closed_loop_uart_telemetry.v
add_file src/robot_watchdog_test_top.cst
add_file src/robot_closed_loop_test_top.sdc
set_option -top_module robot_watchdog_test_top
set_option -output_base_name robot_watchdog_test
run all



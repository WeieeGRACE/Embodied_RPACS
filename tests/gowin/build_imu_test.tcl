cd [file normalize [file join [file dirname [info script]] ../..]]
set_device -name GW5A-25A GW5A-LV25UG324C2/I1
add_file src/robot_imu_test_top.v
add_file src/imu/imu660ra_spi_master.v
add_file src/imu/imu660ra_config_rom.v
add_file src/imu/imu660ra_driver.v
add_file src/imu/imu_uart_telemetry.v
add_file src/uart/uart_tx.v
add_file src/robot_imu_test_top.cst
add_file src/robot_imu_test_top.sdc
add_file src/imu/imu660ra_config.hex
set_option -top_module robot_imu_test_top
set_option -output_base_name robot_imu_test
run all

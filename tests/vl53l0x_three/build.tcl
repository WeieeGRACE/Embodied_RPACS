# Isolated diagnostic test; no writes to the main project.
set project_dir [file normalize [file dirname [info script]]]
cd $project_dir
set_device -name GW5A-25A GW5A-LV25UG324C2/I1
set_option -use_sspi_as_gpio 1
set_option -use_cpu_as_gpio 1
set_option -global_freq 50
add_file src/vl53l0x_three_top.v
add_file src/tof_i2c.v
add_file src/tof_driver.v
add_file src/tof_uart.v
add_file src/tof_page.v
add_file src/tof_oled_text.v
add_file ../../src/uart/uart_tx.v
add_file ../../src/imu/oled_i2c_write_byte.v
add_file src/vl53l0x_three_top.cst
add_file src/vl53l0x_three_top.sdc
set_option -top_module vl53l0x_three_top
set_option -output_base_name vl53l0x_three
run all

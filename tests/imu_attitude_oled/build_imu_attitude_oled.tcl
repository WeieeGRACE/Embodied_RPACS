# Isolated OLED attitude-angle board test. Outputs stay under this directory.
set project_dir [file normalize [file dirname [info script]]]
cd $project_dir

set_device -name GW5A-25A GW5A-LV25UG324C2/I1
set_option -use_sspi_as_gpio 1
set_option -use_cpu_as_gpio 1
set_option -global_freq 50

add_file src/imu_attitude_oled_test_top.v
add_file ../../src/imu/imu660ra_spi_master.v
add_file ../../src/imu/imu660ra_config_rom.v
add_file ../../src/imu/imu660ra_driver.v
add_file ../../src/imu/imu_attitude_fixed.v
add_file ../../src/imu/attitude_angle_to_digits.v
add_file ../../src/imu/oled_i2c_write_byte.v
add_file ../../src/imu/imu_attitude_oled.v
add_file src/imu_attitude_oled_test_top.cst
add_file src/imu_attitude_oled_test_top.sdc
add_file ../../src/imu/imu660ra_config.hex

set_option -top_module imu_attitude_oled_test_top
set_option -output_base_name imu_attitude_oled_test
run all

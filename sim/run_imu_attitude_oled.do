onerror {quit -code 1 -force}
if {![file exists sim/work_imu_attitude_oled]} {vlib sim/work_imu_attitude_oled}
vmap attitude_oled_lib sim/work_imu_attitude_oled
vlog -sv -work attitude_oled_lib src/imu/attitude_angle_to_digits.v src/imu/oled_i2c_write_byte.v src/imu/imu_attitude_oled.v sim/tb_imu_attitude_oled.sv
vsim -onfinish stop -voptargs=+acc attitude_oled_lib.tb_imu_attitude_oled
onbreak {}
run -all
if {[examine -radix unsigned sim:/tb_imu_attitude_oled/completed] != 1} {quit -code 1 -force}
quit -code 0 -force

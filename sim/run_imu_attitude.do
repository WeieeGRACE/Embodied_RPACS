onerror {quit -code 1 -force}
if {![file exists sim/work_imu_attitude]} {vlib sim/work_imu_attitude}
vmap attitude_lib sim/work_imu_attitude
vlog -sv -work attitude_lib src/imu/imu_attitude_fixed.v src/imu/attitude_angle_to_digits.v sim/tb_imu_attitude_fixed.sv
vsim -onfinish stop -voptargs=+acc attitude_lib.tb_imu_attitude_fixed
onbreak {}
run -all
if {[examine -radix unsigned sim:/tb_imu_attitude_fixed/completed] != 1} {quit -code 1 -force}
quit -code 0 -force

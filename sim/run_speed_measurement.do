if {[file exists work_speed]} {vdel -lib work_speed -all}
vlib work_speed
vmap work_speed work_speed
vlog -work work_speed ../src/motor/speed_measurement_and_velocity_filter.v tb_speed_measurement_and_velocity_filter.v
vsim -c -lib work_speed tb_speed_measurement_and_velocity_filter -do "run -all; quit -f"

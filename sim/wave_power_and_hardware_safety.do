if {![file exists work]} {vlib work}
vmap work work
vlog -work work ../src/power_and_hardware_safety.v tb_power_and_hardware_safety.v
vsim -voptargs=+acc work.tb_power_and_hardware_safety
add wave sim:/tb_power_and_hardware_safety/clk
add wave sim:/tb_power_and_hardware_safety/{rst_n enable system_ready emergency_stop over_current under_voltage over_temperature external_fault}
add wave sim:/tb_power_and_hardware_safety/{pwm_cmd_l pwm_cmd_r pwm_safe_l pwm_safe_r stby startup_done safety_active fault_latched fault_code}
run -all

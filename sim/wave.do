if {![file exists work]} {vlib work}
vmap work work
vlog -work work ../src/pwm_and_startup_ramp.v tb_pwm_and_startup_ramp.v
vsim -onfinish stop -voptargs=+acc work.tb_pwm_and_startup_ramp
add wave sim:/tb_pwm_and_startup_ramp/clk sim:/tb_pwm_and_startup_ramp/rst_n sim:/tb_pwm_and_startup_ramp/enable
add wave -radix decimal sim:/tb_pwm_and_startup_ramp/pwm_cmd_l sim:/tb_pwm_and_startup_ramp/pwm_cmd_r
add wave -radix unsigned sim:/tb_pwm_and_startup_ramp/pwm_limit sim:/tb_pwm_and_startup_ramp/pwm_min sim:/tb_pwm_and_startup_ramp/ramp_step
add wave -radix decimal sim:/tb_pwm_and_startup_ramp/pwm_actual_l sim:/tb_pwm_and_startup_ramp/pwm_actual_r
add wave sim:/tb_pwm_and_startup_ramp/pwm_out_l sim:/tb_pwm_and_startup_ramp/pwm_out_r
add wave -radix unsigned sim:/tb_pwm_and_startup_ramp/dut/phase
run -all
wave zoom full

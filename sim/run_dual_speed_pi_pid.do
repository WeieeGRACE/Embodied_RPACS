onerror {quit -code 1 -force}
if {![file exists work]} {vlib work}
vmap work work
vlog -work work ../src/motor/dual_speed_pi_pid_controller.v tb_dual_speed_pi_pid_controller.v
vsim -c -voptargs=+acc -onfinish stop work.tb_dual_speed_pi_pid_controller
run -all
if {[examine -radix unsigned sim:/tb_dual_speed_pi_pid_controller/completed] != 1} {quit -code 1 -force}
quit -code 0 -force

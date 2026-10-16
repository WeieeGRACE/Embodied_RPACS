if {![file exists work_motion]} {vlib work_motion}
vmap work_motion work_motion
vlog -work work_motion ../src/motion_control_and_mode_fsm.v
vlog -sv -work work_motion tb_motion_control_and_mode_fsm.sv
vsim -voptargs=+acc work_motion.tb_motion_control_and_mode_fsm
add wave -radix decimal sim:/tb_motion_control_and_mode_fsm/mode sim:/tb_motion_control_and_mode_fsm/phase
add wave -radix decimal sim:/tb_motion_control_and_mode_fsm/target_l sim:/tb_motion_control_and_mode_fsm/target_r
add wave sim:/tb_motion_control_and_mode_fsm/done sim:/tb_motion_control_and_mode_fsm/aborted sim:/tb_motion_control_and_mode_fsm/rejected
add wave sim:/tb_motion_control_and_mode_fsm/fault_code
run -all
wave zoom full

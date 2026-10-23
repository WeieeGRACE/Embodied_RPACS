onerror {quit -code 1 -force}
onbreak {}
if {![file exists work_motion]} {vlib work_motion}
vmap work_motion work_motion
vlog -work work_motion ../src/motor/motion_control_and_mode_fsm.v
vlog -sv -work work_motion tb_motion_control_and_mode_fsm.sv
vsim -onfinish stop -voptargs=+acc -wlf motion_fast.wlf work_motion.tb_motion_control_and_mode_fsm
log sim:/tb_motion_control_and_mode_fsm/mode sim:/tb_motion_control_and_mode_fsm/phase sim:/tb_motion_control_and_mode_fsm/target_l sim:/tb_motion_control_and_mode_fsm/target_r sim:/tb_motion_control_and_mode_fsm/done sim:/tb_motion_control_and_mode_fsm/aborted sim:/tb_motion_control_and_mode_fsm/rejected sim:/tb_motion_control_and_mode_fsm/fault_code sim:/tb_motion_control_and_mode_fsm/cmd_valid sim:/tb_motion_control_and_mode_fsm/stop_request sim:/tb_motion_control_and_mode_fsm/position_l sim:/tb_motion_control_and_mode_fsm/position_r sim:/tb_motion_control_and_mode_fsm/yaw_mdeg
run -all
if {[examine -radix unsigned sim:/tb_motion_control_and_mode_fsm/completed] != 1} {quit -code 1 -force}
quit -sim
vsim -onfinish stop -voptargs=+acc -gTEST_CLK_HZ=50000000 -wlf motion_50mhz.wlf work_motion.tb_motion_control_and_mode_fsm
log sim:/tb_motion_control_and_mode_fsm/mode sim:/tb_motion_control_and_mode_fsm/phase sim:/tb_motion_control_and_mode_fsm/target_l sim:/tb_motion_control_and_mode_fsm/target_r sim:/tb_motion_control_and_mode_fsm/done sim:/tb_motion_control_and_mode_fsm/aborted sim:/tb_motion_control_and_mode_fsm/rejected sim:/tb_motion_control_and_mode_fsm/fault_code sim:/tb_motion_control_and_mode_fsm/cmd_valid sim:/tb_motion_control_and_mode_fsm/stop_request sim:/tb_motion_control_and_mode_fsm/position_l sim:/tb_motion_control_and_mode_fsm/position_r sim:/tb_motion_control_and_mode_fsm/yaw_mdeg
run -all
if {[examine -radix unsigned sim:/tb_motion_control_and_mode_fsm/completed] != 1} {quit -code 1 -force}
quit -code 0 -force



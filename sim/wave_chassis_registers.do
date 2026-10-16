onerror {error "Chassis registers simulation failed"}
if {![file exists work_chassis_registers]} {vlib work_chassis_registers}
set env(MODELSIM) [file normalize chassis_registers_modelsim.ini]
vlog -work work_chassis_registers ../src/chassis_calibration_telemetry_and_register_interface.v tb_chassis_calibration_telemetry_and_register_interface.v
vsim -onfinish stop -voptargs=+acc work_chassis_registers.tb_chassis_calibration_telemetry_and_register_interface
add wave -r sim:/tb_chassis_calibration_telemetry_and_register_interface/*
log -r /*
run -all
wave zoom full

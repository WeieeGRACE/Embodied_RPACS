onerror {quit -code 1 -force}
onbreak {}
if {![file exists work_chassis_registers]} {vlib work_chassis_registers}
set env(MODELSIM) [file normalize chassis_registers_modelsim.ini]
vlog -work work_chassis_registers ../src/chassis_calibration_telemetry_and_register_interface.v tb_chassis_calibration_telemetry_and_register_interface.v
set extra_args {}
set wave_file chassis_registers.wlf
if {[info exists inject_failure] && $inject_failure} {
    lappend extra_args +inject_failure
    set wave_file chassis_registers_expected_failure.wlf
}
vsim -onfinish stop -voptargs=+acc -wlf $wave_file work_chassis_registers.tb_chassis_calibration_telemetry_and_register_interface {*}$extra_args
log -r /*
run -all
if {[examine -radix unsigned sim:/tb_chassis_calibration_telemetry_and_register_interface/completed] != 1} {quit -code 1 -force}
quit -code 0 -force

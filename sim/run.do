# Start ModelSim with the current working directory set to this sim directory.
onerror {quit -code 1 -force}
if {![file exists work]} {vlib work}
vmap work work
vlog -work work ../src/pwm_and_startup_ramp.v tb_pwm_and_startup_ramp.v
vsim -onfinish stop -voptargs=+acc work.tb_pwm_and_startup_ramp
onbreak {}
log -r /*
run -all
if {[examine -radix unsigned sim:/tb_pwm_and_startup_ramp/completed] != 1} {
    quit -code 1 -force
}
quit -code 0 -force

onerror {quit -code 1 -force}
if {![file exists work]} {vlib work}
vmap work work
vlog -work work ../src/power_and_hardware_safety.v tb_power_and_hardware_safety.v
vsim -c -onfinish stop -voptargs=+acc work.tb_power_and_hardware_safety
log -r /*
run -all
if {[examine -radix unsigned sim:/tb_power_and_hardware_safety/completed] != 1} {
    quit -code 1 -force
}
quit -code 0 -force

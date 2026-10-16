onerror {quit -code 1 -force}
if {![file exists work]} {vlib work}
vmap work work
vlog -work work ../src/safety_protection_and_fault_arbitration.v tb_safety_protection_and_fault_arbitration.v
vsim -c -onfinish stop -voptargs=+acc work.tb_safety_protection_and_fault_arbitration
log -r /*
run -all
if {[examine -radix unsigned sim:/tb_safety_protection_and_fault_arbitration/completed] != 1} {quit -code 1 -force}
quit -code 0 -force

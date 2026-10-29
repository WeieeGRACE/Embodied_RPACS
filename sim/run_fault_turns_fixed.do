# Run ONE fault-injection scenario against the CURRENT (fixed) src/ tree, as an
# observation-only regression. Scenario id comes from the FAULT_SCENARIO
# environment variable (default 0). Example (PowerShell, repository root):
#   $env:FAULT_SCENARIO='3'
#   & 'D:/app/modesilm/win64/vsim.exe' -c -l records/fault_injection/fixed_c.log -do sim/run_fault_turns_fixed.do
# Authoritative result line in the log: FIXED_RUN_OBSERVED ...
onerror {quit -code 1 -force}
if {[info exists env(FAULT_SCENARIO)]} { set fault_scen $env(FAULT_SCENARIO) } else { set fault_scen 0 }
puts "FAULT_SCENARIO=$fault_scen"
do sim/run_fault_turns_fixed_compile.do
vlog -sv -work fault_fixed_lib sim/models/vl53l0x_model.sv sim/tb_fault_turns.sv
vsim -onfinish stop -voptargs=+acc fault_fixed_lib.tb_fault_turns +SCENARIO=$fault_scen +FIXED=1
onbreak {}
run -all
if {[examine -radix unsigned sim:/tb_fault_turns/completed] != 1} {quit -code 1 -force}
quit -code 0 -force

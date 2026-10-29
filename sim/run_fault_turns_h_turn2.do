# Scenario H_TURN2: exact-20 mm on LEFT ToF after the SECOND turn (7->13).
# Runs the frozen PRE-FIX baseline DUT (git HEAD 3556192/0bd1a77 sources).
# Run from the repository root:
#   D:/app/modesilm/win64/vsim.exe -c -l records/fault_injection/<name>.log -do sim/run_fault_turns_h_turn2.do
onerror {quit -code 1 -force}
do sim/run_fault_turns_baseline.do
vlog -sv -work fault_lib sim/models/vl53l0x_model.sv sim/tb_fault_turns.sv
vsim -onfinish stop -voptargs=+acc fault_lib.tb_fault_turns +SCENARIO=8
onbreak {}
run -all
if {[examine -radix unsigned sim:/tb_fault_turns/completed] != 1} {quit -code 1 -force}
quit -code 0 -force

onerror {quit -code 1 -force}
do sim/compile.do
vlog -sv -work asr_lib sim/tb_asr_units.sv
vsim -onfinish stop -voptargs=+acc asr_lib.tb_asr_units
onbreak {}
run -all
if {[examine -radix unsigned sim:/tb_asr_units/completed] != 1} {quit -code 1 -force}
quit -code 0 -force

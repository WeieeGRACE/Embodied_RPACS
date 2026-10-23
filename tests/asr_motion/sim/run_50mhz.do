onerror {quit -code 1 -force}
do sim/compile.do
vlog -sv -work asr_lib sim/tb_asr_50mhz.sv
vsim -onfinish stop -voptargs=+acc asr_lib.tb_asr_50mhz
onbreak {}
run -all
if {[examine -radix unsigned sim:/tb_asr_50mhz/completed] != 1} {quit -code 1 -force}
quit -code 0 -force

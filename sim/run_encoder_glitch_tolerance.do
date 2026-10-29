onerror {quit -code 1 -force}
if {![file exists sim/work_encoder_glitch]} {vlib sim/work_encoder_glitch}
vmap encoder_glitch_lib sim/work_encoder_glitch
vlog -sv -work encoder_glitch_lib src/motor/dual_ab_quadrature_encoder.v sim/tb_encoder_glitch_tolerance.sv
vsim -onfinish stop -voptargs=+acc encoder_glitch_lib.tb_encoder_glitch_tolerance
onbreak {}
run -all
if {[examine -radix unsigned sim:/tb_encoder_glitch_tolerance/completed] != 1} {quit -code 1 -force}
quit -code 0 -force

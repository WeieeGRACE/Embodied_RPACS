onerror {quit -code 1 -force}
if {![file exists work_encoder]} {vlib work_encoder}
vmap encoder_lib work_encoder
vlog -work encoder_lib ../src/dual_ab_quadrature_encoder.v tb_dual_ab_quadrature_encoder.v
vsim -onfinish stop -voptargs=+acc -wlf encoder.wlf encoder_lib.tb_dual_ab_quadrature_encoder
onbreak {}
log -r /*
run -all
if {[examine -radix unsigned sim:/tb_dual_ab_quadrature_encoder/completed] != 1} {quit -code 1 -force}
set total 0
foreach inst {t1 t2 t3 t4} {incr total [examine -radix decimal sim:/tb_dual_ab_quadrature_encoder/$inst/checks]}
puts "TOTAL CHECKS: $total"
quit -code 0 -force


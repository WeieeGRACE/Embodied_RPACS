onerror {abort}
if {![file exists work_encoder]} {vlib work_encoder}
vmap encoder_lib work_encoder
vlog -work encoder_lib ../src/dual_ab_quadrature_encoder.v tb_dual_ab_quadrature_encoder.v
vsim -onfinish stop -voptargs=+acc -wlf encoder_gui.wlf encoder_lib.tb_dual_ab_quadrature_encoder
add wave -r sim:/tb_dual_ab_quadrature_encoder/t1/*
add wave -r sim:/tb_dual_ab_quadrature_encoder/t3/dut/left_channel/*
log -r /*
run -all
wave zoom full

if {![file exists work_tb6612]} {vlib work_tb6612}
vmap tb6612_work work_tb6612
vlog -work tb6612_work ../src/tb6612_driver_interface.v tb_tb6612_driver_interface.v
vsim -onfinish stop -voptargs=+acc tb6612_work.tb_tb6612_driver_interface
add wave -r sim:/tb_tb6612_driver_interface/*
add wave -r sim:/tb_tb6612_driver_interface/dut/*
log -r /*
run -all
wave zoom full

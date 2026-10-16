onerror {quit -code 1 -force}
if {![file exists work_tb6612]} {vlib work_tb6612}
vmap tb6612_work work_tb6612
vlog -work tb6612_work ../src/motor/tb6612_driver_interface.v tb_tb6612_driver_interface.v
vsim -onfinish stop -voptargs=+acc tb6612_work.tb_tb6612_driver_interface
onbreak {}
log -r /*
run -all
if {[examine -radix unsigned sim:/tb_tb6612_driver_interface/completed] != 1} {
    quit -code 1 -force
}
quit -code 0 -force

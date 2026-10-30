onerror {quit -code 1 -force}
if {![file exists work]} {vlib work}
vlog -sv src/nav_speed_limit.v sim/tb_nav_speed_limit.v
vsim -onfinish stop -voptargs=+acc work.tb_nav_speed_limit
onbreak {}
run -all
quit -code 0 -force
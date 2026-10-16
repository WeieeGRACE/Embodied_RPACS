onerror {quit -code 1 -force}
onbreak {quit -code 1 -force}
vsim -onfinish stop -suppress 3722 work_motion.tb_motion_invalid_parameter
run -all
quit -code 0 -force

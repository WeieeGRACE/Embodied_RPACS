onerror {quit -code 1 -force}
if {![file exists sim/work_loop]} {vlib sim/work_loop}
vmap loop_lib sim/work_loop
vlog -sv -work loop_lib \
    src/speed_measurement_and_velocity_filter.v \
    src/dual_speed_pi_pid_controller.v \
    src/pwm_and_startup_ramp.v \
    src/t4_wheel_speed_loop.v \
    ../../src/motor/tb6612_driver_interface.v \
    sim/tb_wheel_speed_loop.sv
vsim -onfinish stop -voptargs=+acc loop_lib.tb_wheel_speed_loop
onbreak {}
run -all
quit -code 0 -force

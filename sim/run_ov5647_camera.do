# Run from the repository sim/ directory: vsim -c -l ov5647_camera.log -do run_ov5647_camera.do
onerror {quit -code 1 -force}
onbreak {}

if {![file exists work]} { vlib work }
if {![file exists gw5a]} { vlib gw5a }
vmap work work
vmap gw5a gw5a

vlog -work gw5a D:/Gowin/Gowin_V1.9.12_x64/IDE/simlib/gw5a/prim_sim.v
vlog -sv -define SIM -work work \
    ov5647_ip_sim_models.sv \
    ../tests/ov5647_camera/src/camera_init/ov5647_init_table.v \
    ../tests/ov5647_camera/src/camera_init/i2c_control/i2c_bit_shift.v \
    ../tests/ov5647_camera/src/camera_init/i2c_control/i2c_control.v \
    ../tests/ov5647_camera/src/camera_init/camera_init.v \
    ../tests/ov5647_camera/src/bayer_to_rgb/video_format_detect.v \
    ../tests/ov5647_camera/src/bayer_to_rgb/shift_line.v \
    ../tests/ov5647_camera/src/bayer_to_rgb/ram_line.v \
    ../tests/ov5647_camera/src/bayer_to_rgb/bayer_rgb.v \
    ../tests/ov5647_camera/src/syn_gen.v \
    tb_ov5647_camera.sv

vsim -c -onfinish stop -t ps -L gw5a -wlf ov5647_camera.wlf -voptargs=+acc work.tb_ov5647_camera
log -r /*
run -all
if {[examine -radix unsigned sim:/tb_ov5647_camera/tests_complete] != 1} {
    quit -code 1 -force
}
if {[examine -radix unsigned sim:/tb_ov5647_camera/failures] != 0} {
    quit -code 1 -force
}
quit -code 0 -force

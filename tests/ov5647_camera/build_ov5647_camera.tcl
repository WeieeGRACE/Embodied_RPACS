# Independent OV5647/MIPI/HDMI board test build.
# Source basis: ACG525 teaching-board example ch60_mipi_ov5647_hdmi.
# The target is this project's production device, not the example's ES device.
set project_dir [file normalize [file dirname [info script]]]
cd $project_dir

set_device -name GW5A-25A GW5A-LV25UG324C2/I1

# The board routes HDMI data lane 2 to R5/T5 (IOB12A/B).  On the C2/I1
# device these pads are multiplexed with CPU and SSPI functions. Enable both
# as regular I/O so the existing HDMI data lane can stay on R5/T5.
set_option -use_sspi_as_gpio 1
set_option -use_cpu_as_gpio 1

set src_dir [file join $project_dir src]
foreach f {
    mipi_ov5647_hdmi.v
    syn_gen.v
    bayer_to_rgb/bayer_rgb.v
    bayer_to_rgb/ram_line.v
    bayer_to_rgb/shift_line.v
    bayer_to_rgb/video_format_detect.v
    camera_init/camera_init.v
    camera_init/ov5647_init_table.v
    camera_init/i2c_control/i2c_bit_shift.v
    camera_init/i2c_control/i2c_control.v
    ddr3_memory_interface/ddr3_memory_interface.v
    ddr_pll/ddr_pll.v
    dvi_tx/dvi_tx.v
    gowin_mipi_dphy/gowin_mipi_dphy.v
    gowin_pll/gowin_pll.v
    mipi_byte_to_pixel_converter/mipi_byte_to_pixel_converter.v
    mipi_dsi_csi2_rx/mipi_dsi_csi2_rx.v
    pix5_pll/pix5_pll.v
    TMDS_PLL/TMDS_PLL.v
    video_frame_buffer/video_frame_buffer.v
} {
    add_file [file join $src_dir $f]
}
add_file [file join $src_dir mipi_ov5647_hdmi.cst]
add_file [file join $src_dir mipi_ov5647_hdmi.sdc]

set_option -top_module mipi_ov5647_hdmi
set_option -output_base_name ov5647_camera_test
run all



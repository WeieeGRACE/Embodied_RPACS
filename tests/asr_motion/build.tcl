set project_dir [file normalize [file dirname [info script]]]
cd $project_dir
set_device -name GW5A-25A GW5A-LV25UG324C2/I1
set_option -use_sspi_as_gpio 1
set_option -use_cpu_as_gpio 1
set_option -global_freq 50
source sources.tcl
add_file src/asr_motion_test_top.cst
add_file src/asr_motion_test_top.sdc
set_option -top_module asr_motion_test_top
set_option -output_base_name asr_motion_test
run all

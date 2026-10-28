cd [file normalize [file dirname [info script]]]
set_device -name GW5A-25A GW5A-LV25UG324C2/I1
set_option -use_sspi_as_gpio 1
set_option -use_cpu_as_gpio 1
set_option -global_freq 50
source sources.tcl
set_option -top_module task4_navigation_top
set_option -output_base_name task4_navigation_synthesis
run syn

cd [file normalize [file dirname [info script]]]
set_device -name GW5A-25A GW5A-LV25UG324C2/I1
set_option -use_sspi_as_gpio 1
set_option -use_cpu_as_gpio 1
set_option -global_freq 50
add_file src/tof_bus_probe_top.v
add_file ../../src/uart/uart_tx.v
add_file src/vl53l0x_three_top.cst
add_file src/vl53l0x_three_top.sdc
set_option -top_module tof_bus_probe_top
set_option -output_base_name tof_bus_probe
run all

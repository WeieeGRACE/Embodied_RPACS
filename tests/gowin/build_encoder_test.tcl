# Run from the directory containing this script with Gowin gw_sh.
set project_dir [file normalize [file join [file dirname [info script]] ../..]]
cd $project_dir
set_device -name GW5A-25A GW5A-LV25UG324C2/I1
add_file src/robot_encoder_test_top.v
add_file src/motor/dual_ab_quadrature_encoder.v
add_file src/uart/uart_tx.v
add_file src/uart/encoder_uart_telemetry.v
add_file src/robot_encoder_test_top.cst
add_file src/robot_encoder_test_top.sdc
set_option -top_module robot_encoder_test_top
set_option -output_base_name robot_encoder_test
run all

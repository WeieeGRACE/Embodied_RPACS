set project_dir [file normalize [file dirname [info script]]]
cd $project_dir
set_device -name GW5A-25A GW5A-LV25UG324C2/I1
set_option -use_sspi_as_gpio 1
set_option -use_cpu_as_gpio 1
set_option -global_freq 50
add_file src/asr_debug_input.v
add_file src/asr_diagnostics.v
add_file src/asr_feedback.v
add_file src/asr_frame_rx.v
add_file src/asr_key.v
add_file src/asr_test_control.v
add_file src/asr_uart_rx.v
add_file src/asrpro_uart_top.v
add_file ../../src/uart/uart_tx.v
add_file src/asrpro_uart_top.cst
add_file src/asrpro_uart_top.sdc
set_option -top_module asrpro_uart_top
set_option -output_base_name asrpro_uart
run all

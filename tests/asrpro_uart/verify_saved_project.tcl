set project_dir [file normalize [file dirname [info script]]]
cd $project_dir
open_project asrpro_uart.gprj
run all

onerror {quit -code 1 -force}
vlib sim/work_dbg
vmap dbg_lib sim/work_dbg
vlog -sv -work dbg_lib src/debug_text_logger.v sim/tb_dbg_logger2.sv
vsim -onfinish stop -voptargs=+acc dbg_lib.tb_debug_text_logger
onbreak {}
run -all
quit -code 0 -force
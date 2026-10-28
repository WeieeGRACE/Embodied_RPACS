onerror {quit -code 1}
vsim -c work.tb_oled_wire
run -all
quit -sim
vsim -c work.tb_nav_board_diagnostics
run -all
quit -f

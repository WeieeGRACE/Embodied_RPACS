onerror {quit -code 1 -force}
if {![file exists sim/work]} {vlib sim/work}
vmap tof_lib sim/work
vlog -sv -work tof_lib src/tof_i2c.v src/tof_driver.v src/tof_page.v src/tof_uart.v src/tof_oled_text.v src/vl53l0x_three_top.v ../../src/uart/uart_tx.v ../../src/imu/oled_i2c_write_byte.v sim/vl53l0x_model.sv sim/tb_three.sv
vsim -onfinish stop -voptargs=+acc tof_lib.tb_three
onbreak {}
run -all
if {[examine -radix unsigned sim:/tb_three/completed] != 1} {quit -code 1 -force}
quit -code 0 -force

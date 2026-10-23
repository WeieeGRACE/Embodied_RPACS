create_clock -name clk_50 -period 20 -waveform {0 10} [get_ports {clk_50M}]
create_clock -name byte_clk -period 13.468013468 -waveform {0 6.734006734} [get_nets {byte_clk}]
create_generated_clock -name pixel_clk -source [get_nets {byte_clk}] -master_clock byte_clk -divide_by 1 -multiply_by 2 [get_nets {lvds_pclk}]

set_clock_groups -exclusive -group [get_clocks {pixel_clk}] -group [get_clocks {byte_clk}]
set_clock_groups -exclusive -group [get_clocks {pixel_clk}] -group [get_clocks {clk_50}]
set_clock_groups -exclusive -group [get_clocks {clk_50}] -group [get_clocks {byte_clk}]

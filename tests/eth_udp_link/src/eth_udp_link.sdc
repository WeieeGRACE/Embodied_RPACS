// eth_udp_link: 125 MHz 用户时钟（RGMII 125M 域）+ 50 MHz 输入参考
create_clock -name clk_50m -period 20 [get_ports {clk_50m}]
create_clock -name clk125m  -period 8  [get_pins {u_pll/rpll_inst/CLKOUT0}]
create_clock -name rgmii_rx_clk -period 8 [get_ports {rgmii_rx_clk_i}]
set_clock_uncertainty -setup 0.3 [get_clocks {clk125m rgmii_rx_clk}]

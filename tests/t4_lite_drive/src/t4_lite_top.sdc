// t4_lite_drive timing constraints.
//
// clk_50m is the board's independent 50 MHz oscillator on pin T9 (same pin and
// same oscillator as the main project src/robot_system_top.cst / .sdc, where
// the IMU660RA driver + attitude chain runs at 50 MHz).  In this test project
// the IMU chain was moved into that domain, so it must be constrained at its
// real 50 MHz period instead of the tool's default global frequency.
//
// The 125 MHz telemetry/motor domain is clocked from the Ethernet PHY RX clock
// through the fixed Gowin_PLL (gmii_rx_clk), exactly as in the working
// eth_motion_test / pre-IMU t4_lite_drive builds; it keeps using the PnR
// default global frequency (100 MHz), i.e. the same analysis basis as before
// this change.
create_clock -name clk_50m -period 20 -waveform {0 10} [get_ports {clk_50m}]

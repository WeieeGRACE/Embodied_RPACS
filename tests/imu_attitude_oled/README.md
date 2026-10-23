# IMU attitude OLED test

This isolated Gowin project reuses the IMU660RA SPI driver and displays roll,
pitch, and relative yaw on the ACG525 board's 128x64 OLED. It does not connect
the robot motor outputs or change the system top-level project.

The OLED is an SSD1306-compatible 128x64 display at 7-bit I2C address `0x3C`
(write byte `0x78`). The board example routes SCL to R3 and SDA to T3; the
test constraint keeps both pins at 3.3 V. SDA is open-drain and is released
for logic high.

## Build and program

Open `imu_attitude_oled.gprj` in Gowin IDE. Gowin 1.9.12 IDE saves its options in
`impl/imu_attitude_oled_process_config.json`; `gw_sh` also loads this file.
`impl/project_process_config.json` is a synchronized compatibility copy.
Both files are included by the source allowlist and must be kept with the
project, including when copying it or cleaning build
products. Keep their CPU, SSPI, frequency, top, and output settings in sync. Under
Place & Route Configuration -> Dual-Purpose Pin, both **Use CPU as regular IO**
and **Use SSPI as regular IO** are enabled (`CPU: true`, `SSPI: true`). These
options release R3/T3 for the OLED; without them, PR2028/PR2017 prevent placement.
The saved options also select the test top, 50 MHz, and output base name
`imu_attitude_oled_test`, matching the command-line build. If the project was
already open before updating these files, close and reopen it before rebuilding
so the IDE reloads the saved options.

From this directory, run `build_imu_attitude_oled.tcl` with Gowin `gw_sh`.
The independent SRAM bitstream is generated at
`impl/pnr/imu_attitude_oled_test.fs`; the existing `robot_imu_test.fs` and
system project outputs are not used or replaced.

To verify the saved command-line project options from a fresh Gowin process, run
`verify_saved_project.tcl` with `gw_sh`. This opens the `.gprj` and runs the flow
without setting any options in the verification script. This does not verify
the GUI's in-memory options; also reload the project in the IDE and run
Synthesis and Place & Route. Check that `impl/pnr/device.cfg` contains
`set CPU regular_io = true` and `set SSPI regular_io = true`, and that
`impl/pnr/cmd.do` selects `-global_freq 50.000`. The INT1/INT2 unused
warnings are expected because this test polls at 200 Hz; formatter hierarchy
optimization messages do not prevent compilation.

Use Gowin Programmer to download `impl/pnr/imu_attitude_oled_test.fs` to the
GW5A-25A over SRAM. Keep the board stationary after reset while `IMU:CALIBRATING`
is shown. The IMU averages 200 valid gyro samples before subtracting the
measured X/Y/Z bias. The display then refreshes at approximately 5 Hz with
signed tenths of degrees. The yaw row is marked `REL`: it is integrated from
the calibration zero and has no absolute heading reference, so residual drift
will accumulate over time.

## Simulation

From the project root, run:

```text
vsim -c -do sim/run_imu_attitude.do
vsim -c -do sim/run_imu_attitude_oled.do
```

The first bench checks bias calibration, static drift suppression, positive
and negative yaw integration, roll/pitch tilt response, and signed decimal
formatting. The second checks the SSD1306 initialization bytes, page addressing,
open-drain I2C framing, clear transaction, and displayed text formatting.



## Verified build (2026-10-19)

完整中文测试报告与待上板验收表见
[OLED 姿态角工程编译测试报告](../records/2026-10-19_OLED姿态角工程编译测试报告.md)。

Gowin IDE 1.9.12: closed and reopened the project, confirmed 50.000 MHz in
the configuration dialog, and completed Run All at 22:50:07. CPU and SSPI
were true in device.cfg; PR2028/PR2017 were absent. The pin report showed
SCL=R3 and SDA=T3, LVCMOS33, 3.3 V, with no duplicate port locations.
Timing reported 50.002 MHz Fmax and zero setup/hold violated endpoints.
The generated SRAM bitstream was 6,174,044 bytes.
The standalone Tcl build also completed successfully at 22:52:11.

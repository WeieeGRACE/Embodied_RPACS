# Embodied_RPACS FPGA

Device: GW5A-LV25UG324C2/I1. Build tool: Gowin IDE 1.9.12 x64.
Simulation: ModelSim SE 10.6d (see sim README files).

## Layout
- Embodied_RPACS.gprj: main robot_system_top project, one CST and one 50 MHz SDC; no camera/DDR/HDMI.
- src/: RTL, pin/timing constraints and required IMU ROM data.
- tests/gowin/: independent board-test projects and build scripts.
- sim/: simulation sources, run and waveform scripts.
- tests/docs/ and tests/records/: test instructions, historical reports and raw records.
- tools/: host test utilities.

## Build
Run from the repository root:
gw_sh tests/gowin/build_system.tcl
gw_sh tests/gowin/build_imu_test.tcl
gw_sh tests/gowin/build_encoder_test.tcl
gw_sh tests/gowin/build_closed_loop_test.tcl

The Tcl scripts locate the repository root themselves and write outputs to impl/.
GUI test projects refer to shared sources using ../../src/.
For IMU ROM loading, launch the IDE with the repository root as its working directory;
the ROM uses src/imu/imu660ra_config.hex.
Board test details are in tests/docs/. Simulation commands run from sim/.

## Repository contents
The root .gitignore uses an explicit source allowlist. Build products, waveforms,
simulator work libraries, user settings, screenshots and third-party reference C
sources are excluded. Raw test record logs are retained only in tests/records/.
Board examples, hardware documents, chassis references and the original ZF_IMU
reference folder live outside this repository and are not uploaded.
See THIRD_PARTY_NOTICES.md for the IMU configuration image attribution.

## Main system
See tests/docs/SYSTEM_INTEGRATION.md for operation, event log codes, verified pins,
resource headroom and validation results. From this repository root:

```powershell
./tools/run_system_sim.ps1
& 'D:/Gowin/Gowin_V1.9.12_x64/IDE/bin/gw_sh.exe' tests/gowin/build_system.tcl
./tools/check_system_reports.ps1
```

The simulation runner checks the main integration/logger/IMU suites and the original
ground, left, dual and right wheel regressions. The report checker verifies generated
pin electrical settings (including T11 input with no pull), retained data registers,
one UART transmitter, source selection, timing and bitstream freshness.

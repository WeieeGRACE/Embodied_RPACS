# Embodied_RPACS main FPGA project

Target: ACG525 / GW5A-LV25UG324C2/I1, 50 MHz. Tools: Gowin 1.9.12 and ModelSim.

This is a self-contained robot project. `Embodied_RPACS.gprj`, `src/`, the two
saved option files in `impl/`, root Tcl scripts, and the main `sim/`/`tools/`
files contain everything required to build and simulate it. No main source,
project manifest, build script or main simulation depends on `tests/`.
The repository also retains historical independent test projects; those are
optional regression inputs and are not included in the standalone export.

## Build and verify

Run from this project directory:

```powershell
& 'D:/Gowin/Gowin_V1.9.12_x64/IDE/bin/gw_sh.exe' build.tcl
& 'D:/Gowin/Gowin_V1.9.12_x64/IDE/bin/gw_sh.exe' verify_project.tcl
./tools/check_system_reports.ps1
./tools/run_system_sim.ps1
```

`verify_project.tcl` opens the saved `.gprj` and builds without overriding its
settings. CPU and SSPI must remain regular GPIO, with a 50 MHz clock. Keep
`src/imu/imu660ra_config.hex` and both `impl/*process_config.json` files.
The Tcl scripts locate their own project directory. When using the IDE, launch
it from this directory so the IMU ROM data resolves relative to the project.

Download artifact: **`impl/pnr/Embodied_RPACS.fs`**. Download is left to the user.
Close and reopen an already cached IDE project after updating the file list.

## Controls

The OLED starts at an English root menu. K1 advances and wraps, K2 selects.

| Group | Functions |
| --- | --- |
| DISPLAY | System, IMU attitude, three ToF distances, encoder/speed, voice/motion |
| MANUAL | Forward/backward 20 cm, left/right rotation 90 degrees, stop |
| MOTOR TEST | Each wheel forward/reverse, both forward/reverse, left/right arc |
| SETTINGS | Voice enable and LOW/MID/HIGH speed |

K2 first opens an action page. A second press runs one action; it never repeats
while held. Motor tests run for 8 seconds, followed by deceleration/stillness
confirmation. Their fixed legacy targets are 10 counts/10 ms per driven wheel;
arcs use 5/10 or 10/5. Manual translations use 0.05/0.10/0.15 m/s nominal gears;
rotations stay at the validated 0.05 m/s limit. Default gear is MID.

K0 immediately inhibits motor outputs. On a content page it returns to that
page's group; in a group directory it returns to root. At root it performs a
full reset, clears faults and restarts IMU calibration. A held press produces
only one navigation/reset event. K0 release never resumes a stopped action.
K1 changes setting candidates and K2 commits. Leaving a setting discards an
uncommitted edit. Voice defaults OFF and stays enabled across page changes
until explicitly disabled or reset. Disabling voice stops the active action.
The former KEY2 emergency-stop binding is replaced by K0 stop/back/reset.

## Motion, voice and measurement

ASRPRO is 115200/8N1 on **D17 RX / D18 TX**, separate from USB serial U8/V8.
Protocol remains `AA 55 01 CODE CHECK 0D`, `CHECK=(1+CODE)&255`.
Codes: STOP=02, QUERY=03, FORWARD=10, BACKWARD=11, LEFT=12, RIGHT=13,
FASTER=14, SLOWER=15; SEARCH=01 is unavailable. Replies use TYPE=02
(00 accepted, 01 busy, 02 unavailable) and TYPE=03 state (00 idle, 04 stopped,
06 fault, 10 moving, 11 done), with the same six-byte framing/checksum.
Disabled voice rejects movement and gear commands; STOP and QUERY still work.

Menu and voice share one feedback/PI/PWM chain. Busy actions cannot be replaced
or queued. STOP wins conflicts; menu movement wins simultaneous voice movement.
Menu actions also publish motion state on ASRPRO TX. Three consecutive zero-count
10 ms windows are required before completion/restart. No USB heartbeat is needed.
IMU/encoder faults, stale feedback, 10-second action timeout, one-second failure
to stop, and ASR feedback FIFO overflow latch a fault until root reset.

Three ToF buses run continuously regardless of the displayed page. They only
supply distance/error display, never obstacle avoidance or motion interlocks.
Their example-derived driver reports communication/readiness validity, not
verified optical accuracy. A failed channel hides its old distance and retries
through XSHUT without blocking the other channels. See `PIN_ASSIGNMENT.md`.

LED D14=ready, C14=motion/settling, B9=latched fault, A9=voice enabled.
Encoder display uses signed 32-bit hexadecimal counts and signed decimal
counts/10 ms. Yaw is left-positive, matching the tested `YAW_SIGN=-1` motion
feedback; roll and pitch retain their sensor convention.
USB TX V8 carries event-only diagnostics: startup `S M 00`, `S N 00`, IMU start
or failure `S I xx`; IMU errors `E I xx`, encoder faults `E L 01`/`E R 01`,
and aggregate main fault `E X 01`. No heartbeat watchdog/startup event is emitted.
USB RX U8 is reserved and has no motion-control function.

## Board acceptance

1. Load the main `.fs`, keep the robot still through 200 IMU samples, and check
   the root menu, ready LED, voice OFF and stationary motors.
2. Check K1 wrap, K2 selection, content/group K0 return and root K0 calibration.
3. Check three ToF readings and move targets independently. Missing/failed
   sensors must show errors with hidden old distances; movement has no ToF guard.
4. First lift the wheels: enter each motor-test page, confirm no movement on
   entry, press K2 again, check direction, K0 immediate stop and no auto-resume.
5. On clear ground, check all four quantitative actions and record actual
   distance/angle. Wheel circumference/counts remain nominal and need calibration.
6. Enable voice in SETTINGS, return to other pages, test all commands, busy
   refusal, STOP and speed changes. Disable voice and verify movement is rejected.
7. Check root reset restores MID/voice OFF and requires calibration again.

Simulation/build evidence and limitations are in `records/main_20261023/VALIDATION.md`.
Simulation is not a measurement of physical distance, angle, braking or ToF accuracy.

## Standalone export

`./tools/export_main.ps1` creates `release/Embodied_RPACS_main_20261023/` with
only the main project, its own sensor model, scripts, docs and saved IDE options.
Build and simulate that directory directly; it has no `tests/` directory.

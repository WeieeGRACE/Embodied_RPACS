# Validation results

Primary and standalone bitstream SHA-256: 313a84c51ea92cd36ff7500363aeda7e68280c7b1671410c809f84395fe52247
Source manifest fingerprint: c1b3a24c029ee4e68f597fc1cd088bd7550b16840bff83e097ab4c430ca00807
Copied main-project sources verified: 29
Post-route Fmax: 50.418 MHz; setup violations: 0; hold violations: 0; setup/hold TNS: 0.000.
Resources: 15,604/23,040 LUT+ALU (68%), 5,763/23,040 logic FF (25%), 4/56 BSRAM, 9/28 DSP.

Primary project and isolated export passed complete motion/menu/ToF/voice/OLED integration, OLED bitmap and 50 MHz UART/SPI/PWM simulations. The standalone project has no tests directory or external test paths; its FS is byte-identical to the primary FS.

ASRPRO protocol regressions and all five ToF suites passed. IMU, OLED, speed, ground/closed-loop motor, motion-FSM and TB6612 regressions passed. The invalid-parameter test produced its expected assertion; ModelSim marks that expected-failure test as a command error.

All four quantitative menu actions and all eight 8-second debug actions passed in the plant simulation. Hardware acceptance remains pending: the FS has not been downloaded. Physical travel, angle repeatability, braking distance and ToF optical accuracy require board measurement.

Build: final_build.log, saved_project_build.log; standalone: standalone_final_build.log, standalone_saved_build.log.
Simulations: main_simulation.log, legacy_regression.log, standalone_sim.log.
Reports: final_report_check.log, standalone_report.log, standalone_sources.log.
OLED bitmap captures: oled_root.png, oled_voice.png, oled_tof.png.

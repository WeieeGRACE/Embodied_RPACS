# OV5647 camera board test

This is an independent board-test project based on the ACG525 teaching-board
example `ch60_mipi_ov5647_hdmi`. It is kept separate from the robot motor/IMU
top levels. The copied HDL retains the example's source headers and vendor IP
wrappers; the project target is `GW5A-LV25UG324C2/I1` (`gw5a25a-008`).

## Simulation first

Run from `sim/` with ModelSim SE 10.7:

```text
vsim -c -l ov5647_camera.log -do run_ov5647_camera.do
```

The regression checks the 58-entry OV5647 SCCB table and 232 ACK bits, CSI-2
RAW-8 (`DT=0x2A`) frame/line markers, ECC rejection, truncated-packet recovery,
byte-to-pixel ordering, Bayer output, DDR3 application-side write/read
correspondence, 1920x1080 active timing within 2200x1125 totals, and
complementary active TMDS contract outputs.

The simulator cannot decrypt the Gowin 2.3 protected CSI-2, DPHY, frame-buffer,
and DVI wrappers distributed with the example. `sim/ov5647_ip_sim_models.sv`
therefore provides documented-port functional adapters for the protocol-level
regression. Gowin primitive simulation library compilation is still exercised;
MIPI electrical behavior, DDR3 pin training, and HDMI signal integrity remain
hardware-only checks.

## Build the board image

Run from the repository root:

```text
gw_sh tests/ov5647_camera/build_ov5647_camera.tcl
```

The script uses Gowin IDE 1.9.12 and writes the generated implementation
products under `tests/ov5647_camera/impl/`, including
`ov5647_camera_test.fs`. Do not use the ES-device `.fs` shipped beside the
original example for this project's C2/I1 device.

The camera test uses the project target `GW5A-LV25UG324C2/I1` and its reset
pin B16 in the DDR bank, configured for its 1.5 V I/O rail. C15 is the
project emergency-stop input. Optional I2C debug mirrors
on C17/C18 were removed because those pins are assigned to encoder inputs in
the robot project. The ACG525 camera, DDR3 and HDMI routes remain unchanged.


On C2/I1 the HDMI data pair R5/T5 shares CPU/SSPI multiplexing. The build
script enables both regular-I/O options before placement, so the third TMDS lane
remains on the board-routed pair.

## Validation status (2026-10-17)

ModelSim regression returned 0 with 0 errors and 0 warnings. The protected
IP is replaced by functional models; this is not an end-to-end simulation of
the shipping encrypted IP or the board top level.

Gowin 1.9.12 synthesis, placement, routing and bitstream generation returned 0
for GW5A-LV25UG324C2/I1. HDMI R5/T5 is placed as LVDS25 with CPU and SSPI
regular-I/O options enabled. Build with the Tcl script: the GUI project lists
sources but does not store these Tcl options.

The HDMI PLL was changed from VCO 1500 MHz / 2 to 750 MHz / 1, retaining
750 MHz serial and 150 MHz pixel clocks while removing the VCO range warning.
The CSI byte-clock constraint now matches the 74.25 MHz PLL input setting.
The inherited video mode therefore retains approximately 60.606 Hz frame rate
(150 MHz / 2200 / 1125), rather than exact 60 Hz.

Static timing is not closed: the latest report has 1 setup-violated endpoint
and 22 hold-violated endpoints. Three TA1132 generated-clock warnings remain.
Use this image only for preliminary board testing; successful bitstream
generation and functional-model simulation do not establish reliable DDR/HDMI
operation across device corners. See VALIDATION.md for the test record.
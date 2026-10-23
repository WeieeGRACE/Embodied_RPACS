# Third-party notices

The HDL under `src/` was copied from the ACG525 teaching-board distribution,
example `ch60_mipi_ov5647_hdmi`, supplied with the board materials. Original
copyright and vendor headers are preserved in the copied files. Gowin-generated
IP wrappers remain protected vendor source and are used under the terms of the
Gowin IDE/IP distribution.

The copied camera table, SCCB controller, Bayer pipeline, CSI-2/DPHY,
DDR3-frame-buffer, PLL and DVI sources are included only to make this board
test independently buildable. They are not merged into the robot top level.

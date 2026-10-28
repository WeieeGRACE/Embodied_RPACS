# Reopen the saved standalone task-4 board project and rebuild using its saved
# device, top-level, constraints, source list and process settings.
cd [file normalize [file dirname [info script]]]
open_project task4_navigation_board.gprj
# These two GW5A dedicated-function pin groups are used as ordinary GPIO by
# the board pinout; set them before implementation, as in the initial build.
set_option -use_sspi_as_gpio 1
set_option -use_cpu_as_gpio 1
run all

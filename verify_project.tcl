# Build saved IDE settings without overriding top, clock, pin or device options.
cd [file normalize [file dirname [info script]]]
open_project Embodied_RPACS.gprj
run all

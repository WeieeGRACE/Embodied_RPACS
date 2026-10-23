# Verify persisted IDE project options without overriding them in this script.
cd [file normalize [file join [file dirname [info script]] ../..]]
open_project Embodied_RPACS.gprj
run all

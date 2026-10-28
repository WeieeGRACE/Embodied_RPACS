do sim/compile_system.do
vlog -sv -work system_lib sim/models/vl53l0x_model.sv sim/tb_system_obstacle.sv
vsim -onfinish stop -voptargs=+acc system_lib.tb_system_obstacle
when {/tb_system_obstacle/dut/control/dispatch/stop_event == '1'} {
  echo WHEN_STOP_EVENT
  examine -radix unsigned {/tb_system_obstacle/dut/control/dispatch/takeover} {/tb_system_obstacle/dut/control/dispatch/exit_cmd} {/tb_system_obstacle/dut/control/dispatch/pause_cmd} {/tb_system_obstacle/dut/control/dispatch/busy} {/tb_system_obstacle/dut/control/dispatch/stop_gate} {/tb_system_obstacle/dut/control/dispatch/stop_request} {/tb_system_obstacle/dut/control/menu_valid} {/tb_system_obstacle/dut/control/menu_command} {/tb_system_obstacle/dut/control/cv} {/tb_system_obstacle/dut/control/command} {/tb_system_obstacle/dut/control/avoid_mode} {/tb_system_obstacle/dut/control/auto_run}
}
when {/tb_system_obstacle/dut/control/dispatch/takeover == '1'} {
  echo WHEN_TAKEOVER
  examine -radix unsigned {/tb_system_obstacle/dut/control/command} {/tb_system_obstacle/dut/control/cv} {/tb_system_obstacle/dut/control/avoid_mode} {/tb_system_obstacle/dut/control/manual_active} {/tb_system_obstacle/dut/control/dispatch/manual_owned}
}
run 2200 ms
quit -f

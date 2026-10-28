if {![file exists work]} {vlib work}
vlog -sv src/nav_pose_range_guard.v src/nav_route_supervisor.v src/nav_segment_table.v src/nav_tof_wall_event.v src/nav_map_wall_association.v src/nav_event_segment_controller.v src/task4_navigation_top.v src/task4_event_navigation_top.v sim/tb_nav_pose_range_guard.v sim/tb_nav_route_supervisor.v sim/tb_nav_event_segment.v sim/tb_nav_map_wall_association.v sim/tb_nav_closed_loop.v
quit -f

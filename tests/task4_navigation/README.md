# 任务四事件分段导航测试工程

独立导航 RTL、真实轮速闭环仿真、碰撞预测和 Gowin 板级工程均位于本目录；不修改主工程或旧避障工程。

运行仿真与几何检查：

```powershell
.\run_sim.ps1
```

执行板级实现与保存工程重开复验：

```powershell
& 'D:\Gowin\Gowin_V1.9.12_x64\IDE\bin\gw_sh.exe' build_board.tcl
& 'D:\Gowin\Gowin_V1.9.12_x64\IDE\bin\gw_sh.exe' verify_board_saved_project.tcl
```

最新板级输出是 `impl/pnr/task4_navigation_board.fs`，Fmax 57.707 MHz，最差 setup slack 2.671 ns，50 MHz setup/hold 违例为 0。它是 A→OPEN_WAYPOINT_1 的 50 mm/s 单段探针，不会自动烧录。上板步骤见 [FIELD_TEST_GUIDE.md](FIELD_TEST_GUIDE.md)，仿真、碰撞预测及完整限制见 [IMPLEMENTATION_REPORT.md](IMPLEMENTATION_REPORT.md)。

完整 A→B、A→C、A→D、A→E 和巡检路线仍未通过 ±50 mm 墙端保守包络审计，禁止使用单段探针位流执行这些路线。


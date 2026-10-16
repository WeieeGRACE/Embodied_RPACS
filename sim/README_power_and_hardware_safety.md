# 独立电源与硬件安全模块仿真

`power_and_hardware_safety.v` 不实例化工程内其他模块。`external_fault` 是上层故障汇总入口，模块只执行安全门控、启动等待、故障锁存和 PWM 限幅。

在本目录运行：

```powershell
vsim -c -l simulation_power_and_hardware_safety.log -do run_power_and_hardware_safety.do
```

成功退出码为 0。交互波形使用：

```tcl
do wave_power_and_hardware_safety.do
```

故障优先级为：急停、过流、欠压、过温、外部故障。testbench 覆盖复位、稳定启动、PWM 正负限幅、急停、故障锁存、优先级、清除后重新启动以及禁用输出。

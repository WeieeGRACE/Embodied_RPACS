# 安全保护与故障仲裁仿真

在本目录执行：

```powershell
vsim -c -l simulation_safety_protection_and_fault_arbitration.log -do run_safety_protection_and_fault_arbitration.do
```

模块独立完成故障优先级、锁存、清除后重新启动、测距减速/停车和 PWM 限幅。故障优先级为急停、过流、欠压、过温、看门狗、传感器故障、外部故障。`RANGE_CHECK_ENABLE=0` 时可关闭测距安全输入。

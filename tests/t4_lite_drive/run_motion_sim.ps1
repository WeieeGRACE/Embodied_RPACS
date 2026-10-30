# 一键跑完电机控制相关的全部仿真（不需要板子）。
#
#   .\run_motion_sim.ps1
#
# 四个 testbench：
#   tb_eth_motion_bridge  基础 15 项（原有）
#   tb_eth_motion_unit    单元级 24 项（占空比扫描/周期/方向/死区/限幅/真值表/安全）
#   tb_eth_motion_system  端到端 27 项（协议帧 -> 桥接 -> 电机输出，含 STOP/掉链/急停）
#   tb_wheel_speed_loop   速度闭环 13 项（真实 125 MHz/20 kHz 时间基准：50/100/300 mm/s
#                         跟踪、±50 差速原地转、后退、SET_VEL(0,0)、使能关断）
param(
    [string]$Vsim = 'D:\app\modesilm\win64\vsim.exe'
)
$ErrorActionPreference = 'Stop'
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $here
New-Item -ItemType Directory -Force records | Out-Null

$suites = @(
    @{ Name = 'bridge';    Do = 'sim/run_motion.do';             Log = 'records/tb_bridge.log' },
    @{ Name = 'unit';      Do = 'sim/run_motion_unit.do';        Log = 'records/tb_unit.log' },
    @{ Name = 'system';    Do = 'sim/run_motion_system.do';      Log = 'records/tb_system.log' },
    @{ Name = 'speedloop'; Do = 'sim/run_wheel_speed_loop.do';   Log = 'records/tb_wheel_speed_loop.log' }
)

$results = @()
foreach ($s in $suites) {
    Write-Host "==> running $($s.Name)" -ForegroundColor Cyan
    $out = & $Vsim -c -do "do $($s.Do)" 2>&1
    $out | Set-Content -LiteralPath $s.Log -Encoding utf8
    $pass = ($out | Select-String -Pattern 'PASS tb_').Count -gt 0
    $fail = ($out | Select-String -Pattern 'FAIL tb_').Count -gt 0
    $checks = ($out | Select-String -Pattern 'PASS tb_.*\((\d+) checks\)').Matches
    $results += [pscustomobject]@{
        Suite = $s.Name
        Status = if ($pass -and -not $fail) { 'PASS' } else { 'FAIL' }
        Log = $s.Log
    }
    Write-Host ("    {0} ({1})" -f $results[-1].Status, $s.Log)
}

Write-Host ""
$results | Format-Table -AutoSize | Out-String | Write-Host

$allPass = ($results | Where-Object { $_.Status -ne 'PASS' }).Count -eq 0
if ($allPass) {
    Write-Host "All motion simulation suites passed." -ForegroundColor Green
    exit 0
} else {
    Write-Host "Some motion simulation suites FAILED." -ForegroundColor Red
    exit 1
}

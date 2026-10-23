param([string]$Vsim = 'D:/app/modesilm/win64/vsim.exe')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
Push-Location $projectRoot
try {
    foreach ($suite in @('imu_attitude','imu_attitude_oled','speed_prefetch')) {
        & $Vsim -c -l "records/main_20261023/regression_$suite.log" -do "sim/run_$suite.do"
        if ($LASTEXITCODE -ne 0) { throw "Regression failed: $suite" }
    }
    Push-Location sim
    try {
        foreach ($suite in @('ground_tests','closed_loop_test','dual_closed_loop_test','right_closed_loop_test','motion','tb6612')) {
            & $Vsim -c -l "../records/main_20261023/regression_$suite.log" -do "run_$suite.do"
            if ($LASTEXITCODE -ne 0) { throw "Regression failed: $suite" }
        }
    } finally { Pop-Location }
    Push-Location sim
    try {
        & $Vsim -c -l ../records/main_20261023/regression_motion_invalid.log -do run_motion_invalid.do
        if ($LASTEXITCODE -eq 0 -or -not (Select-String -LiteralPath ../records/main_20261023/regression_motion_invalid.log -Pattern 'invalid motion FSM parameters' -Quiet)) { throw 'Expected invalid motion parameter rejection missing' }
    } finally { Pop-Location }
    Write-Output 'PASS existing IMU/OLED/speed/motor/motion regressions'
} finally { Pop-Location }

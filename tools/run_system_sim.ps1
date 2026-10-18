param([string]$Vsim = 'vsim')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
Push-Location $projectRoot
try {
    & $Vsim -c -l system_tests.log -do sim/run_system.do
    if ($LASTEXITCODE -ne 0) { throw 'Main integration/IMU/logger simulation failed' }
    Push-Location (Join-Path $projectRoot 'sim')
    try {
        foreach ($suite in @('ground_tests','closed_loop_test','dual_closed_loop_test','right_closed_loop_test')) {
            & $Vsim -c -l "system_${suite}_regression.log" -do "run_${suite}.do"
            if ($LASTEXITCODE -ne 0) { throw "Regression failed: $suite" }
        }
    } finally { Pop-Location }
    Write-Output 'PASS all system and legacy simulation suites'
} finally { Pop-Location }

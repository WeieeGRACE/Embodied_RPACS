param([string]$ModelSim = 'D:/app/modesilm/win64/vsim.exe')
$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    New-Item -ItemType Directory -Force records | Out-Null
    foreach ($suite in @('edges', 'bus', 'three', 'oled', 'timing')) {
        & $ModelSim -c -do "do sim/run_$suite.do" *> "records/$suite.log"
        if ($LASTEXITCODE -ne 0) { throw "$suite failed; see records/$suite.log" }
        if (-not (Select-String -Path "records/$suite.log" -Pattern 'PASS' -Quiet)) {
            throw "$suite did not report PASS"
        }
        Write-Host "PASS $suite"
    }
} finally { Pop-Location }

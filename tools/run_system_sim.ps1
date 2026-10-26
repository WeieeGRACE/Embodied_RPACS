param([string]$Vsim = 'D:/app/modesilm/win64/vsim.exe')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
Push-Location $projectRoot
try {
    New-Item -ItemType Directory -Force records/obstacle_20261025 | Out-Null
    foreach ($suite in @('system','system_oled','system_50mhz','system_dispatch','live_speed','system_obstacle','attitude_pipeline')) {
        & $Vsim -c -l "records/obstacle_20261025/$suite.log" -do "sim/run_$suite.do"
        if ($LASTEXITCODE -ne 0) { throw "Main simulation failed: $suite" }
        $log = Get-Content -LiteralPath "records/obstacle_20261025/$suite.log" -Raw
        if (!$log.TrimEnd().EndsWith('Errors: 0, Warnings: 0') -or $log -match '\*\* (Fatal|Error)') { throw "Missing success footer: $suite" }
    }
    Write-Output 'PASS all seven integration, OLED, 50MHz, dispatcher, live speed, obstacle and attitude-pipeline suites'
} finally { Pop-Location }

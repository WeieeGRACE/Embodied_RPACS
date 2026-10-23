param([string]$Vsim = 'D:/app/modesilm/win64/vsim.exe')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
Push-Location $projectRoot
try {
    New-Item -ItemType Directory -Force records/main_20261023 | Out-Null
    foreach ($suite in @('system','system_oled','system_50mhz')) {
        & $Vsim -c -l "records/main_20261023/$suite.log" -do "sim/run_$suite.do"
        if ($LASTEXITCODE -ne 0) { throw "Main simulation failed: $suite" }
        if (-not (Select-String -LiteralPath "records/main_20261023/$suite.log" -Pattern 'Errors: 0, Warnings: 0' -Quiet)) { throw "Missing success footer: $suite" }
    }
    Write-Output 'PASS independent main integration, OLED bitmap and 50MHz suites'
} finally { Pop-Location }

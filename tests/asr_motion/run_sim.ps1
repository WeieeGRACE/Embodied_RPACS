param([string]$Vsim='D:/app/modesilm/win64/vsim.exe')
$ErrorActionPreference='Stop'
Push-Location $PSScriptRoot
try {
    foreach ($suite in @('units','integration','50mhz')) {
        $log = if ($suite -eq '50mhz') { 'clock_50mhz' } else { $suite }
        & $Vsim -c -l "records/$log.log" -do "sim/run_$suite.do"
        if ($LASTEXITCODE -ne 0) { throw "ASR motion suite failed: $suite" }
        if (-not (Select-String -Path "records/$log.log" -Pattern 'Errors: 0, Warnings: 0' -Quiet)) { throw "Missing simulator success report: $suite" }
    }
    & (Join-Path $PSScriptRoot 'serial_test.ps1') -SelfTest
    if ($LASTEXITCODE -ne 0) { throw 'Serial utility self-test failed' }
    Write-Output 'PASS all ASR motion simulation suites and offline serial tool checks'
} finally { Pop-Location }

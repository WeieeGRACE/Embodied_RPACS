param(
    [string]$ModelSim = 'D:/app/modesilm/win64/vsim.exe'
)
# Run every ModelSim suite of the uart_bridge project and fail loudly on any FAIL.
$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    New-Item -ItemType Directory -Force records | Out-Null
    $suites = @(
        @{ name = 'compile';  do = 'sim/run_compile.do' },
        @{ name = 'core';     do = 'sim/run_core.do' },
        @{ name = 'loopback'; do = 'sim/run_loopback.do' },
        @{ name = 'adapter';  do = 'sim/run_adapter.do' }
    )
    foreach ($s in $suites) {
        $log = "records/tb_$($s.name).log"
        & $ModelSim -c -do "do $($s.do)" *> $log
        if ($LASTEXITCODE -ne 0) { throw "$($s.name) exit code $LASTEXITCODE; see $log" }
        $text = Get-Content -LiteralPath $log -Raw
        if ($text -match 'FAIL ') { throw "$($s.name) reported FAIL; see $log" }
        if ($s.name -ne 'compile' -and -not ($text -match 'PASS tb_')) {
            throw "$($s.name) missing PASS marker; see $log"
        }
        Write-Host "PASS $($s.name)  -> $log"
    }
    Write-Output 'All uart_bridge simulation suites passed.'
} finally {
    Pop-Location
}

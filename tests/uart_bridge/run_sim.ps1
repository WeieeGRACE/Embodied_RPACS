param(
    [string]$ModelSim = 'D:/app/modesilm/win64/vsim.exe',
    [string]$Python   = 'python'
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
        @{ name = 'adapter';  do = 'sim/run_adapter.do' },
        @{ name = 'system';   do = 'sim/run_system.do' }
    )
    foreach ($s in $suites) {
        $log = "records/tb_$($s.name).log"
        & $ModelSim -c -do "do $($s.do)" 2>&1 | Out-File -LiteralPath $log -Encoding utf8
        if ($LASTEXITCODE -ne 0) { throw "$($s.name) exit code $LASTEXITCODE; see $log" }
        $text = Get-Content -LiteralPath $log -Raw
        if ($text -match 'FAIL ') { throw "$($s.name) reported FAIL; see $log" }
        if ($s.name -ne 'compile' -and -not ($text -match 'PASS tb_')) {
            throw "$($s.name) missing PASS marker; see $log"
        }
        Write-Host "PASS $($s.name)  -> $log"
    }

    # Python <-> RTL conformance on a shared adversarial corpus (optional: needs python)
    $py = Get-Command $Python -ErrorAction SilentlyContinue
    if ($py) {
        & $py.Source tools/conformance.py --run --vsim $ModelSim 2>&1 | Out-File -LiteralPath records/conformance.log -Encoding utf8
        if ($LASTEXITCODE -ne 0) { throw "conformance FAIL; see records/conformance.log" }
        Write-Host "PASS conformance -> records/conformance.log"
    } else {
        Write-Host "SKIP conformance (python not found; pass -Python <path>)"
    }

    Write-Output 'All uart_bridge simulation suites passed.'
} finally {
    Pop-Location
}

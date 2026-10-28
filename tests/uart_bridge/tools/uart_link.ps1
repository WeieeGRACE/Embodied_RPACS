param(
    [string]$Port = 'COM14',
    [int]$Baud = 115200,
    [ValidateSet('monitor', 'heartbeat', 'bench', 'selftest', 'ports')]
    [string]$Action = 'monitor',
    [string]$Python = 'C:\Users\Administartor\.dsh\dsh-runtimes\dsh-primary-runtime\dependencies\python\python.exe'
)
# PowerShell front-end for uart_link.py (same tool, same protocol).
#
#   .\uart_link.ps1 -Action ports
#   .\uart_link.ps1 -Action monitor -Port COM14
#   .\uart_link.ps1 -Action heartbeat -Port COM14
#   .\uart_link.ps1 -Action bench -Port COM14
#
# Close Fluent Serial Assistant before running: one owner per COM port.
$ErrorActionPreference = 'Stop'
$tool = Join-Path $PSScriptRoot 'tools\uart_link.py'
if (!(Test-Path $tool)) { throw "missing $tool" }
if (!(Test-Path $Python)) {
    $Python = (Get-Command python -ErrorAction Stop).Source
}
$env:PYTHONIOENCODING = 'utf-8'
$common = @('--port', $Port, '--baud', "$Baud")
switch ($Action) {
    'ports'     { & $Python $tool ports }
    'selftest'  { & $Python $tool selftest }
    'monitor'   { & $Python $tool monitor @common }
    'heartbeat' { & $Python $tool send @common heartbeat --count 20 --rate 10 }
    'bench'     {
        $out = Join-Path $PSScriptRoot ("records/bench_" + (Get-Date -Format 'yyyyMMdd_HHmmss') + ".json")
        & $Python $tool bench @common --out $out
        Write-Host "report: $out"
    }
}
exit $LASTEXITCODE

param(
    [string]$Port = 'COM14',
    [int]$Baud = 115200,
    [ValidateSet('monitor', 'heartbeat', 'bench', 'selftest', 'ports')]
    [string]$Action = 'monitor',
    [string]$Python = 'python'   # 可传 -Python 指定解释器；默认走 PATH
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
$tool     = Join-Path $PSScriptRoot 'uart_link.py'
$recDir   = Join-Path (Join-Path $PSScriptRoot '..') 'records'
if (!(Test-Path $tool)) { throw "missing $tool" }
if (!(Test-Path $Python)) {
    $pyCmd = Get-Command python -ErrorAction SilentlyContinue
    if (!$pyCmd) { throw "python not found: pass -Python <path to python.exe>" }
    $Python = $pyCmd.Source
}
$env:PYTHONIOENCODING = 'utf-8'
$common = @('--port', $Port, '--baud', "$Baud")
switch ($Action) {
    'ports'     { & $Python $tool ports }
    'selftest'  { & $Python $tool selftest }
    'monitor'   { & $Python $tool monitor @common }
    'heartbeat' { & $Python $tool send @common heartbeat --count 20 --rate 10 }
    'bench'     {
        New-Item -ItemType Directory -Force $recDir | Out-Null
        $out = Join-Path $recDir ("bench_" + (Get-Date -Format 'yyyyMMdd_HHmmss') + ".json")
        & $Python $tool bench @common --out $out
        Write-Host "report: $out"
    }
}
exit $LASTEXITCODE

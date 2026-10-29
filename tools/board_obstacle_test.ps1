# Main-project board check helper: optional SRAM programming + read-only USB
# event-log capture + automatic pass/fail summary.
#
# The FPGA USB link is FPGA->PC diagnostics only (115200 8N1): this script never
# writes to the port, so it cannot disturb the ASRPRO motion UART.
#
# Typical use (user runs the obstacle demo while the capture is running):
#   ./tools/board_obstacle_test.ps1 -Port COM14 -Seconds 180
#   ./tools/board_obstacle_test.ps1 -Port COM14 -Seconds 180 -Program
#
# Exit code 0 only when at least MinHeartbeats "D " debug/heartbeat lines were
# captured and no "E " error/fault line appeared.
param(
    [Parameter(Mandatory = $true)][string]$Port,
    [int]$Seconds = 120,
    [switch]$Program,
    [string]$FsFile = 'impl/pnr/Embodied_RPACS.fs',
    [string]$Device = 'GW5A-25A',
    [int]$MinHeartbeats = 5,
    [string]$OutDir = 'records/robustness_20261029'
)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
Push-Location $projectRoot
try {
    New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
    $stamp = (Get-Date).ToString('yyyyMMdd_HHmmss')

    if ($Program) {
        if (!(Test-Path -LiteralPath $FsFile)) { throw "Bitstream not found: $FsFile" }
        $programmer = 'D:/Gowin/Gowin_V1.9.12_x64/Programmer/bin/programmer_cli.exe'
        if (!(Test-Path -LiteralPath $programmer)) { throw "programmer_cli.exe not found: $programmer" }
        $programLog = Join-Path $OutDir "board_program_$stamp.log"
        $hash = (Get-FileHash -LiteralPath $FsFile -Algorithm SHA256).Hash
        Write-Host "Programming SRAM with $FsFile"
        Write-Host "  SHA256 $hash"
        & $programmer --device $Device --operation_index 2 --fsFile $FsFile *> $programLog
        if ($LASTEXITCODE -ne 0) { throw "SRAM programming failed, see $programLog" }
        $programText = Get-Content -LiteralPath $programLog -Raw
        if ($programText -notmatch 'Program|Succeed|success|完成') {
            Write-Warning "programmer output did not report success; inspect $programLog"
        }
        Write-Host "  SRAM program OK -> $programLog"
        Start-Sleep -Milliseconds 800
    }

    $capture = Join-Path $OutDir "board_capture_$stamp.log"
    & (Join-Path $PSScriptRoot 'read_serial_log.ps1') -Port $Port -Seconds $Seconds -OutFile $capture
    if (!(Test-Path -LiteralPath $capture)) { throw "Capture file missing: $capture" }

    $lines = @(Get-Content -LiteralPath $capture)
    $heartbeats = @($lines | Where-Object { $_ -match '^D ' }).Count
    $errors = @($lines | Where-Object { $_ -match '^E ' })
    $boots = @($lines | Where-Object { $_ -match '^S ' })
    $summary = Join-Path $OutDir "board_check_$stamp.md"
    @(
        "# Board capture $stamp",
        '',
        "- port: $Port",
        "- seconds: $Seconds",
        "- programmed: $([bool]$Program)",
        "- bitstream: $FsFile",
        "- capture: $capture",
        "- heartbeats/debug lines: $heartbeats",
        "- startup lines: $($boots.Count)",
        "- error/fault lines: $($errors.Count)",
        '',
        '## Error lines',
        ''
    ) + @($errors | ForEach-Object { "- ``$_``" }) | Set-Content -LiteralPath $summary -Encoding UTF8

    Write-Host "Captured $($lines.Count) lines -> $capture"
    Write-Host "  heartbeats/debug=$heartbeats startup=$($boots.Count) errors=$($errors.Count)"
    if ($errors.Count) {
        Write-Host '  ERROR/FAULT lines:' -ForegroundColor Red
        $errors | ForEach-Object { Write-Host "    $_" -ForegroundColor Red }
    }
    if ($heartbeats -lt $MinHeartbeats) { throw "Only $heartbeats heartbeat lines (need >= $MinHeartbeats): USB link or FPGA boot problem" }
    if ($errors.Count) { throw "$($errors.Count) error/fault line(s) captured; see $summary" }
    Write-Host "PASS board capture: $heartbeats heartbeats, no error/fault lines (summary $summary)"
}
finally { Pop-Location }

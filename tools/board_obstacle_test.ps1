# Main-project board check helper: optional SRAM programming + read-only USB
# event-log capture + automatic pass/fail + route/PAUSED summary.
#
# The FPGA USB link is FPGA->PC diagnostics only (115200 8N1): this script never
# writes to the port, so it cannot disturb the ASRPRO motion UART.
#
# Typical use (user runs the obstacle demo while the capture is running):
#   ./tools/board_obstacle_test.ps1 -Seconds 180
#   ./tools/board_obstacle_test.ps1 -Seconds 180 -Program
#   ./tools/board_obstacle_test.ps1 -Port COM23 -Seconds 180 -FailOnPause
#
# -Port defaults to 'auto': the CH9102/FTDI debug adapter is detected
# automatically (the COM number changes when the USB cable is re-plugged).
#
# Exit code 0 only when at least MinHeartbeats "D " debug/heartbeat lines were
# captured and no "E " error/fault line appeared (and, with -FailOnPause, no
# avoid_mode pause was seen).
param(
    [string]$Port = 'auto',
    [int]$Seconds = 120,
    [switch]$Program,
    [switch]$FailOnPause,
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

    if ($Port -eq 'auto') {
        $candidates = @(Get-CimInstance Win32_PnPEntity -ErrorAction SilentlyContinue |
            Where-Object { $_.Name -match '\(COM\d+\)' -and $_.Name -notmatch '蓝牙|Bluetooth' })
        $pick = $candidates | Where-Object { $_.Name -match 'CH9102|CH34|FTDI|USB-SERIAL|USB-Enhanced|WCH-Link' } | Select-Object -First 1
        if (!$pick) { $pick = $candidates | Select-Object -First 1 }
        if (!$pick) { throw 'No USB serial debug port found; plug the FPGA debug USB cable and retry' }
        $null = $pick.Name -match '\(COM(\d+)\)'
        $Port = 'COM' + $Matches[1]
        Write-Host "Auto-detected debug serial port: $Port ($($pick.Name))"
    }

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
        if ($programText -notmatch 'Finished|Succeed|success|Status Code') {
            Write-Warning "programmer output did not report success; inspect $programLog"
        }
        Write-Host "  SRAM program OK -> $programLog"
        Start-Sleep -Milliseconds 800
    }

    $capture = Join-Path $OutDir "board_capture_$stamp.log"
    & (Join-Path $PSScriptRoot 'read_serial_log.ps1') -Port $Port -Seconds $Seconds -OutFile $capture -Timestamped

    $lines = @(Get-Content -LiteralPath $capture)
    $heartbeats = 0
    $errors = New-Object System.Collections.Generic.List[string]
    $starts = 0
    $states = @{}
    $pauses = New-Object System.Collections.Generic.List[string]
    $faults = New-Object System.Collections.Generic.List[string]
    $lastAuto = -1
    foreach ($raw in $lines) {
        $line = $raw.Trim()
        if ($line -match '^\[(\d\d:\d\d:\d\d\.\d\d\d)\]\s+D\s+([0-9A-Fa-f]{16})') {
            $t = $Matches[1]
            $v = [Convert]::ToUInt64($Matches[2], 16)
            $reason = ($v -shr 48) -band 0xFF
            $reused = ($v -shr 13) -band 0x7
            $state = ($v -shr 8) -band 0x1F
            $flags = $v -band 0xFF
            $fault = if ($flags -band 0x10) { 1 } else { 0 }
            $auto = if ($flags -band 0x02) { 1 } else { 0 }
            $avoid = if ($flags -band 0x01) { 1 } else { 0 }
            $heartbeats++
            if ($states.ContainsKey($state)) { $states[$state]++ } else { $states[$state] = 1 }
            if ($fault -eq 1) { $faults.Add("$t state=$state reason=0x{0:X2} reused=$reused" -f $reason) }
            if ($lastAuto -eq 1 -and $auto -eq 0 -and $avoid -eq 1 -and $fault -eq 0) {
                $pauses.Add("$t state=$state reason=0x{0:X2} reused=$reused (PAUSED, K2 resumes)" -f $reason)
            }
            $lastAuto = $auto
            continue
        }
        if ($line -match '\s+E\s+(\S+)\s+(\S+)') { $errors.Add($line); continue }
        if ($line -match '\s+S\s+(\S+)\s+(\S+)') { $starts++; continue }
    }

    $summary = Join-Path $OutDir "board_check_$stamp.md"
    @(
        "# Board capture $stamp",
        '',
        "- port: $Port",
        "- seconds: $Seconds",
        "- programmed: $([bool]$Program)",
        "- bitstream: $FsFile",
        "- capture: $capture",
        "- debug lines: $heartbeats",
        "- startup lines: $starts",
        "- error/fault lines: $($errors.Count)",
        "- fault (fault bit) snapshots: $($faults.Count)",
        "- avoid-mode pauses: $($pauses.Count)",
        "- route state histogram: " + (($states.GetEnumerator() | Sort-Object Name | ForEach-Object { "s$($_.Name)x$($_.Value)" }) -join ' '),
        '',
        '## Error lines', ''
    ) + @($errors | ForEach-Object { "- ``$_``" }) + @(
        '',
        '## FAULT snapshots', ''
    ) + @($faults | ForEach-Object { "- $_" }) + @(
        '',
        '## PAUSED transitions', ''
    ) + @($pauses | ForEach-Object { "- $_" }) | Set-Content -LiteralPath $summary -Encoding UTF8

    Write-Host "Captured $($lines.Count) lines -> $capture"
    Write-Host "  debug=$heartbeats startup=$starts errors=$($errors.Count) faults=$($faults.Count) pauses=$($pauses.Count)"
    if ($faults.Count) { Write-Host '  FAULT snapshots:' -ForegroundColor Red; $faults | ForEach-Object { Write-Host "    $_" -ForegroundColor Red } }
    if ($pauses.Count) { Write-Host '  PAUSED transitions:' -ForegroundColor Yellow; $pauses | ForEach-Object { Write-Host "    $_" -ForegroundColor Yellow } }
    if ($errors.Count) {
        Write-Host '  ERROR/FAULT lines:' -ForegroundColor Red
        $errors | ForEach-Object { Write-Host "    $_" -ForegroundColor Red }
    }
    if ($heartbeats -lt $MinHeartbeats) { throw "Only $heartbeats debug lines (need >= $MinHeartbeats): USB link or FPGA boot problem" }
    if ($errors.Count) { throw "$($errors.Count) error/fault line(s) captured; see $summary" }
    if ($FailOnPause -and $pauses.Count) { throw "$($pauses.Count) PAUSED transition(s) captured; see $summary" }
    Write-Host "PASS board capture: $heartbeats debug lines, no error/fault lines (summary $summary)"
}
finally { Pop-Location }

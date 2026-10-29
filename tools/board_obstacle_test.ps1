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
#   ./tools/board_obstacle_test.ps1 -Port COM24 -Program `
#       -FsFile tests/p6_uart_debug/impl/pnr/Embodied_RPACS_p6uart.fs -Seconds 300 -FailOnPause
#   ./tools/board_obstacle_test.ps1 -AnalyzeOnly -Capture records/.../log
#
# -Port defaults to 'auto': CH9102/FTDI/WCH-Link debug adapters are detected
# automatically (the COM number changes when a USB cable is re-plugged).
#
# Exit code 0 only when at least MinHeartbeats "D " debug/heartbeat lines were
# captured and no "E " error/fault line appeared (and, with -FailOnPause, no
# avoid_mode pause was seen).
param(
    [string]$Port = 'auto',
    [int]$Seconds = 120,
    [switch]$Program,
    [switch]$FailOnPause,
    [switch]$AnalyzeOnly,
    [string]$Capture = '',
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

    if ($AnalyzeOnly) {
        if (!$Capture -or !(Test-Path -LiteralPath $Capture)) { throw "AnalyzeOnly needs an existing -Capture log" }
    } else {
        if ($Port -eq 'auto') {
            $candidates = @(Get-CimInstance Win32_PnPEntity -ErrorAction SilentlyContinue |
                Where-Object { $_.Name -match '\(COM\d+\)' -and $_.Name -notmatch '蓝牙|Bluetooth' })
            $pick = $candidates | Where-Object { $_.Name -match 'CH9102|CH34|FTDI|USB-SERIAL|USB-Enhanced|WCH-Link' } | Select-Object -First 1
            if (!$pick) { $pick = $candidates | Select-Object -First 1 }
            if (!$pick) { throw 'No USB serial debug port found; plug the debug USB cable/WCH-Link and retry' }
            $null = $pick.Name -match '\(COM(\d+)\)'
            $Port = 'COM' + $Matches[1]
            Write-Host "Auto-detected debug serial port: $Port ($($pick.Name))"
        }

        if ($Program) {
            if (!(Test-Path -LiteralPath $FsFile)) { throw "Bitstream not found: $FsFile" }
            # programmer_cli resolves --fsFile against its own working directory,
            # so always pass an absolute path.
            $fsPath = (Resolve-Path -LiteralPath $FsFile).Path
            $programmer = 'D:/Gowin/Gowin_V1.9.12_x64/Programmer/bin/programmer_cli.exe'
            if (!(Test-Path -LiteralPath $programmer)) { throw "programmer_cli.exe not found: $programmer" }
            $programLog = Join-Path $OutDir "board_program_$stamp.log"
            $hash = (Get-FileHash -LiteralPath $fsPath -Algorithm SHA256).Hash
            Write-Host "Programming SRAM with $fsPath"
            Write-Host "  SHA256 $hash"
            & $programmer --device $Device --operation_index 2 --fsFile "$fsPath" *> $programLog
            if ($LASTEXITCODE -ne 0) { throw "SRAM programming failed (Gowin downloader connected?), see $programLog" }
            $programText = Get-Content -LiteralPath $programLog -Raw
            if ($programText -notmatch 'Finished|Succeed|success|Status Code') {
                Write-Warning "programmer output did not report success; inspect $programLog"
            }
            Write-Host "  SRAM program OK -> $programLog"
            Start-Sleep -Milliseconds 800
        }

        $Capture = Join-Path $OutDir "board_capture_$stamp.log"
        & (Join-Path $PSScriptRoot 'read_serial_log.ps1') -Port $Port -Seconds $Seconds -OutFile $Capture -Timestamped
    }

    $lines = @(Get-Content -LiteralPath $Capture)
    $heartbeats = 0
    $errors = New-Object System.Collections.Generic.List[string]
    $starts = 0
    $states = @{}
    $pauses = New-Object System.Collections.Generic.List[string]
    $routePauses = New-Object System.Collections.Generic.List[string]
    $faults = New-Object System.Collections.Generic.List[string]
    $lastAuto = -1
    $prevReason = 0
    $prevState = 0
    foreach ($raw in $lines) {
        $line = $raw.Trim()
        $stampText = ''
        if ($line -match '^\[(\d\d:\d\d:\d\d\.\d\d\d)\]\s*') {
            $stampText = $Matches[1]
            $line = $line.Substring($Matches[0].Length)
        }
        if ($line -match '^D\s+([0-9A-Fa-f]{16})') {
            $v = [Convert]::ToUInt64($Matches[1], 16)
            $reason = ($v -shr 48) -band 0xFF
            $reused = ($v -shr 13) -band 0x7
            $state = ($v -shr 8) -band 0x1F
            $flags = $v -band 0xFF
            $fault = if ($flags -band 0x10) { 1 } else { 0 }
            $auto = if ($flags -band 0x02) { 1 } else { 0 }
            $avoid = if ($flags -band 0x01) { 1 } else { 0 }
            $heartbeats++
            if ($states.ContainsKey($state)) { $states[$state]++ } else { $states[$state] = 1 }
            if ($fault -eq 1) { $faults.Add("$stampText state=$state reason=0x{0:X2} reused=$reused" -f $reason) }
            if ($lastAuto -eq 1 -and $auto -eq 0 -and $avoid -eq 1 -and $fault -eq 0) {
                # A route pause comes from BLOCKED/FAULT or a route retry reason
                # (0x81..0x8F); anything else is the operator pressing K2.
                $isRoutePause = ($prevState -eq 17) -or ($prevState -eq 19) -or (($prevReason -band 0xF0) -eq 0x80)
                $msg = ("$stampText PAUSED after state={0} reason=0x{1:X2} reused={2}" -f $prevState, $prevReason, $reused)
                if ($isRoutePause) { $routePauses.Add($msg) } else { $pauses.Add($msg + ' (user K2 pause)') }
            }
            $lastAuto = $auto
            $prevReason = $reason
            $prevState = $state
            continue
        }
        if ($line -match '^E\s+(\S+)\s+(\S+)') { $errors.Add($raw.Trim()); continue }
        if ($line -match '^S\s+(\S+)\s+(\S+)') { $starts++; continue }
    }

    $summary = Join-Path $OutDir "board_check_$stamp.md"
    @(
        "# Board capture $stamp",
        '',
        "- capture: $Capture",
        "- analyzed only: $([bool]$AnalyzeOnly)",
        "- debug lines: $heartbeats",
        "- startup lines: $starts",
        "- error/fault lines: $($errors.Count)",
        "- fault (fault bit) snapshots: $($faults.Count)",
        "- route pauses (BLOCKED/retry exhausted): $($routePauses.Count)",
        "- user K2 pauses: $($pauses.Count)",
        "- route state histogram: " + (($states.GetEnumerator() | Sort-Object Name | ForEach-Object { "s$($_.Name)x$($_.Value)" }) -join ' '),
        '',
        '## Error lines', ''
    ) + @($errors | ForEach-Object { "- ``$_``" }) + @(
        '',
        '## FAULT snapshots', ''
    ) + @($faults | ForEach-Object { "- $_" }) + @(
        '',
        '## Route PAUSED transitions (demo failures)', ''
    ) + @($routePauses | ForEach-Object { "- $_" }) + @(
        '',
        '## User K2 pauses', ''
    ) + @($pauses | ForEach-Object { "- $_" }) | Set-Content -LiteralPath $summary -Encoding UTF8

    Write-Host "Analyzed $($lines.Count) lines from $Capture"
    Write-Host "  debug=$heartbeats startup=$starts errors=$($errors.Count) faults=$($faults.Count) route_pauses=$($routePauses.Count) user_pauses=$($pauses.Count)"
    if ($faults.Count) { Write-Host '  FAULT snapshots:' -ForegroundColor Red; $faults | ForEach-Object { Write-Host "    $_" -ForegroundColor Red } }
    if ($routePauses.Count) { Write-Host '  ROUTE PAUSED transitions:' -ForegroundColor Yellow; $routePauses | ForEach-Object { Write-Host "    $_" -ForegroundColor Yellow } }
    if ($pauses.Count) { Write-Host '  User K2 pauses:' -ForegroundColor DarkGray; $pauses | ForEach-Object { Write-Host "    $_" -ForegroundColor DarkGray } }
    if ($errors.Count) {
        Write-Host '  ERROR/FAULT lines:' -ForegroundColor Red
        $errors | ForEach-Object { Write-Host "    $_" -ForegroundColor Red }
    }
    if ($heartbeats -lt $MinHeartbeats) { throw "Only $heartbeats debug lines (need >= $MinHeartbeats): USB link or FPGA boot problem" }
    if ($errors.Count) { throw "$($errors.Count) error/fault line(s) captured; see $summary" }
    if ($FailOnPause -and $routePauses.Count) { throw "$($routePauses.Count) route PAUSED transition(s) captured; see $summary" }
    Write-Host "PASS board analysis: $heartbeats debug lines, no error/fault lines (summary $summary)"
}
finally { Pop-Location }

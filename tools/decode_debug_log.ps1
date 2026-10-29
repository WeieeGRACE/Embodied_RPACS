# Decode main-project USB event-log captures.
#
# Line formats produced by src/uart/system_event_logger.v:
#   S <source> <code>          startup/status event (e.g. "S I 00")
#   E <source> <code>          error/fault event
#   D <16 hex digits>          debug_state snapshot (changes + 1 Hz heartbeat)
#
# debug_state layout (system_motion.v):
#   [63:56] start_diag      [55:48] route_reason   [47:40] menu_command
#   [39:32] ack             [31:24] aux_type       [23:16] aux_code
#   [15:13] tof_reused      [12:8]  route_state
#   [7] voice_enable [6] modules_ready [5] tof_all_valid [4] fault
#   [3] busy [2] waiting [1] auto_run [0] avoid_mode
#
# Usage:
#   ./tools/decode_debug_log.ps1 -Path records/.../board_capture.log
#   ./tools/decode_debug_log.ps1 -Path ... -ChangesOnly
param(
    [Parameter(Mandatory = $true)][string]$Path,
    [switch]$ChangesOnly,
    [string]$OutFile = ''
)
$ErrorActionPreference = 'Stop'
if (!(Test-Path -LiteralPath $Path)) { throw "Log not found: $Path" }
$lines = @(Get-Content -LiteralPath $Path)
$out = New-Object System.Collections.Generic.List[string]
$lastSig = ''
$snapshots = 0
$errors = New-Object System.Collections.Generic.List[string]
$starts = New-Object System.Collections.Generic.List[string]
$routes = @{}
$lastState = -1
foreach ($line in $lines) {
    $line = $line.Trim()
    if (!$line) { continue }
    if ($line -match '^E\s') { $errors.Add($line); $out.Add("ERR  $line"); continue }
    if ($line -match '^S\s') { $starts.Add($line); $out.Add("BOOT $line"); continue }
    if ($line -notmatch '^D\s+([0-9A-Fa-f]{16})') { continue }
    $snapshots++
    $value = [Convert]::ToUInt64($Matches[1], 16)
    $startDiag = ($value -shr 56) -band 0xFF
    $reason = ($value -shr 48) -band 0xFF
    $menuCmd = ($value -shr 40) -band 0xFF
    $ack = ($value -shr 32) -band 0xFF
    $auxType = ($value -shr 24) -band 0xFF
    $auxCode = ($value -shr 16) -band 0xFF
    $reused = ($value -shr 13) -band 0x7
    $state = ($value -shr 8) -band 0x1F
    $flags = $value -band 0xFF
    $sig = "{0}|{1}|{2}|{3}|{4}|{5}|{6}" -f $state, $reason, $reused, $flags, $menuCmd, $startDiag, $ack
    if ($state -ne $lastState) {
        if ($lastState -ge 0) { $routes[$state] = 1 + ($routes[$state] -as [int]) }
        $lastState = $state
    }
    if ($ChangesOnly -and $sig -eq $lastSig) { continue }
    $lastSig = $sig
    $flagText = @()
    if ($flags -band 0x80) { $flagText += 'voice' }
    if ($flags -band 0x40) { $flagText += 'ready' }
    if ($flags -band 0x20) { $flagText += 'tof3' }
    if ($flags -band 0x10) { $flagText += 'FAULT' }
    if ($flags -band 0x08) { $flagText += 'busy' }
    if ($flags -band 0x04) { $flagText += 'wait' }
    if ($flags -band 0x02) { $flagText += 'auto' }
    if ($flags -band 0x01) { $flagText += 'avoid' }
    $out.Add(("D {0}  state={1,-2} reason=0x{2:X2} reused={3} {4} menu=0x{5:X2} diag={6} ack=0x{7:X2} {8}" -f
        $Matches[1], $state, $reason, $reused, (($flagText -join ',')), $menuCmd, $startDiag, $ack, $line))
}
$summary = @(
    '',
    "== decode summary: file=$Path snapshots=$snapshots errors=$($errors.Count) boot=$($starts.Count)",
    ('route_state histogram: ' + (($routes.GetEnumerator() | Sort-Object Name | ForEach-Object { "s$($_.Name)x$($_.Value)" }) -join ' '))
) + @($errors | ForEach-Object { "error line: $_" })
$out.AddRange([string[]]$summary)
$text = $out -join "`n"
if ($OutFile) { $text | Set-Content -LiteralPath $OutFile -Encoding UTF8 }
Write-Output $text

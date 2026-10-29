# One-off analysis of a timestamped board_run capture (debug_state lines only).
# Prints only field changes so the route timeline stays readable.
param(
    [Parameter(Mandatory = $true)][string]$Path,
    [int]$MaxEvents = 400
)
$ErrorActionPreference = 'Stop'
$last = ''
$events = 0
$reusedTotal = 0
$reusedSet = 0
$stateCount = @{}
foreach ($raw in Get-Content -LiteralPath $Path) {
    if ($raw -notmatch '^\[(\d\d:\d\d:\d\d\.\d\d\d)\]\s+D\s+([0-9A-Fa-f]{16})') { continue }
    $t = $Matches[1]
    $v = [Convert]::ToUInt64($Matches[2], 16)
    $startDiag = ($v -shr 56) -band 0xFF
    $reason = ($v -shr 48) -band 0xFF
    $menuCmd = ($v -shr 40) -band 0xFF
    $ack = ($v -shr 32) -band 0xFF
    $auxType = ($v -shr 24) -band 0xFF
    $auxCode = ($v -shr 16) -band 0xFF
    $reused = ($v -shr 13) -band 0x7
    $state = ($v -shr 8) -band 0x1F
    $flags = $v -band 0xFF
    $voice = if ($flags -band 0x80) { 1 } else { 0 }
    $ready = if ($flags -band 0x40) { 1 } else { 0 }
    $tof3 = if ($flags -band 0x20) { 1 } else { 0 }
    $fault = if ($flags -band 0x10) { 1 } else { 0 }
    $busy = if ($flags -band 0x08) { 1 } else { 0 }
    $wait = if ($flags -band 0x04) { 1 } else { 0 }
    $auto = if ($flags -band 0x02) { 1 } else { 0 }
    $avoid = if ($flags -band 0x01) { 1 } else { 0 }
    $sig = "$state|$reason|$reused|$auto|$avoid|$busy|$fault|$tof3|$startDiag|$menuCmd|$auxType|$auxCode"
    if ($state -eq 8 -or $state -eq 5 -or $state -eq 6 -or $state -eq 23 -or $state -eq 9 -or $state -eq 13 -or $state -eq 16 -or $state -eq 20) {
        $reusedTotal++
        if ($reused -ne 0) { $reusedSet++ }
    }
    if ($stateCount.ContainsKey($state)) { $stateCount[$state]++ } else { $stateCount[$state] = 1 }
    if ($sig -eq $last) { continue }
    $last = $sig
    $events++
    if ($events -le $MaxEvents) {
        Write-Output ("{0} state={1,-2} reason=0x{2:X2} reused={3} auto={4} avoid={5} busy={6} fault={7} tof3={8} diag={9} menu=0x{10:X2} aux={11:X2}/{12:X2}" -f `
            $t, $state, $reason, $reused, $auto, $avoid, $busy, $fault, $tof3, $startDiag, $menuCmd, $auxType, $auxCode)
    }
}
Write-Output ''
Write-Output ("== state histogram: " + (($stateCount.GetEnumerator() | Sort-Object Name | ForEach-Object { "s$($_.Name)x$($_.Value)" }) -join ' '))
if ($reusedTotal) { Write-Output ("== barrier/state samples with reused!=0: {0}/{1}" -f $reusedSet, $reusedTotal) }

# ModelSim regression for the post-90-degree-turn robustness fix.
#
# Runs the fault-injection testbench against the CURRENT src/ tree (fixed
# version, +FIXED=1 +NEWMENU=1) for the requested scenarios and asserts that
# every run reports fault_seen=0 / fault_latched=0, plus paused=0 /
# blocked_seen=0 in its FIXED_RUN_OBSERVED line (a paused route is the
# user-visible failure even when no fault latches). Scenario 10 additionally
# requires reached_shift=1: a permanently clipped obstacle-side lane must open
# the fixed-mileage fallback detour instead of stalling WAIT_PARALLEL.
# Scenario 7 (a real 60 mm reading <= AVOID_MM) requires near_shift=1: near
# readings stay valid and open the same fallback instead of blocking motion.
# Scenario 11 (clip appears only after PASS started, seen on the board as
# BLOCKED reason 0x04) requires pass_reached=1 and pass_clip_done=1: the pass
# must finish on fixed mileage and resume CRUISE.
# Scenario 12 (box corner grazes the front lane for a few samples while PASS
# drives past it, also seen on the board as BLOCKED reason 0x04) requires
# pass_reached=1 and pass_front_done=1: the leg front threshold is debounced by
# FRONT_LEG_MS, so a short graze must not abort the pass.
#
# Usage:
#   ./tools/run_fault_injection.ps1                 # all 13 scenarios
#   ./tools/run_fault_injection.ps1 -Scenarios 0,3,7,10
#   ./tools/run_fault_injection.ps1 -Vsim D:/app/modesilm/win64/vsim.exe
param(
    [int[]]$Scenarios = @(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12),
    [string]$Vsim = 'D:/app/modesilm/win64/vsim.exe',
    [string]$OutDir = 'records/fault_injection/fixed_final_regression'
)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
Push-Location $projectRoot
try {
    New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
    $names = @{
        0 = 'a_front'; 1 = 'a_side'; 2 = 'b_permanent20'; 3 = 'c_i2c_nack'
        4 = 'd_imu_burst'; 5 = 'e_encoder_jitter'; 6 = 'ctrl_100ms20'
        7 = 'g_near_60mm'; 8 = 'h_turn2'; 9 = 'i_hold20'
        10 = 'k_clip_fallback'; 11 = 'l_pass_clip'; 12 = 'm_pass_front_graze'
    }
    $failures = @()
    foreach ($scenario in $Scenarios) {
        $name = if ($names.ContainsKey($scenario)) { $names[$scenario] } else { "scenario_$scenario" }
        $log = Join-Path $OutDir ("fixed_regression_{0:D2}_{1}.log" -f $scenario, $name)
        $env:FAULT_SCENARIO = [string]$scenario
        Write-Host "=== scenario $scenario ($name) ==="
        & $Vsim -c -l $log -do sim/run_fault_turns_fixed.do *> $null
        $code = $LASTEXITCODE
        if (!(Test-Path -LiteralPath $log)) { $failures += "scenario ${scenario}: no log"; continue }
        $text = Get-Content -LiteralPath $log -Raw
        $observed = [regex]::Matches($text, 'FIXED_RUN_OBSERVED scenario=[^\r\n]*')
        $line = if ($observed.Count) { $observed[$observed.Count - 1].Value } else { '' }
        $clean = $code -eq 0 -and $line -match 'fault_seen=0' -and $line -match 'fault_latched=0' -and
            $line -match 'paused=0' -and $line -match 'blocked_seen=0'
        if ($scenario -eq 10) { $clean = $clean -and $line -match 'reached_shift=1' -and $line -match 'fallback_done=1' }
        if ($scenario -eq 7) { $clean = $clean -and $line -match 'near_shift=1' }
        if ($scenario -eq 11) { $clean = $clean -and $line -match 'pass_reached=1' -and $line -match 'pass_clip_done=1' }
        if ($scenario -eq 12) { $clean = $clean -and $line -match 'pass_reached=1' -and $line -match 'pass_front_done=1' }
        if ($clean) { Write-Host "  PASS $line" } else {
            Write-Host "  FAIL exit=$code $line"
            $failures += "scenario $scenario ($name): exit=$code"
        }
    }
    Remove-Item Env:FAULT_SCENARIO -ErrorAction SilentlyContinue
    if ($failures.Count) {
        $failures | ForEach-Object { Write-Host "FAIL $_" -ForegroundColor Red }
        throw ("fault-injection regression failed: " + ($failures -join '; '))
    }
    Write-Host "PASS fault-injection regression: $($Scenarios.Count) scenarios, no fault latch, no PAUSED (logs in $OutDir)"
}
finally {
    Remove-Item Env:FAULT_SCENARIO -ErrorAction SilentlyContinue
    Pop-Location
}

param([string]$Vsim='D:/app/modesilm/win64/vsim.exe')
$ErrorActionPreference='Stop'
Push-Location $PSScriptRoot
try {
  New-Item -ItemType Directory -Force records | Out-Null
  & $Vsim -c -l records/rtl_compile.log -do sim/run_nav.do
  if ($LASTEXITCODE -ne 0) { throw 'RTL compilation failed; see records/rtl_compile.log' }
  & $Vsim -c -l records/rtl_unit.log -do sim/run_pose.do
  if ($LASTEXITCODE -ne 0) { throw 'Pose RTL simulation failed' }
  & $Vsim -c -l records/rtl_guard.log -do sim/run_supervisor.do
  if ($LASTEXITCODE -ne 0) { throw 'Supervisor RTL simulation failed' }
  & $Vsim -c -l records/rtl_event_segment.log -do sim/run_event_segment.do
  if ($LASTEXITCODE -ne 0) { throw 'Event segment RTL simulation failed' }
  & $Vsim -c -l records/rtl_map_association.log -do sim/run_map_association.do
  if ($LASTEXITCODE -ne 0) { throw 'Map wall association simulation failed' }
  & $Vsim -c -l records/rtl_closed_loop.log -do sim/run_closed_loop.do
  if ($LASTEXITCODE -ne 0) { throw 'Physical closed-loop route simulation failed' }
  & $Vsim -c -l records/board_compile.log -do sim/run_board_compile.do
  if ($LASTEXITCODE -ne 0) { throw 'Board-level RTL compile failed' }
  & $Vsim -c -l records/board_safe_default.log -do sim/run_board_safe.do
  if ($LASTEXITCODE -ne 0) { throw 'Board safe-default simulation failed' }
  if (-not (Select-String -Path records/rtl_unit.log -Pattern 'PASS: range 20 discard' -Quiet)) { throw 'Pose RTL PASS marker missing' }
  if (-not (Select-String -Path records/rtl_guard.log -Pattern 'PASS: geometry gate' -Quiet)) { throw 'Supervisor RTL PASS marker missing' }
  if (-not (Select-String -Path records/rtl_event_segment.log -Pattern 'PASS: map-linked sample debounce' -Quiet)) { throw 'Event segment RTL PASS marker missing' }
  if (-not (Select-String -Path records/rtl_map_association.log -Pattern 'PASS: expected-wall range association' -Quiet)) { throw 'Map wall association PASS marker missing' }
  if (-not (Select-String -Path records/rtl_closed_loop.log -Pattern 'PASS: physical wheel/IMU/ToF closed loop' -Quiet)) { throw 'Physical closed-loop PASS marker missing' }
  if (-not (Select-String -Path records/board_safe_default.log -Pattern 'PASS: sensor-calibration fault, injected driver E-stop cutoff/latch, reset-safe re-entry' -Quiet)) { throw 'Probe safe-default PASS marker missing' }
  & python tools/navigation_geometry.py | Tee-Object -FilePath records/navigation_geometry_run.log
  if ($LASTEXITCODE -ne 0) { throw 'Navigation geometry report failed' }
  & python tools/geometry_audit.py | Tee-Object -FilePath records/geometry_run.log
  if ($LASTEXITCODE -ne 0) { throw 'Geometry audit failed' }
  & python tools/collision_prediction.py | Tee-Object -FilePath records/collision_prediction_run.log
  if ($LASTEXITCODE -ne 0) { throw 'Collision prediction failed' }
} finally { Pop-Location }

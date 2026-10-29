param([string]$Destination = 'release/Embodied_RPACS_main_20261026')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$exportRoot = [IO.Path]::GetFullPath((Join-Path $projectRoot $Destination))
if (!$exportRoot.StartsWith($projectRoot + [IO.Path]::DirectorySeparatorChar, [StringComparison]::OrdinalIgnoreCase)) {
    throw 'Export destination must be inside this project'
}
if (Test-Path -LiteralPath $exportRoot) { throw 'Destination already exists; choose a new destination' }
New-Item -ItemType Directory -Path $exportRoot | Out-Null
function Copy-MainFile([string]$relative) {
    $target = Join-Path $exportRoot $relative
    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $target) | Out-Null
    Copy-Item -LiteralPath (Join-Path $projectRoot $relative) -Destination $target
}
[xml]$project = Get-Content -LiteralPath (Join-Path $projectRoot 'Embodied_RPACS.gprj') -Raw
foreach ($file in $project.Project.FileList.File) {
    if ($file.path -match '(?i)(^|/)tests/|\.\.|^[A-Za-z]:|^[/\\]') { throw "Non-independent source: $($file.path)" }
    Copy-MainFile $file.path
}
foreach ($file in @(
    'OBSTACLE_BOARD_TEST.md','Embodied_RPACS.gprj','build.tcl','verify_project.tcl','README.md','PIN_ASSIGNMENT.md',
    'impl/Embodied_RPACS_process_config.json','impl/project_process_config.json',
    'sim/compile_system.do','sim/run_system.do','sim/run_system_oled.do','sim/run_system_50mhz.do',
    'sim/run_system_dispatch.do','sim/run_live_speed.do','sim/run_system_obstacle.do','sim/run_attitude_pipeline.do',
    'sim/tb_system_dispatch.sv','sim/tb_live_speed.sv','sim/tb_system_obstacle.sv','sim/tb_attitude_pipeline.sv','sim/tb_robot_system.sv','sim/tb_system_event_logger.sv','sim/tb_system_oled.sv','sim/tb_system_50mhz.sv',
    'sim/models/vl53l0x_model.sv','tools/run_system_sim.ps1','tools/check_system_reports.ps1','tools/export_main.ps1','tools/board_obstacle_test.ps1','tools/read_serial_log.ps1','tools/run_legacy_regression.ps1','tools/run_fault_injection.ps1','tools/read_serial_log.ps1'
)) { Copy-MainFile $file }
foreach ($file in @('THIRD_PARTY_NOTICES.md','records/obstacle_20261025/VALIDATION.md')) {
    if (Test-Path -LiteralPath (Join-Path $projectRoot $file)) { Copy-MainFile $file }
}
New-Item -ItemType Directory -Force -Path (Join-Path $exportRoot 'records/obstacle_20261025') | Out-Null
if (Test-Path -LiteralPath (Join-Path $exportRoot 'tests')) { throw 'Unexpected test directory in main export' }
Write-Output "PASS self-contained main source export: $exportRoot"



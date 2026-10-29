# Read-only acceptance checks against the generated Gowin reports and mapped netlist.
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
function Require($condition, [string]$message) { if (!$condition) { throw $message } }
function Read-Project([string]$relative) { Get-Content -LiteralPath (Join-Path $projectRoot $relative) -Raw }
# Windows PowerShell 5.1 cannot parse Gowin's process configs (case-duplicate
# keys), so pull the few checked fields out with a regex instead of JSON.
function Get-OptionValue([string]$text, [string]$key) {
    if ($text -match ('"(?i)' + [regex]::Escape($key) + '"\s*:\s*("[^"]*"|[^,}

]+)')) { return $Matches[1].Trim('"') }
    return $null
}
function Html-Rows([string]$html) {
    foreach ($row in [regex]::Matches($html, '(?is)<tr\b[^>]*>(.*?)</tr>')) {
        $cells = @([regex]::Matches($row.Groups[1].Value, '(?is)<t[dh]\b[^>]*>(.*?)</t[dh]>') | ForEach-Object {
            [System.Net.WebUtility]::HtmlDecode(([regex]::Replace($_.Groups[1].Value, '<[^>]*>', '')).Trim())
        })
        if ($cells.Count) { ,$cells }
    }
}
$expectedPins = @{
 clk_50m='T9'; reset_n='B16'; key_enable_n='A15'; key_estop_n='C15'
 encoder_a_l='C17';encoder_b_l='C18';encoder_a_r='F14';encoder_b_r='G14'
 PWMA='U17';AIN1='M16';AIN2='M14';PWMB='T17';BIN1='N15';BIN2='H12';STBY='G13'
 imu_spi_sck='V15';imu_spi_mosi='V16';imu_spi_miso='U18';imu_cs_n='T18';imu_int1='N14';imu_int2='N16'
 imu_gnd_reserved='T11';led_ready='D14';led_run='C14';led_fault='B9';led_heartbeat='A9'
 oled_scl='R3';oled_sda='T3';asr_rx='D17';asr_tx='D18'
 tof_front_scl='G16';tof_front_sda='G18';tof_front_gpio1='F17';tof_front_xshut_n='F18'
 tof_left_scl='H13';tof_left_sda='H14';tof_left_gpio1='K15';tof_left_xshut_n='K16'
 tof_right_scl='J13';tof_right_sda='K14';tof_right_gpio1='K12';tof_right_xshut_n='K13'
}
$inputs=@('clk_50m','reset_n','key_enable_n','key_estop_n','encoder_a_l','encoder_b_l','encoder_a_r','encoder_b_r','imu_spi_miso','imu_int1','imu_int2','imu_gnd_reserved','asr_rx','tof_front_gpio1','tof_left_gpio1','tof_right_gpio1')
$down=@('encoder_a_l','encoder_b_l','encoder_a_r','encoder_b_r','imu_int1','imu_int2')
$up=@('key_enable_n','key_estop_n','asr_rx')
$cst=Read-Project 'src/robot_system_top.cst'
$locations=[regex]::Matches($cst,'IO_LOC\s+"([^"]+)"\s+(\w+)')
Require ($locations.Count -eq $expectedPins.Count) 'Unexpected constraint port count'
$used=@{}
foreach($location in $locations) {
 $name=$location.Groups[1].Value;$pin=$location.Groups[2].Value
 Require ($expectedPins[$name] -eq $pin) "Wrong source constraint: $name=$pin"
 Require (!$used.ContainsKey($pin)) "Duplicate pin: $pin"
 $used[$pin]=$name
}
$pinRows=@{}
foreach($row in (Html-Rows (Read-Project 'impl/pnr/Embodied_RPACS.pin.html'))) {
 if($expectedPins.ContainsKey($row[0])) { $pinRows[$row[0]]=$row }
}
foreach($name in $expectedPins.Keys) {
 Require ($pinRows.ContainsKey($name)) "Missing generated pin: $name"
 $row=$pinRows[$name]
 $direction=if($name -eq 'oled_sda' -or $name -match '^tof_.*_(scl|sda|xshut_n)$'){'io'}elseif($inputs -contains $name){'in'}else{'out'}
 $pull=if($down -contains $name){'DOWN'}elseif($up -contains $name){'UP'}else{'NONE'}
 $drive=if($direction -eq 'in'){'OFF'}else{'8'}
 Require ($row[2].Split('/')[0] -eq $expectedPins[$name] -and $row[3] -eq 'Y') "Unconstrained/wrong generated pin: $name"
 Require ($row[4] -eq $direction -and $row[7] -eq 'LVCMOS33' -and $row[8] -eq $drive -and $row[9] -eq $pull) "Wrong electrical settings: $name"
}
[xml]$project=Read-Project 'Embodied_RPACS.gprj'
$files=@($project.Project.FileList.File)
Require (@($files | Where-Object { $_.top -eq '1' -and $_.path -eq 'src/robot_system_top.v' }).Count -eq 1) 'Wrong top module'
Require (@($files | Where-Object { $_.type -eq 'file.cst' }).Count -eq 1) 'More than one CST'
Require (@($files | Where-Object { $_.type -eq 'file.sdc' }).Count -eq 1) 'More than one SDC'
Require (@($files | Where-Object { $_.path -match 'camera|ov5647|ddr|hdmi' }).Count -eq 0) 'Camera/display source included'
foreach($module in @('imu_attitude_fixed','attitude_angle_to_digits','oled_i2c_write_byte')) {
 Require (@($files | Where-Object { $_.path -eq "src/imu/$module.v" -and $_.enable -eq '1' }).Count -eq 1) "Missing OLED/attitude source: $module"
}
$deviceConfig=Read-Project 'impl/pnr/device.cfg'
Require ($deviceConfig -match 'set CPU regular_io = true' -and $deviceConfig -match 'set SSPI regular_io = true') 'OLED dual-purpose pins not released'
Require ((Read-Project 'impl/pnr/cmd.do') -match '-global_freq 50\.000') 'Wrong PnR frequency'
foreach($config in @('impl/Embodied_RPACS_process_config.json','impl/project_process_config.json')) {
 $optionsText=Read-Project $config
 Require ((Get-OptionValue $optionsText 'CPU') -eq 'true' -and (Get-OptionValue $optionsText 'SSPI') -eq 'true' -and [double](Get-OptionValue $optionsText 'Global_Freq') -eq 50 -and (Get-OptionValue $optionsText 'TopModule') -eq 'robot_system_top') "Wrong saved IDE options: $config"
}
$netlist=Read-Project 'impl/gwsynthesis/Embodied_RPACS.vg'
Require ($netlist -match '(?s)TBUF oled_sda_s0\s*\(\s*\.O\(oled_sda\),\s*\.I\(GND\),\s*\.OEN\(') 'OLED SDA must only drive low or release'
foreach($module in @('imu660ra_driver','imu_attitude_fixed','motion_control_and_mode_fsm','dual_ab_quadrature_encoder','dual_speed_pi_pid_controller','tb6612_driver_interface')) {
 Require ([regex]::Matches($netlist, ('(?m)^\s*'+$module+'\s+\w+\s*\(')).Count -eq 1) "Expected one owner of $module"
}
foreach($name in @('tof_front_scl','tof_front_sda','tof_front_xshut_n','tof_left_scl','tof_left_sda','tof_left_xshut_n','tof_right_scl','tof_right_sda','tof_right_xshut_n')) {
 $pattern=if($name -match '_xshut_n$') {
  '(?s)TBUF '+$name+'_s0\s*\(\s*\.O\('+$name+'\),\s*\.I\(GND\),\s*\.OEN\('
 } else {
  '(?s)IOBUF '+$name+'_iobuf\s*\(\s*\.O\([^)]*\),\s*\.IO\('+$name+'\),\s*\.I\(GND\),\s*\.OEN\('
 }
 Require ($netlist -match $pattern) "ToF output must be open drain: $name"
}
Require ($netlist -notmatch '(?im)^\s*(?:PLL\w*|rPLL)\s+\S+\s*\(') 'Unexpected PLL instance'
Require (@($files | Where-Object { $_.path -match '(?i)(^|/)tests/|test_top|heartbeat_watchdog' }).Count -eq 0) 'Main project depends on test/heartbeat sources'
foreach($file in $files) { Require ($file.path -notmatch '^[/\\]|^[A-Za-z]:|\.\.') "Nonportable source: $($file.path)" }
$timing=Read-Project 'impl/pnr/Embodied_RPACS_tr_content.html'
foreach($row in (Html-Rows $timing)) {
 if($row[0] -match '^Numbers of (Setup|Hold) Violated Endpoints$') { Require ($row[1] -eq '0') "Timing violation: $($row -join ' ')" }
 if($row[0] -eq 'clk_50m' -and $row[1] -in @('Setup','Hold')) { Require ($row[2] -eq '0.000' -and $row[3] -eq '0') 'Nonzero TNS' }
}
$bitstream=Join-Path $projectRoot 'impl/pnr/Embodied_RPACS.fs'
Require (Test-Path -LiteralPath $bitstream) 'Missing bitstream'
$reportDate=(Get-Item -LiteralPath $bitstream).LastWriteTimeUtc
foreach($file in $files) { Require ((Get-Item -LiteralPath (Join-Path $projectRoot $file.path)).LastWriteTimeUtc -le $reportDate) "Stale bitstream: $($file.path)" }
Write-Output 'PASS 42 unique constrained pins, electrical settings, ten open-drain outputs, CPU/SSPI GPIO, saved 50MHz options, one IMU/encoder/motion/PI/driver owner, no test/heartbeat dependency, zero setup/hold violations and fresh bitstream'
Get-FileHash -LiteralPath $bitstream -Algorithm SHA256 | Select-Object Hash,Path

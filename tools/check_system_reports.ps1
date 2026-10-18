# Read-only acceptance checks against the generated Gowin reports and mapped netlist.
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
function Require($condition, [string]$message) { if (!$condition) { throw $message } }
function Read-Project([string]$relative) { Get-Content -LiteralPath (Join-Path $projectRoot $relative) -Raw }
function Html-Rows([string]$html) {
    foreach ($row in [regex]::Matches($html, '(?is)<tr\b[^>]*>(.*?)</tr>')) {
        $cells = @([regex]::Matches($row.Groups[1].Value, '(?is)<t[dh]\b[^>]*>(.*?)</t[dh]>') | ForEach-Object {
            [System.Net.WebUtility]::HtmlDecode(([regex]::Replace($_.Groups[1].Value, '<[^>]*>', '')).Trim())
        })
        if ($cells.Count) { ,$cells }
    }
}
$expectedPins = @{
 clk_50m='T9'; reset_n='B16'; key_enable_n='A15'; key_estop_n='C15'; uart_rx='U8'; uart_tx='V8'
 encoder_a_l='C17';encoder_b_l='C18';encoder_a_r='F14';encoder_b_r='G14'
 PWMA='U17';AIN1='M16';AIN2='M14';PWMB='T17';BIN1='N15';BIN2='H12';STBY='G13'
 imu_spi_sck='V15';imu_spi_mosi='V16';imu_spi_miso='U18';imu_cs_n='T18';imu_int1='N14';imu_int2='N16'
 imu_gnd_reserved='T11';led_ready='D14';led_run='C14';led_fault='B9';led_heartbeat='A9'
}
$inputs=@('clk_50m','reset_n','key_enable_n','key_estop_n','uart_rx','encoder_a_l','encoder_b_l','encoder_a_r','encoder_b_r','imu_spi_miso','imu_int1','imu_int2','imu_gnd_reserved')
$down=@('encoder_a_l','encoder_b_l','encoder_a_r','encoder_b_r','imu_int1','imu_int2')
$up=@('key_enable_n','key_estop_n','uart_rx')
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
 $direction=if($inputs -contains $name){'in'}else{'out'}
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
$netlist=Read-Project 'impl/gwsynthesis/Embodied_RPACS.vg'
foreach($entry in @(@('position',64),@('illegal_transition',64),@('imu_sample',99))) {
 $count=[regex]::Matches($netlist, ('\bDFF\w+\s+'+$entry[0]+'_\d+_s\d+\s*\(')).Count
 Require ($count -eq $entry[1]) "Preserved $($entry[0]) bits: $count, expected $($entry[1])"
}
Require ([regex]::Matches($netlist,'\buart_tx\s+serial\s*\(').Count -eq 1) 'Expected exactly one UART transmitter'
Require ($netlist -notmatch '(?im)^\s*(?:PLL\w*|rPLL|MULT\w*|DSP\w*)\s+\S+\s*\(') 'Unexpected PLL/DSP instance'
$timing=Read-Project 'impl/pnr/Embodied_RPACS_tr_content.html'
foreach($row in (Html-Rows $timing)) {
 if($row[0] -match '^Numbers of (Setup|Hold) Violated Endpoints$') { Require ($row[1] -eq '0') "Timing violation: $($row -join ' ')" }
 if($row[0] -eq 'clk_50m' -and $row[1] -in @('Setup','Hold')) { Require ($row[2] -eq '0.000' -and $row[3] -eq '0') 'Nonzero TNS' }
}
$bitstream=Join-Path $projectRoot 'impl/pnr/Embodied_RPACS.fs'
Require (Test-Path -LiteralPath $bitstream) 'Missing bitstream'
$reportDate=(Get-Item -LiteralPath $bitstream).LastWriteTimeUtc
foreach($file in $files) { Require ((Get-Item -LiteralPath (Join-Path $projectRoot $file.path)).LastWriteTimeUtc -le $reportDate) "Stale bitstream: $($file.path)" }
Write-Output 'PASS 28 unique constrained pins/electrical settings, T11 input/NONE, one top/CST/SDC, no camera, 64 position + 64 illegal counter bits, 99 IMU sample/status bits, one UART TX, no DSP/PLL, zero setup/hold violations and TNS'
Get-FileHash -LiteralPath $bitstream -Algorithm SHA256 | Select-Object Hash,Path

param(
    [string]$Port,
    [ValidateSet('forward','backward','left','right','stop','status','faster','slower','search','monitor','calibration')]
    [string]$Action = 'monitor',
    [ValidateRange(1,3600)][int]$Seconds = 15,
    [switch]$SelfTest
)
$ErrorActionPreference = 'Stop'
$codes = @{forward=0x10;backward=0x11;left=0x12;right=0x13;stop=2;status=3;faster=0x14;slower=0x15;search=1}
$statusNames = @{0='IDLE';4='STOPPED';6='FAULT';0x10='MOVING';0x11='DONE'}
$responseNames = @{0='ACCEPTED';1='BUSY';2='UNAVAILABLE'}
$script:window = [Collections.Generic.List[byte]]::new()
$script:age = [Diagnostics.Stopwatch]::StartNew()
function Get-CommandFrame([int]$Code) { return ,([byte[]](0xAA,0x55,1,$Code,(($Code+1)-band 255),13)) }
function Read-FeedbackByte([byte]$Value) {
    if ($script:window.Count -gt 0 -and $script:age.ElapsedMilliseconds -ge 20) { $script:window.Clear() }
    if ($script:window.Count -eq 0) {
        if ($Value -ne 0xAA) { return }
        $script:age.Restart()
    }
    $script:window.Add($Value)
    if ($script:window.Count -gt 6) { $script:window.RemoveAt(0) }
    $w = $script:window
    if ($w.Count -eq 6 -and $w[0] -eq 0xAA -and $w[1] -eq 0x55 -and
        ($w[2] -eq 2 -or $w[2] -eq 3) -and $w[4] -eq (($w[2]+$w[3])-band 255) -and $w[5] -eq 13) {
        $result = [pscustomobject]@{Type=[int]$w[2];Code=[int]$w[3]}
        $w.Clear()
        return $result
    }
}
function Convert-SignedHex([string]$Hex) {
    $number = [Convert]::ToInt64($Hex,16)
    if ($number -ge 2147483648) { $number -= 4294967296 }
    return $number
}
if ($SelfTest) {
    foreach ($name in $codes.Keys) {
        $f = Get-CommandFrame $codes[$name]
        if ($f.Length -ne 6 -or $f[4] -ne (($f[2]+$f[3])-band 255)) { throw 'Command encoding failed' }
    }
    $decoded = @(foreach ($b in [byte[]](0x77,0xAA,0xAA,0x55,2,0,2,13,0xAA,0x55,3,0x11,0x14,13)) { Read-FeedbackByte $b })
    if ($decoded.Count -ne 2 -or $decoded[0].Type -ne 2 -or $decoded[1].Code -ne 0x11) { throw 'Feedback decoding failed' }
    if ((Convert-SignedHex 'FFFFFFFF') -ne -1 -or (Convert-SignedHex '80000000') -ne -2147483648) { throw 'Signed calibration decode failed' }
    Write-Output 'PASS command frames, feedback resynchronization, signed diagnostic decode; no serial port opened'
    exit 0
}
if (-not $Port) { throw 'Specify -Port COMn, or use -SelfTest for offline verification.' }
$serial = [IO.Ports.SerialPort]::new($Port,115200,[IO.Ports.Parity]::None,8,[IO.Ports.StopBits]::One)
$serial.ReadTimeout = 50
$serial.WriteTimeout = 1000
$serial.DtrEnable = $false
$serial.RtsEnable = $false
$serial.Handshake = [IO.Ports.Handshake]::None
$watch = [Diagnostics.Stopwatch]::StartNew()
$acknowledged = $false
$accepted = $false
$exitCode = 0
try {
    $serial.Open()
    $serial.DiscardInBuffer()
    if ($codes.ContainsKey($Action)) {
        $frame = Get-CommandFrame $codes[$Action]
        Write-Output ('TX '+[BitConverter]::ToString($frame).Replace('-',' '))
        $serial.Write($frame,0,$frame.Length)
        $exitCode = 1
    }
    $line = ''
    $buffer = [byte[]]::new(512)
    while ($watch.Elapsed.TotalSeconds -lt $Seconds) {
        $available = [Math]::Min($serial.BytesToRead,$buffer.Length)
        if ($available -eq 0) { Start-Sleep -Milliseconds 2; continue }
        $length = $serial.Read($buffer,0,$available)
        for ($i=0; $i -lt $length; $i++) {
            if ($Action -eq 'calibration') {
                if ($buffer[$i] -eq 10) {
                    if ($line.Trim() -match '^L=([0-9A-F]{8}) R=([0-9A-F]{8}) Y=([0-9A-F]{8})$') {
                        Write-Output ('L={0} counts  R={1} counts  Y={2} mdeg' -f
                            (Convert-SignedHex $Matches[1]),(Convert-SignedHex $Matches[2]),(Convert-SignedHex $Matches[3]))
                    }
                    $line = ''
                } elseif ($line.Length -lt 128) { $line += [char]$buffer[$i] } else { $line = '' }
                continue
            }
            $feedback = Read-FeedbackByte $buffer[$i]
            if ($null -eq $feedback) { continue }
            if ($feedback.Type -eq 2) {
                $acknowledged = $true
                $accepted = $feedback.Code -eq 0
                Write-Output ('RESPONSE {0:X2} {1}' -f $feedback.Code,$responseNames[$feedback.Code])
            } else {
                Write-Output ('STATUS {0:X2} {1}' -f $feedback.Code,$statusNames[$feedback.Code])
                if ($Action -eq 'monitor') { continue }
                if ($acknowledged) {
                    if (-not $accepted -or $feedback.Code -eq 6) { $exitCode = 2; break }
                    if (($Action -eq 'stop' -and $feedback.Code -eq 4) -or
                        ($Action -in @('forward','backward','left','right') -and $feedback.Code -eq 0x11) -or
                        ($Action -in @('status','faster','slower','search'))) { $exitCode = 0; break }
                }
            }
        }
        if ($codes.ContainsKey($Action) -and $acknowledged -and ($exitCode -eq 0 -or $exitCode -eq 2)) { break }
    }
    if ($exitCode -eq 1) { Write-Warning 'Timed out without confirmed completion. No command was retried; the board retains its own 10-second action timeout.' }
} finally { if ($serial.IsOpen) { $serial.Close() }; $serial.Dispose() }
exit $exitCode

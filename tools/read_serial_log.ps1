# Read-only serial listener for FPGA event diagnostics (115200 8N1).
# Never writes to the port: the USB link is FPGA->PC diagnostics only, and the
# motion control path (ASR UART) is physically wired to the ASRPRO board.
param(
    [string]$Port = 'COM14',
    [int]$Seconds = 60,
    [string]$OutFile = 'records/board_smoke_serial.log'
)
$ErrorActionPreference = 'Stop'
$sp = New-Object System.IO.Ports.SerialPort
$sp.PortName = $Port
$sp.BaudRate = 115200
$sp.Parity = [System.IO.Ports.Parity]::None
$sp.StopBits = [System.IO.Ports.StopBits]::One
$sp.DataBits = 8
$sp.ReadTimeout = 500
$sp.DtrEnable = $false
$sp.RtsEnable = $false
$sp.Open()
$deadline = (Get-Date).AddSeconds($Seconds)
$log = New-Object System.Collections.Generic.List[string]
try {
    while ((Get-Date) -lt $deadline) {
        try {
            $line = $sp.ReadLine()
            $stamp = (Get-Date).ToString('HH:mm:ss.fff')
            $entry = "[$stamp] $line"
            $log.Add($line)
            Write-Host $entry
        } catch [TimeoutException] { }
    }
} finally {
    $sp.Close()
}
$log | Set-Content -Path $OutFile -Encoding ASCII
Write-Host "captured $($log.Count) lines -> $OutFile"

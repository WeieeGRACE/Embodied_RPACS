# Read-only serial listener for FPGA event diagnostics (115200 8N1).
# Never writes to the port: the USB link is FPGA->PC diagnostics only, and the
# motion control path (ASR UART) is physically wired to the ASRPRO board.
# Lines are appended to the output file immediately so a long capture survives
# a port disconnect or a competing serial tool.
param(
    [string]$Port = 'COM14',
    [int]$Seconds = 60,
    [string]$OutFile = 'records/board_smoke_serial.log',
    [switch]$Timestamped
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
$outDir = Split-Path -Parent $OutFile
if ($outDir -and !(Test-Path -LiteralPath $outDir)) { New-Item -ItemType Directory -Force -Path $outDir | Out-Null }
Set-Content -LiteralPath $OutFile -Value '' -Encoding ASCII
$count = 0
try {
    $sp.Open()
    $deadline = (Get-Date).AddSeconds($Seconds)
    $writer = New-Object System.IO.StreamWriter($OutFile, $true, [System.Text.Encoding]::ASCII)
    try {
        while ((Get-Date) -lt $deadline) {
            try {
                $line = $sp.ReadLine()
            } catch [TimeoutException] { continue }
            $stamp = (Get-Date).ToString('HH:mm:ss.fff')
            $entry = "[$stamp] $line"
            if ($Timestamped) { $writer.WriteLine($entry) } else { $writer.WriteLine($line) }
            $count++
            if (($count % 20) -eq 0) { $writer.Flush() }
            Write-Host $entry
        }
    } finally {
        $writer.Flush()
        $writer.Close()
    }
} catch {
    Write-Warning "capture stopped after $count lines: $($_.Exception.Message)"
} finally {
    if ($sp.IsOpen) { $sp.Close() }
}
Write-Host "captured $count lines -> $OutFile"

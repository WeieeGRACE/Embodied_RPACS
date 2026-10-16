param([Parameter(Mandatory=$true)][string]$Port)
# Close the serial assistant first: one process owns this COM port.
$serial = [System.IO.Ports.SerialPort]::new($Port,115200,[System.IO.Ports.Parity]::None,8,[System.IO.Ports.StopBits]::One)
$serial.Handshake = [System.IO.Ports.Handshake]::None
$serial.ReadTimeout = 100
$serial.WriteTimeout = 500
$logPath = Join-Path $PSScriptRoot ("heartbeat_" + (Get-Date -Format "yyyyMMdd_HHmmss") + ".log")
$log = [System.IO.StreamWriter]::new($logPath,$false,[System.Text.Encoding]::UTF8)
try {
 $serial.Open()
 Write-Host "Heartbeat A5 every 100 ms. SPACE toggles heartbeat; Q quits. Telemetry: $logPath"
 $sending = $true
 $timer = [System.Diagnostics.Stopwatch]::StartNew()
 $next = 0L
 while ($true) {
  if ([Console]::KeyAvailable) {
   $key = [Console]::ReadKey($true).Key
   if ($key -eq [ConsoleKey]::Q) { break }
   if ($key -eq [ConsoleKey]::Spacebar) {
    $sending = -not $sending
    $log.WriteLine("# " + (Get-Date -Format o) + " sending=" + $sending)
    $log.Flush()
    Write-Host "sending=$sending (resume cannot clear a latched watchdog)"
   }
  }
  if ($sending -and $timer.ElapsedMilliseconds -ge $next) {
   $serial.Write([byte[]]@(0xA5),0,1)
   $log.WriteLine("# " + (Get-Date -Format o) + " TX=A5")
   $next = $timer.ElapsedMilliseconds + 100
  }
  if ($serial.BytesToRead -gt 0) {
   $text = $serial.ReadExisting()
   $log.Write($text)
   [Console]::Write($text)
   $log.Flush()
  }
  Start-Sleep -Milliseconds 5
 }
} finally {
 if ($serial.IsOpen) { $serial.Close() }
 $serial.Dispose()
 $log.Dispose()
}

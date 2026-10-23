param(
    [string]$Port = 'COM14',
    [int]$Lines = 30,
    [string]$Log = 'records/board_latest.log',
    [switch]$Download
)
$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    if ($Download) {
        & python check_reports.py
        if ($LASTEXITCODE -ne 0) { throw 'Artifact checks failed' }
        $tofFs = (Resolve-Path 'impl/pnr/vl53l0x_three.fs').Path
        & 'D:/Gowin/Gowin_V1.9.12_x64/Programmer/bin/programmer_cli.exe' `
            --device GW5A-25A --operation_index 2 --fsFile $tofFs *> records/download_latest.log
        $tofDownload = Get-Content records/download_latest.log -Raw
        if ($LASTEXITCODE -ne 0 -or $tofDownload -notmatch 'Finished\.' -or $tofDownload -match 'Error:') {
            throw 'SRAM download failed; see records/download_latest.log'
        }
    }
    $tofPort = [System.IO.Ports.SerialPort]::new($Port,115200,[System.IO.Ports.Parity]::None,8,[System.IO.Ports.StopBits]::One)
    try {
        $tofPort.ReadTimeout = 10000
        $tofPort.Open()
        $tofPort.DiscardInBuffer()
        $tofRows = @(for ($tofIndex=0; $tofIndex -lt $Lines; $tofIndex++) { $tofPort.ReadLine().TrimEnd() })
        $tofRows | Set-Content -LiteralPath $Log -Encoding utf8
        $tofRows | Write-Output
    } finally { if ($tofPort.IsOpen) { $tofPort.Close() }; $tofPort.Dispose() }
    & python check_board.py --log $Log
    if ($LASTEXITCODE -ne 0) { throw 'Captured data failed integrity checks' }
} finally { Pop-Location }

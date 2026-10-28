param(
    [string]$GwSh = 'D:/Gowin/Gowin_V1.9.12_x64/IDE/bin/gw_sh.exe',
    [string]$Programmer = 'D:/Gowin/Gowin_V1.9.12_x64/Programmer/bin/programmer_cli.exe',
    [switch]$Download
)
# Build the standalone UART bridge project and print the resource budget.
#
# SAFETY: -Download programs SRAM only (--run 2), never the external flash, so a power
# cycle restores whatever firmware was there before. The bitstream keeps every motor
# driver output low and STBY = 0, so the cart cannot move.
$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    New-Item -ItemType Directory -Force records | Out-Null
    & $GwSh build.tcl *> records/build.log
    if ($LASTEXITCODE -ne 0) { throw "build failed; see records/build.log" }

    $rpt = 'impl/pnr/uart_bridge.rpt.txt'
    if (!(Test-Path $rpt)) { throw "missing $rpt" }
    $lines = Get-Content -LiteralPath $rpt
    $start = ($lines | Select-String -Pattern '^3\. Resource Usage Summary$').LineNumber
    if ($start) {
        $lines[($start - 1)..([Math]::Min($start + 12, $lines.Count - 1))] | ForEach-Object { Write-Host $_ }
    }
    $fs = 'impl/pnr/uart_bridge.fs'
    if (!(Test-Path $fs)) { throw "missing bitstream $fs" }
    Write-Host "bitstream: $fs  ($((Get-Item $fs).Length) bytes)"

    if ($Download) {
        & $Programmer --device GW5A-25A --run 2 --fsFile (Resolve-Path $fs) `
            --cable 'Gowin USB Cable(FT2CH)' | Tee-Object records/download_sram.log | Out-Null
        Write-Host 'SRAM programmed (volatile). Power-cycle restores the previous firmware.'
    }
} finally {
    Pop-Location
}

param(
    [string]$Port = 'COM14',
    [double]$Duration = 20,
    [double]$Soak = 30,
    [switch]$SkipSoak
)
# PC 侧先跑一遍（不需要相机），确认"接线 + 位流 + 协议"三条都对，再部署到 Pi。
#
#   .\run_pc.ps1 -Port COM14                 # 预检 + 20s 链路采集
#   .\run_pc.ps1 -Port COM14 -Soak 60        # 再跑 60s 链路 soak（--no-camera）
#
# 依赖：python（PATH 里的即可）+ tests/uart_bridge/tools（协议库，自动引用）
$ErrorActionPreference = 'Stop'
try { [Console]::OutputEncoding = [System.Text.Encoding]::UTF8 } catch {}
$proj = Split-Path $PSScriptRoot -Parent
$rec = Join-Path $proj 'records'
New-Item -ItemType Directory -Force $rec | Out-Null
$py = if ($env:PYTHON) { $env:PYTHON } else { 'python' }
$ts = Get-Date -Format 'HHmm'
$env:PYTHONIOENCODING = 'utf-8'

$json = @()
Write-Host "== 1) 预检 $Port =="
& $py (Join-Path $proj 'src\preflight.py') --port $Port --json (Join-Path $rec "preflight_$ts.json")
$pf = $LASTEXITCODE
$json += (Join-Path $rec "preflight_$ts.json")

Write-Host "`n== 2) 链路采集 ${Duration}s =="
& $py (Join-Path $proj 'src\link_client.py') --port $Port --duration $Duration `
    --csv (Join-Path $rec "link_$ts.csv") --json (Join-Path $rec "link_$ts.json")
$lk = $LASTEXITCODE
$json += (Join-Path $rec "link_$ts.json")

if (!$SkipSoak) {
    Write-Host "`n== 3) 链路 soak ${Soak}s（无相机）=="
    & $py (Join-Path $proj 'src\camera_link_soak.py') --port $Port --duration $Soak `
        --no-camera --json (Join-Path $rec "soak_link_$ts.json")
    $sk = $LASTEXITCODE
    $json += (Join-Path $rec "soak_link_$ts.json")
}

Write-Host "`n== 4) 报告 =="
& $py (Join-Path $proj 'src\report.py') @json --out (Join-Path $rec "report_pc_$ts.md") `
    --title "PC 侧链路预跑（$Port）"

Write-Host ""
if ($pf -eq 0 -and $lk -eq 0) {
    Write-Host "PC 侧通过 —— 可以把串口线移到 Pi，执行 .\tools\deploy_to_pi.ps1"
    exit 0
} else {
    Write-Host "PC 侧未通过：preflight=$pf link=$lk（先看 records\report_pc_$ts.md）"
    exit 1
}

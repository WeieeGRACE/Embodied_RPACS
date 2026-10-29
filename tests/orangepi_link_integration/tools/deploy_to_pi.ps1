param(
    [string]$Remote = 'aipro',
    [string]$RemoteDir = '/home/HwHiAiUser/rpacs_link_integration',
    [switch]$SkipPreflight,
    [switch]$InstallDeps
)
# 把本测试工程 + 复用到的 uart_bridge/tools 部署到 Orange Pi，并在板上跑预检。
#
#   .\deploy_to_pi.ps1                 # 部署 + 预检
#   .\deploy_to_pi.ps1 -InstallDeps    # 顺带在相机 venv 里装 pyserial
#   .\deploy_to_pi.ps1 -SkipPreflight  # 只部署
#
# 依赖：本机 OpenSSH（ssh/scp）已能用密钥免密登录 $Remote。
# 远端用相机 venv 的 python（里面有 numpy / pyorbbecsdk / pyserial 3.5）。
$ErrorActionPreference = 'Stop'
try { [Console]::OutputEncoding = [System.Text.Encoding]::UTF8 } catch {}
$here = $PSScriptRoot                                   # ...\tests\orangepi_link_integration\tools
$proj = Split-Path $here -Parent                        # ...\tests\orangepi_link_integration
$tests = Split-Path $proj -Parent                       # ...\tests
$uartTools = Join-Path $tests 'uart_bridge\tools'
$venvPy = '~/rpacs_phase0/pyorbbecsdk-main/.venv/bin/python'

function Invoke-Remote([string]$cmd) {
    $out = ssh $Remote $cmd 2>&1
    $out
    return $LASTEXITCODE
}

if (!(Test-Path $uartTools)) { throw "missing $uartTools" }
if (!(Get-Command scp -ErrorAction SilentlyContinue)) { throw 'scp not found (install OpenSSH client)' }

Write-Host "== 1) 远端准备目录 =="
Invoke-Remote "mkdir -p $RemoteDir/src $RemoteDir/config $RemoteDir/tools $RemoteDir/records $RemoteDir/uart_tools" | Out-Host
Invoke-Remote "chmod +x $RemoteDir/run_all.sh 2>/dev/null; true" | Out-Null

Write-Host "== 2) 上传文件 =="
# uart_bridge 的编解码工具（单一事实来源）
scp (Join-Path $uartTools '*.py') "${Remote}:${RemoteDir}/uart_tools/" | Out-Host
if ($LASTEXITCODE -ne 0) { throw 'scp uart_tools failed' }
# 本工程
scp (Join-Path $proj 'config\link.json') "${Remote}:${RemoteDir}/config/" | Out-Host
scp (Join-Path $proj 'src\*.py') "${Remote}:${RemoteDir}/src/" | Out-Host
scp (Join-Path $proj 'tools\*.py') "${Remote}:${RemoteDir}/tools/" | Out-Host
scp (Join-Path $proj 'run_all.sh') "${Remote}:${RemoteDir}/" | Out-Host
if ($LASTEXITCODE -ne 0) { throw 'scp project files failed' }

Write-Host "== 3) 远端环境检查 =="
Invoke-Remote @"
echo '-- serial devices --'; ls -l /dev/ttyUSB* /dev/serial/by-id/* 2>/dev/null || echo 'NO ttyUSB (FPGA 串口线未插到 Pi)';
echo '-- usb chips --'; lsusb | grep -iE '1a86|0403|ch34|ch9102|2bc5|orbbec' || echo 'none';
echo '-- camera --'; lsusb | grep -iE 'orbbec|2bc5' || echo 'NO orbbec camera on USB';
echo '-- python --'; $venvPy -c 'import sys;print(sys.version.split()[0])';
echo '-- pyserial --'; $venvPy -c 'import serial;print(serial.__version__)' 2>&1 | tail -1;
echo '-- groups --'; groups
"@ | Out-Host

if ($InstallDeps) {
    Write-Host "== 4) 安装 pyserial（相机 venv） =="
    Invoke-Remote "$venvPy -m pip install --disable-pip-version-check -q pyserial" | Out-Host
}

if (!$SkipPreflight) {
    Write-Host "== 5) 板上预检（需要 FPGA 已烧链路位流 + 串口线插到 Pi） =="
    $rc = Invoke-Remote @"
cd $RemoteDir && UART_BRIDGE_TOOLS=$RemoteDir/uart_tools $venvPy src/preflight.py --json records/preflight.json 2>&1 | tail -20
"@
    $rc | Out-Host
}

Write-Host ""
Write-Host "部署完成。板上一键运行："
Write-Host "  ssh $Remote 'cd $RemoteDir && UART_BRIDGE_TOOLS=$RemoteDir/uart_tools $venvPy src/preflight.py'"
Write-Host "  ssh $Remote 'cd $RemoteDir && $venvPy src/camera_link_soak.py --port /dev/ttyUSB0 --duration 120 --json records/soak.json'"
Write-Host "  scp ${Remote}:${RemoteDir}/records/soak.json .\records\"

param([ValidateSet('Screenshot','Click','Keys')][string]$Action='Screenshot',[int]$X,[int]$Y,[string]$Keys)
Add-Type @'
using System;
using System.Runtime.InteropServices;
public class TWWindow {
 [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
 [DllImport("user32.dll")] public static extern bool SetProcessDPIAware();
 [DllImport("user32.dll")] public static extern IntPtr SetThreadDpiAwarenessContext(IntPtr c);
 [DllImport("user32.dll")] public static extern bool SetCursorPos(int x,int y);
 [DllImport("user32.dll")] public static extern bool SetPhysicalCursorPos(int x,int y);
 [DllImport("user32.dll")] public static extern void mouse_event(uint f,uint x,uint y,uint d,UIntPtr e);
}
'@
[TWWindow]::SetProcessDPIAware() | Out-Null
[TWWindow]::SetThreadDpiAwarenessContext([IntPtr](-4)) | Out-Null
if($Action -eq 'Screenshot') {
 Add-Type -AssemblyName System.Windows.Forms
 Add-Type -AssemblyName System.Drawing
 $bounds=[System.Windows.Forms.SystemInformation]::VirtualScreen
 Write-Output ('VirtualScreen=' + $bounds.ToString())
 [System.Windows.Forms.Screen]::AllScreens | ForEach-Object { Write-Output ($_.DeviceName + ' ' + $_.Bounds.ToString()) }
 $bmp=[System.Drawing.Bitmap]::new($bounds.Width,$bounds.Height)
 $g=[System.Drawing.Graphics]::FromImage($bmp)
 $g.CopyFromScreen($bounds.Location,[System.Drawing.Point]::Empty,$bounds.Size)
 $bmp.Save((Join-Path $PSScriptRoot 'current_window.png'))
 $g.Dispose();$bmp.Dispose()
} elseif($Action -eq 'Click') {
 [TWWindow]::SetForegroundWindow([IntPtr]1509426) | Out-Null
 Start-Sleep -Milliseconds 200
 [TWWindow]::SetPhysicalCursorPos($X,$Y) | Write-Output
 [TWWindow]::mouse_event(2,0,0,0,[UIntPtr]::Zero)
 [TWWindow]::mouse_event(4,0,0,0,[UIntPtr]::Zero)
} else {
 $twShell=New-Object -ComObject WScript.Shell
 $twShell.SendKeys($Keys)
}

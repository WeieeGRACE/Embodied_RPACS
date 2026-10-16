param([ValidateSet('Read','Connect','Disconnect')][string]$Action='Read')
$ErrorActionPreference='Stop'
Add-Type -AssemblyName UIAutomationClient
Add-Type -AssemblyName UIAutomationTypes
$root=[System.Windows.Automation.AutomationElement]::RootElement
$window=$root.FindFirst([System.Windows.Automation.TreeScope]::Children,
    [System.Windows.Automation.PropertyCondition]::new([System.Windows.Automation.AutomationElement]::NameProperty,'Fluent 串口助手'))
if(!$window){throw 'Fluent Serial Assistant window not found'}
if($Action -eq 'Connect') {
    Add-Type -TypeDefinition 'using System; using System.Runtime.InteropServices; public static class ImuSerialWindow { [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr h, int n); [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h); }'
    $handle=[IntPtr]$window.Current.NativeWindowHandle
    [void][ImuSerialWindow]::ShowWindow($handle,9)
    [void][ImuSerialWindow]::SetForegroundWindow($handle)
}
if($Action -ne 'Read') {
    $name=if($Action -eq 'Connect'){'连接'}else{'断开'}
    $button=$window.FindFirst([System.Windows.Automation.TreeScope]::Descendants,
        [System.Windows.Automation.PropertyCondition]::new([System.Windows.Automation.AutomationElement]::NameProperty,$name))
    if($button){$button.GetCurrentPattern([System.Windows.Automation.InvokePattern]::Pattern).Invoke()}
    Start-Sleep -Milliseconds 800
}
$elements=$window.FindAll([System.Windows.Automation.TreeScope]::Descendants,[System.Windows.Automation.Condition]::TrueCondition)
foreach($element in $elements) {
    $name=$element.Current.Name
    if($name -match '^COM14|^已连接$|^未连接$|^断开$|B/s$'){Write-Output $name}
    if($element.Current.ControlType -eq [System.Windows.Automation.ControlType]::Edit) {
        foreach($pattern in @([System.Windows.Automation.TextPattern]::Pattern,[System.Windows.Automation.ValuePattern]::Pattern)) {
            try {
                $p=$element.GetCurrentPattern($pattern)
                $text=if($pattern -eq [System.Windows.Automation.TextPattern]::Pattern){$p.DocumentRange.GetText(-1)}else{$p.Current.Value}
                if($text -match 'ID=') {
                    $text | Set-Content -LiteralPath "$PSScriptRoot/imu_fluent_capture.txt" -Encoding utf8
                    ($text -split "`n" | Select-Object -Last 4) | Write-Output
                    break
                }
            } catch {}
        }
    }
}
if($Action -eq 'Connect') {
    Add-Type -AssemblyName System.Drawing
    $rect=$window.Current.BoundingRectangle
    $bitmap=[System.Drawing.Bitmap]::new([int]$rect.Width,[int]$rect.Height)
    $graphics=[System.Drawing.Graphics]::FromImage($bitmap)
    try {
        $graphics.CopyFromScreen([int]$rect.Left,[int]$rect.Top,0,0,$bitmap.Size)
        $bitmap.Save("$PSScriptRoot/imu_fluent_connected.png")
    } finally { $graphics.Dispose(); $bitmap.Dispose() }
}

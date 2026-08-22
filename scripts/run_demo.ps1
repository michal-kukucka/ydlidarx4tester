[CmdletBinding()]
param(
    [string]$Port,
    [switch]$Simulate,
    [switch]$Headless,
    [int]$Frames = 0,
    [string]$Record,
    [ValidateRange(-180.0, 180.0)]
    [double]$ForwardAngle = -125.0,
    [ValidateRange(5.0, 360.0)]
    [double]$FieldOfView = 70.0,
    [switch]$ShowAll,
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$ExtraArgs
)

$ErrorActionPreference = "Stop"
$projectRoot = Split-Path -Parent $PSScriptRoot
$python = Join-Path $projectRoot ".venv\Scripts\python.exe"
$demo = Join-Path $projectRoot "demo\x4_visualizer.py"
$sdk = Join-Path $projectRoot "build-x4\ydlidar_sdk.dll"

if (-not (Test-Path $python) -or (-not $Simulate -and -not (Test-Path $sdk))) {
    throw "Project is not built yet. Run .\scripts\setup.ps1 first."
}

$demoArgs = @($demo)
if ($Port) { $demoArgs += @("--port", $Port) }
if ($Simulate) { $demoArgs += "--simulate" }
if ($Headless) { $demoArgs += "--headless" }
if ($Frames -gt 0) { $demoArgs += @("--frames", $Frames) }
if ($Record) { $demoArgs += @("--record", $Record) }
$invariantCulture = [System.Globalization.CultureInfo]::InvariantCulture
$demoArgs += @("--forward-angle", $ForwardAngle.ToString($invariantCulture))
$demoArgs += @("--field-of-view", $FieldOfView.ToString($invariantCulture))
if ($ShowAll) { $demoArgs += "--show-all" }
if ($ExtraArgs) { $demoArgs += $ExtraArgs }

Push-Location $projectRoot
try {
    & $python @demoArgs
    exit $LASTEXITCODE
} finally {
    Pop-Location
}

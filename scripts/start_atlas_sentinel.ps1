param(
  [string]$RepoPath = "C:\ATLAS_PUSH",
  [int]$IntervalSec = 90,
  [int]$HeartbeatSec = 30,
  [int]$CameraIndex = 0,
  [switch]$ConfigureAutoStart,
  [switch]$SpeakAlerts,
  [switch]$NoAutoHeal,
  [switch]$DryRun,
  [switch]$Foreground
)

$ErrorActionPreference = "Stop"

function Resolve-AtlasPython([string]$Root) {
  $venvPy = Join-Path $Root ".venv\Scripts\python.exe"
  $venvPyw = Join-Path $Root ".venv\Scripts\pythonw.exe"
  if (Test-Path $venvPy) {
    return @{
      python = $venvPy
      pythonw = if (Test-Path $venvPyw) { $venvPyw } else { $venvPy }
    }
  }
  return @{
    python = "python"
    pythonw = "pythonw"
  }
}

function Upsert-RunKey {
  param(
    [string]$Name,
    [string]$Value
  )
  $runKeyPath = "HKCU:\Software\Microsoft\Windows\CurrentVersion\Run"
  if (-not (Test-Path $runKeyPath)) {
    New-Item -Path $runKeyPath | Out-Null
  }
  New-ItemProperty -Path $runKeyPath -Name $Name -PropertyType String -Value $Value -Force | Out-Null
}

$sentinelScript = Join-Path $RepoPath "scripts\atlas_sentinel_loop.py"
if (-not (Test-Path $sentinelScript)) {
  throw "No existe script Sentinel: $sentinelScript"
}

$pyInfo = Resolve-AtlasPython -Root $RepoPath
$py = [string]$pyInfo.python
$pyBg = [string]$pyInfo.pythonw
$running = $false
try {
  $running = @(
    Get-CimInstance Win32_Process -ErrorAction SilentlyContinue | Where-Object {
      $_.Name -match "python" -and $_.CommandLine -match "atlas_sentinel_loop.py"
    }
  ).Count -gt 0
} catch {
  $running = $false
}

$args = @(
  $sentinelScript,
  "--interval-sec", "$IntervalSec",
  "--heartbeat-sec", "$HeartbeatSec"
)
if ($CameraIndex -gt 0) { $args += @("--camera-index", "$CameraIndex") }
if ($SpeakAlerts.IsPresent) { $args += "--speak-alerts" }
if ($NoAutoHeal.IsPresent) { $args += "--no-auto-heal" }
if ($DryRun.IsPresent) { $args += "--dry-run" }

Set-Location $RepoPath

if ($ConfigureAutoStart.IsPresent) {
  $launcher = "powershell -NoProfile -ExecutionPolicy Bypass -WindowStyle Hidden -File `"$PSScriptRoot\start_atlas_sentinel.ps1`" -RepoPath `"$RepoPath`" -IntervalSec $IntervalSec -HeartbeatSec $HeartbeatSec"
  if ($CameraIndex -gt 0) { $launcher += " -CameraIndex $CameraIndex" }
  if ($SpeakAlerts.IsPresent) { $launcher += " -SpeakAlerts" }
  if ($NoAutoHeal.IsPresent) { $launcher += " -NoAutoHeal" }
  if ($DryRun.IsPresent) { $launcher += " -DryRun" }
  Upsert-RunKey -Name "ATLAS_SENTINEL" -Value $launcher
}

if ($Foreground.IsPresent) {
  & $py @args
  exit $LASTEXITCODE
}

if ($running) {
  Write-Host "ATLAS Sentinel ya esta ejecutandose."
  if ($ConfigureAutoStart.IsPresent) {
    Write-Host "Autoarranque configurado en HKCU Run: ATLAS_SENTINEL"
  }
  exit 0
}

Start-Process -FilePath $pyBg -ArgumentList $args -WorkingDirectory $RepoPath -WindowStyle Hidden | Out-Null
Write-Host "ATLAS Sentinel iniciado en segundo plano."
Write-Host "Logs: $RepoPath\logs\sentinel"
if ($ConfigureAutoStart.IsPresent) {
  Write-Host "Autoarranque configurado en HKCU Run: ATLAS_SENTINEL"
}

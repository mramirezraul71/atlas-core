param(
  [string]$RepoPath = "C:\ATLAS_PUSH",
  [switch]$EnsureAutoStart,
  [switch]$EnsureCloudflareTunnel,
  [switch]$EnsureSentinel,
  [int]$SentinelCameraIndex = 0
)

$ErrorActionPreference = "Stop"
Set-Location $RepoPath

function Test-HttpOk([string]$Url, [int]$TimeoutSec = 10) {
  try {
    $r = Invoke-WebRequest -Uri $Url -UseBasicParsing -TimeoutSec $TimeoutSec
    return ($r.StatusCode -eq 200)
  } catch {
    return $false
  }
}

function Test-HttpOkWithRetry(
  [string]$Url,
  [int]$TimeoutSec = 10,
  [int]$Retries = 3,
  [int]$DelaySec = 2
) {
  for ($i = 1; $i -le $Retries; $i++) {
    if (Test-HttpOk -Url $Url -TimeoutSec $TimeoutSec) {
      return $true
    }
    if ($i -lt $Retries) {
      Start-Sleep -Seconds $DelaySec
    }
  }
  return $false
}

function Get-JsonOrNull([string]$Url, [int]$TimeoutSec = 8) {
  try {
    $r = Invoke-WebRequest -Uri $Url -UseBasicParsing -TimeoutSec $TimeoutSec
    if ($r.StatusCode -eq 200 -and $r.Content) {
      return ($r.Content | ConvertFrom-Json)
    }
  } catch {}
  return $null
}

function Test-WatchdogAutostartConfigured([string]$TaskName) {
  $taskExists = $false
  try {
    schtasks /Query /TN $TaskName 2>$null | Out-Null
    if ($LASTEXITCODE -eq 0) { $taskExists = $true }
  } catch {}

  $runKeyExists = $false
  try {
    $rk = Get-ItemProperty -Path "HKCU:\Software\Microsoft\Windows\CurrentVersion\Run" -Name $TaskName -ErrorAction Stop
    if ($rk.$TaskName) { $runKeyExists = $true }
  } catch {}

  return @{ task = $taskExists; runkey = $runKeyExists }
}

function Test-RunKeyValue([string]$Name) {
  try {
    $rk = Get-ItemProperty -Path "HKCU:\Software\Microsoft\Windows\CurrentVersion\Run" -Name $Name -ErrorAction Stop
    $v = "" + $rk.$Name
    return -not [string]::IsNullOrWhiteSpace($v)
  } catch {
    return $false
  }
}

Write-Host "== ATLAS Operational Boot ==" -ForegroundColor Cyan

# 1) ROBOT only (NEXUS body en 8000 desactivado - dashboard integrado en 8791/nexus)
$py = Join-Path $RepoPath ".venv\Scripts\python.exe"
if (!(Test-Path $py)) { throw "No existe python en .venv: $py" }
& $py (Join-Path $RepoPath "scripts\start_nexus_services.py") --robot-only | Out-Host

# 2) PUSH
if (-not (Test-HttpOkWithRetry "http://127.0.0.1:8791/health" 8 2 2)) {
  Write-Host "PUSH down -> restart clean..." -ForegroundColor Yellow
  & powershell -NoProfile -ExecutionPolicy Bypass -File (Join-Path $RepoPath "scripts\restart_push_from_api.ps1") | Out-Host
}

# 3) Watchdog (single instance via lock/mutex)
$wdCmd = "push_autorevive_watchdog.ps1"
$already = Get-CimInstance Win32_Process | Where-Object {
  $_.Name -match "powershell" -and $_.CommandLine -match [regex]::Escape($wdCmd)
}
if (-not $already) {
  Write-Host "Starting PUSH watchdog..." -ForegroundColor Green
  Start-Process -FilePath "powershell.exe" -ArgumentList @(
    "-NoProfile",
    "-ExecutionPolicy", "Bypass",
    "-WindowStyle", "Hidden",
    "-File", (Join-Path $RepoPath "scripts\push_autorevive_watchdog.ps1"),
    "-HealthUrl", "http://127.0.0.1:8791/health",
    "-IntervalSec", "30",
    "-FailThreshold", "8",
    "-RestartCooldownSec", "300",
    "-HealthTimeoutSec", "45",
    "-ProbeTimeoutSec", "12",
    "-StartupGraceSec", "180"
  ) | Out-Null
} else {
  Write-Host "Watchdog already running." -ForegroundColor Yellow
}

if ($EnsureAutoStart.IsPresent) {
  $taskName = "ATLAS_PUSH_Autorevive_Watchdog"
  $auto = Test-WatchdogAutostartConfigured -TaskName $taskName
  $runKeyPath = "HKCU:\Software\Microsoft\Windows\CurrentVersion\Run"
  $runCmd = "powershell -NoProfile -ExecutionPolicy Bypass -WindowStyle Hidden -File `"$RepoPath\scripts\run_push_autorevive_watchdog.ps1`" -HealthUrl `"http://127.0.0.1:8791/health`" -IntervalSec 30 -FailThreshold 8 -RestartCooldownSec 300"
  try {
    if (-not (Test-Path $runKeyPath)) {
      New-Item -Path $runKeyPath | Out-Null
    }
    New-ItemProperty -Path $runKeyPath -Name $taskName -PropertyType String -Value $runCmd -Force | Out-Null
  } catch {
    Write-Warning "No se pudo escribir HKCU Run para watchdog: $($_.Exception.Message)"
  }
  if (-not $auto.task -and -not $auto.runkey) {
    Write-Host "Configuring watchdog autostart..." -ForegroundColor Yellow
    & powershell -NoProfile -ExecutionPolicy Bypass -File (Join-Path $RepoPath "scripts\register_push_autorevive_task.ps1") -TaskName $taskName -RunLevel LIMITED | Out-Host
  }
}

if ($EnsureCloudflareTunnel.IsPresent) {
  Write-Host "Ensuring Cloudflare named tunnel..." -ForegroundColor Yellow
  & powershell -NoProfile -ExecutionPolicy Bypass -File (Join-Path $RepoPath "scripts\start_cloudflare_named_tunnel.ps1") -StopQuickTunnels -ConfigureAutoStart | Out-Host
}

if ($EnsureSentinel.IsPresent) {
  $sentinelCmd = "atlas_sentinel_loop.py"
  $sentinelRunning = Get-CimInstance Win32_Process | Where-Object {
    $_.Name -match "python" -and $_.CommandLine -match [regex]::Escape($sentinelCmd)
  }
  if (-not $sentinelRunning) {
    Write-Host "Starting ATLAS Sentinel..." -ForegroundColor Green
    $sentArgs = @(
      "-NoProfile",
      "-ExecutionPolicy", "Bypass",
      "-File", (Join-Path $RepoPath "scripts\start_atlas_sentinel.ps1"),
      "-CameraIndex", "$SentinelCameraIndex"
    )
    if ($EnsureAutoStart.IsPresent) {
      $sentArgs += "-ConfigureAutoStart"
    }
    & powershell @sentArgs | Out-Host
  } else {
    Write-Host "ATLAS Sentinel already running." -ForegroundColor Yellow
    if ($EnsureAutoStart.IsPresent) {
      & powershell -NoProfile -ExecutionPolicy Bypass -File (Join-Path $RepoPath "scripts\start_atlas_sentinel.ps1") -CameraIndex $SentinelCameraIndex -ConfigureAutoStart | Out-Host
    }
  }
}

Start-Sleep -Seconds 2
Write-Host ""
Write-Host "== Health ==" -ForegroundColor Cyan
Write-Host ("PUSH 8791:  " + (Test-HttpOkWithRetry "http://127.0.0.1:8791/health" 12 3 2))
Write-Host ("NEXUS 8000: " + (Test-HttpOkWithRetry "http://127.0.0.1:8000/health" 8 2 1))
Write-Host ("ROBOT 8002: " + (Test-HttpOkWithRetry "http://127.0.0.1:8002/health" 8 2 1))

$screen = Get-JsonOrNull -Url "http://127.0.0.1:8791/screen/status"
if ($screen -and $screen.ok -and $screen.data) {
  Write-Host ("SCREEN: enabled=" + [bool]$screen.data.enabled + " deps_missing=" + (@($screen.data.missing_deps).Count))
} else {
  Write-Host "SCREEN: unavailable" -ForegroundColor Yellow
}

$wa = Get-JsonOrNull -Url "http://127.0.0.1:8791/api/comms/whatsapp/status"
if ($wa) {
  $waReady = [bool]$wa.ready
  $waStatus = ""
  try { $waStatus = [string]$wa.provider_status.status } catch {}
  Write-Host ("WHATSAPP: ready=" + $waReady + " status=" + $waStatus)
  if (-not $waReady) {
    Write-Host "WHATSAPP ACTION: escanea QR desde /api/comms/whatsapp/qr" -ForegroundColor Yellow
  }
} else {
  Write-Host "WHATSAPP: unavailable" -ForegroundColor Yellow
}

if ($EnsureAutoStart.IsPresent) {
  $auto = Test-WatchdogAutostartConfigured -TaskName "ATLAS_PUSH_Autorevive_Watchdog"
  Write-Host ("WATCHDOG_AUTOSTART: task=" + $auto.task + " runkey=" + $auto.runkey)
  Write-Host ("CLOUDFLARED_AUTOSTART: runkey=" + (Test-RunKeyValue "ATLAS_CLOUDFLARED_NAMED"))
  Write-Host ("SENTINEL_AUTOSTART: runkey=" + (Test-RunKeyValue "ATLAS_SENTINEL"))
}

$sentinelActive = $false
try {
  $sentinelActive = @(Get-CimInstance Win32_Process | Where-Object {
    $_.Name -match "python" -and $_.CommandLine -match "atlas_sentinel_loop.py"
  }).Count -gt 0
} catch {}
Write-Host ("SENTINEL: running=" + $sentinelActive)

Write-Host "Dashboard: http://127.0.0.1:8791/ui" -ForegroundColor Green

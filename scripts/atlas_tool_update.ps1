param(
  [Parameter(Mandatory = $true)][string]$Tool,
  [string]$RepoRoot = $(Resolve-Path (Join-Path $PSScriptRoot "..")).Path
)

$ErrorActionPreference = "Stop"

$logFile = Join-Path $RepoRoot "logs\snapshot_safe_diagnostic.log"
New-Item -ItemType Directory -Path (Split-Path $logFile -Parent) -Force | Out-Null

function Write-Snapshot([string]$line) {
  $ts = Get-Date -Format "yyyy-MM-ddTHH:mm:ssK"
  "$ts TOOLS_UPDATE tool=$Tool $line" | Out-File -FilePath $logFile -Append -Encoding utf8
}

function Get-PythonExe {
  $candidates = @(
    (Join-Path $RepoRoot "venv\Scripts\python.exe"),
    (Join-Path $RepoRoot ".venv\Scripts\python.exe")
  )
  foreach ($c in $candidates) { if (Test-Path $c) { return $c } }
  return "python"
}

function Invoke-Cmd {
  param(
    [string]$Name,
    [string]$Exe,
    [string[]]$ArgList = @(),
    [string]$WorkDir = $RepoRoot
  )
  Push-Location $WorkDir
  try {
    & $Exe @ArgList 2>&1 | ForEach-Object { Write-Host $_ }
    $rc = $LASTEXITCODE
    if ($rc -ne 0) {
      throw "$Name failed (rc=$rc)"
    }
  }
  finally {
    Pop-Location
  }
}

function Invoke-WingetUpgradeWithFallback {
  param(
    [string]$Name,
    [string]$PackageId
  )
  $common = @("--id",$PackageId,"--source","winget","--disable-interactivity","--silent","--accept-package-agreements","--accept-source-agreements")
  $commonNoSource = @("--id",$PackageId,"--disable-interactivity","--silent","--accept-package-agreements","--accept-source-agreements")
  $attempts = @(
    @("upgrade") + $common,
    @("upgrade","-e") + $common,
    @("install","-e") + $common,
    @("upgrade") + $commonNoSource,
    @("install","-e") + $commonNoSource
  )
  $lastErr = ""
  foreach ($a in $attempts) {
    $verb = $a[0]
    $args = $a[1..($a.Length-1)]
    $tmpOut = [System.IO.Path]::GetTempFileName()
    $tmpErr = [System.IO.Path]::GetTempFileName()
    $rc = 0
    $raw = ""
    try {
      $argLine = @($verb) + $args
      $p = Start-Process -FilePath "winget" -ArgumentList $argLine -NoNewWindow -PassThru -RedirectStandardOutput $tmpOut -RedirectStandardError $tmpErr
      $finished = $p.WaitForExit(90000)
      if (-not $finished) {
        try { $p.Kill() } catch {}
        $rc = 124
        $raw = "winget_timeout_90s"
      } else {
        $rc = $p.ExitCode
      }
      if (Test-Path $tmpOut) {
        $captured = Get-Content -Path $tmpOut -Raw -ErrorAction SilentlyContinue
        if ($captured) { $raw = ($raw + "`n" + $captured).Trim() }
      }
      if (Test-Path $tmpErr) {
        $capturedErr = Get-Content -Path $tmpErr -Raw -ErrorAction SilentlyContinue
        if ($capturedErr) { $raw = ($raw + "`n" + $capturedErr).Trim() }
      }
    } finally {
      Remove-Item -Path $tmpOut -Force -ErrorAction SilentlyContinue
      Remove-Item -Path $tmpErr -Force -ErrorAction SilentlyContinue
    }
    if ($rc -eq 0) { return @{ ok = $true; reason = "updated_or_installed" } }
    $txt = ($raw | Out-String)
    $low = $txt.ToLowerInvariant()
    $lastErr = ($txt -split "`r?`n" | Where-Object { $_.Trim() } | Select-Object -Last 1)
    if (
      $low.Contains("no applicable update found") -or
      $low.Contains("no newer package versions are available") -or
      $low.Contains("no package found matching input criteria") -or
      $low.Contains("no se encontraron actualizaciones") -or
      $low.Contains("ya est") -or
      $low.Contains("already installed")
    ) {
      return @{ ok = $true; reason = "already_up_to_date" }
    }
    if ($low.Contains("no installed package found matching input criteria") -or $low.Contains("no se encontr")) {
      continue
    }
  }
  throw "$Name failed (all fallback attempts): $lastErr"
}

function Invoke-WingetUpgradeAnyId {
  param(
    [string]$Name,
    [string[]]$PackageIds
  )
  $errors = @()
  foreach ($pkgId in @($PackageIds | Where-Object { $_ -and $_.Trim() })) {
    try {
      $res = Invoke-WingetUpgradeWithFallback -Name "${Name}:$pkgId" -PackageId $pkgId
      if ($res -and $res.ok) { return }
    } catch {
      $errors += @($_.Exception.Message)
    }
  }
  $tail = if ($errors.Count -gt 0) { ($errors -join " | ") } else { "no_candidate_ids" }
  throw "$Name failed (ids exhausted): $tail"
}

$steps = @()
$ok = $true
$py = Get-PythonExe
$snapshot = Join-Path $RepoRoot "scripts\atlas_snapshot_safe.ps1"
$actuatorDir = Join-Path $RepoRoot "tools\atlas_actuators"
$protectedBranches = @("main","master","release","prod","production")
$allowProtectedRaw = if ($env:ATLAS_ALLOW_TOOL_UPDATE_ON_PROTECTED) { $env:ATLAS_ALLOW_TOOL_UPDATE_ON_PROTECTED } else { "0" }
$allowProtectedUpdates = ($allowProtectedRaw.ToLowerInvariant() -notin @("0","false","no","off",""))
$currentBranch = "unknown"

try {
  $b = (& git -C $RepoRoot rev-parse --abbrev-ref HEAD 2>$null)
  if ($LASTEXITCODE -eq 0 -and $b) { $currentBranch = $b.Trim() }
}
catch {}

$isProtectedBranch = $protectedBranches -contains $currentBranch

Write-Snapshot "START"

if ($isProtectedBranch -and -not $allowProtectedUpdates) {
  $ok = $false
  $steps += @{
    step = "guardrail_branch"
    ok = $false
    branch = $currentBranch
    error = "updates_blocked_on_protected_branch"
  }
  Write-Snapshot "TASK_FAIL name=ATLAS_ToolUpdate tool=$Tool error=protected_branch branch=$currentBranch"
}

if ($ok) {
  try {
    Invoke-Cmd -Name "snapshot_safe" -Exe "powershell" -ArgList @("-NoProfile","-ExecutionPolicy","Bypass","-File",$snapshot,"-RepoRoot",$RepoRoot)
    $steps += @{ step = "snapshot_safe"; ok = $true }
  }
  catch {
    $steps += @{ step = "snapshot_safe"; ok = $false; error = $_.Exception.Message }
    $ok = $false
  }
}

if ($ok) {
  try {
    switch ($Tool.ToLowerInvariant()) {
      "ccxt" {
        Invoke-Cmd -Name "pip_upgrade_ccxt" -Exe $py -ArgList @("-m","pip","install","--upgrade","ccxt")
      }
      "playwright_py" {
        Invoke-Cmd -Name "pip_upgrade_playwright" -Exe $py -ArgList @("-m","pip","install","--upgrade","playwright")
        Invoke-Cmd -Name "playwright_install_chromium" -Exe $py -ArgList @("-m","playwright","install","chromium")
      }
      "puppeteer" {
        Invoke-Cmd -Name "npm_upgrade_puppeteer" -Exe "npm.cmd" -ArgList @("install","puppeteer@latest") -WorkDir $actuatorDir
      }
      "playwright_node" {
        Invoke-Cmd -Name "npm_upgrade_playwright" -Exe "npm.cmd" -ArgList @("install","playwright@latest") -WorkDir $actuatorDir
        Invoke-Cmd -Name "npx_install_chromium" -Exe "npx.cmd" -ArgList @("playwright","install","chromium") -WorkDir $actuatorDir
      }
      "ollama" {
        Invoke-WingetUpgradeWithFallback -Name "winget_upgrade_ollama" -PackageId "Ollama.Ollama"
      }
      "cloudflared" {
        Invoke-WingetUpgradeAnyId -Name "winget_upgrade_cloudflared" -PackageIds @(
          "Cloudflare.cloudflared",
          "cloudflare.cloudflared"
        )
      }
      "git" {
        Invoke-WingetUpgradeWithFallback -Name "winget_upgrade_git" -PackageId "Git.Git"
      }
      "node" {
        Invoke-WingetUpgradeAnyId -Name "winget_upgrade_node" -PackageIds @(
          "OpenJS.NodeJS",
          "OpenJS.NodeJS.LTS"
        )
      }
      "tailscale" {
        Invoke-WingetUpgradeWithFallback -Name "winget_upgrade_tailscale" -PackageId "Tailscale.Tailscale"
      }
      # ── Trading / ML stack — pip upgrade ────────────────────────────
      "pandas" {
        Invoke-Cmd -Name "pip_upgrade_pandas" -Exe $py -ArgList @("-m","pip","install","--upgrade","pandas")
      }
      "numpy" {
        Invoke-Cmd -Name "pip_upgrade_numpy" -Exe $py -ArgList @("-m","pip","install","--upgrade","numpy")
      }
      "scipy" {
        Invoke-Cmd -Name "pip_upgrade_scipy" -Exe $py -ArgList @("-m","pip","install","--upgrade","scipy")
      }
      "scikit-learn" {
        Invoke-Cmd -Name "pip_upgrade_sklearn" -Exe $py -ArgList @("-m","pip","install","--upgrade","scikit-learn")
      }
      "xgboost" {
        Invoke-Cmd -Name "pip_upgrade_xgboost" -Exe $py -ArgList @("-m","pip","install","--upgrade","xgboost")
      }
      "lightgbm" {
        Invoke-Cmd -Name "pip_upgrade_lightgbm" -Exe $py -ArgList @("-m","pip","install","--upgrade","lightgbm")
      }
      "statsmodels" {
        Invoke-Cmd -Name "pip_upgrade_statsmodels" -Exe $py -ArgList @("-m","pip","install","--upgrade","statsmodels")
      }
      "ta" {
        Invoke-Cmd -Name "pip_upgrade_ta" -Exe $py -ArgList @("-m","pip","install","--upgrade","ta")
      }
      "yfinance" {
        Invoke-Cmd -Name "pip_upgrade_yfinance" -Exe $py -ArgList @("-m","pip","install","--upgrade","yfinance")
      }
      "optuna" {
        Invoke-Cmd -Name "pip_upgrade_optuna" -Exe $py -ArgList @("-m","pip","install","--upgrade","optuna")
      }
      "stable-baselines3" {
        Invoke-Cmd -Name "pip_upgrade_sb3" -Exe $py -ArgList @("-m","pip","install","--upgrade","stable-baselines3")
      }
      "backtrader" {
        Invoke-Cmd -Name "pip_upgrade_backtrader" -Exe $py -ArgList @("-m","pip","install","--upgrade","backtrader")
      }
      "gymnasium" {
        Invoke-Cmd -Name "pip_upgrade_gymnasium" -Exe $py -ArgList @("-m","pip","install","--upgrade","gymnasium")
      }
      "easyocr" {
        Invoke-Cmd -Name "pip_upgrade_easyocr" -Exe $py -ArgList @("-m","pip","install","--upgrade","easyocr")
      }
      # ── Core framework — pip upgrade ─────────────────────────────────
      "fastapi" {
        Invoke-Cmd -Name "pip_upgrade_fastapi" -Exe $py -ArgList @("-m","pip","install","--upgrade","fastapi")
      }
      "uvicorn" {
        Invoke-Cmd -Name "pip_upgrade_uvicorn" -Exe $py -ArgList @("-m","pip","install","--upgrade","uvicorn[standard]")
      }
      "httpx" {
        Invoke-Cmd -Name "pip_upgrade_httpx" -Exe $py -ArgList @("-m","pip","install","--upgrade","httpx")
      }
      "sqlalchemy" {
        Invoke-Cmd -Name "pip_upgrade_sqlalchemy" -Exe $py -ArgList @("-m","pip","install","--upgrade","SQLAlchemy")
      }
      "pydantic" {
        Invoke-Cmd -Name "pip_upgrade_pydantic" -Exe $py -ArgList @("-m","pip","install","--upgrade","pydantic")
      }
      # ── Software externo ─────────────────────────────────────────────
      "prometheus" {
        $promScript = Join-Path $RepoRoot "tools\prometheus\start_monitoring.ps1"
        if (Test-Path $promScript) {
          Write-Host "Prometheus: re-ejecutando start_monitoring para actualizar binarios..."
          Invoke-Cmd -Name "prometheus_update" -Exe "powershell" -ArgList @("-NoProfile","-ExecutionPolicy","Bypass","-File",$promScript)
        } else {
          throw "Script start_monitoring.ps1 no encontrado en tools\prometheus\"
        }
      }
      "grafana" {
        Invoke-WingetUpgradeAnyId -Name "winget_upgrade_grafana" -PackageIds @(
          "GrafanaLabs.Grafana",
          "GrafanaLabs.Grafana.OSS"
        )
      }
      "docker" {
        Invoke-WingetUpgradeWithFallback -Name "winget_upgrade_docker" -PackageId "Docker.DockerDesktop"
      }
      default {
        throw "Tool no soportada para update: $Tool"
      }
    }
    $steps += @{ step = "update_tool"; ok = $true; tool = $Tool }
    Write-Snapshot "TASK_OK name=ATLAS_ToolUpdate tool=$Tool"
  }
  catch {
    $steps += @{ step = "update_tool"; ok = $false; tool = $Tool; error = $_.Exception.Message }
    $ok = $false
    Write-Snapshot "TASK_FAIL name=ATLAS_ToolUpdate tool=$Tool error=$($_.Exception.Message)"
  }
}
else {
  Write-Snapshot "TASK_FAIL name=ATLAS_ToolUpdate tool=$Tool error=snapshot_failed"
}

$result = @{
  ok = $ok
  tool = $Tool
  branch = $currentBranch
  blocked_protected_branch = ($isProtectedBranch -and -not $allowProtectedUpdates)
  allow_protected_updates = $allowProtectedUpdates
  updated_at = (Get-Date).ToString("o")
  steps = $steps
}

$result | ConvertTo-Json -Depth 8 -Compress

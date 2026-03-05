param(
  [string]$RepoRoot = $(Resolve-Path (Join-Path $PSScriptRoot "..")).Path
)

$ErrorActionPreference = "Stop"

$logFile = Join-Path $RepoRoot "logs\snapshot_safe_diagnostic.log"
New-Item -ItemType Directory -Path (Split-Path $logFile -Parent) -Force | Out-Null

function Write-Snapshot([string]$line) {
  $ts = Get-Date -Format "yyyy-MM-ddTHH:mm:ssK"
  "$ts TOOLS_BOOTSTRAP $line" | Out-File -FilePath $logFile -Append -Encoding utf8
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
    if ($rc -ne 0) { throw "$Name failed (rc=$rc)" }
  }
  finally {
    Pop-Location
  }
}

function Get-PythonExe {
  $venvPy = Join-Path $RepoRoot ".venv\Scripts\python.exe"
  if (Test-Path $venvPy) { return $venvPy }
  return "python"
}

$steps = @()
$ok = $true
$snapshot = Join-Path $RepoRoot "scripts\atlas_snapshot_safe.ps1"
$tailscaleSetup = Join-Path $RepoRoot "scripts\gateway\12_gateway_setup_tailscale.ps1"
$venvActivate = Join-Path $RepoRoot ".venv\Scripts\Activate.ps1"
$actuatorDir = Join-Path $RepoRoot "tools\atlas_actuators"

Write-Snapshot "START"

try {
  Invoke-Cmd -Name "snapshot_safe" -Exe "powershell" -ArgList @("-NoProfile","-ExecutionPolicy","Bypass","-File",$snapshot,"-RepoRoot",$RepoRoot)
  $steps += @{ step = "snapshot_safe"; ok = $true }
}
catch {
  $steps += @{ step = "snapshot_safe"; ok = $false; error = $_.Exception.Message }
  $ok = $false
}

if ($ok) {
  try {
    if (-not (Test-Path $venvActivate)) {
      Invoke-Cmd -Name "create_venv" -Exe "python" -ArgList @("-m","venv",".venv")
      $steps += @{ step = "create_venv"; ok = $true }
    }

    $py = Get-PythonExe
    Invoke-Cmd -Name "pip_upgrade" -Exe $py -ArgList @("-m","pip","install","--upgrade","pip")
    Invoke-Cmd -Name "install_python_deps" -Exe $py -ArgList @("-m","pip","install","ccxt","playwright")
    Invoke-Cmd -Name "playwright_chromium" -Exe $py -ArgList @("-m","playwright","install","chromium")
    $steps += @{ step = "python_deps"; ok = $true; deps = @("ccxt","playwright","sqlite_builtin") }

    Invoke-Cmd -Name "npm_install_actuators" -Exe "npm.cmd" -ArgList @("install") -WorkDir $actuatorDir
    Invoke-Cmd -Name "npm_install_browser_deps" -Exe "npm.cmd" -ArgList @("install","playwright","puppeteer") -WorkDir $actuatorDir
    $steps += @{ step = "node_deps"; ok = $true; deps = @("playwright","puppeteer") }

    if (Test-Path $tailscaleSetup) {
      Invoke-Cmd -Name "tailscale_setup_check" -Exe "powershell" -ArgList @("-NoProfile","-ExecutionPolicy","Bypass","-File",$tailscaleSetup)
      $steps += @{ step = "tailscale_setup_check"; ok = $true }
    }
    else {
      $steps += @{ step = "tailscale_setup_check"; ok = $false; error = "missing_script" }
    }

    Invoke-Cmd -Name "sqlite_check" -Exe $py -ArgList @("-c","import sqlite3; print(sqlite3.sqlite_version)")
    $steps += @{ step = "sqlite_check"; ok = $true }

    $tokenReady = [bool]($env:ATLAS_CENTRAL_CORE) -or [bool]($env:APPROVALS_CHAIN_SECRET)
    if ($tokenReady) {
      $steps += @{ step = "clawdbot_core_token"; ok = $true; source = "env" }
      Write-Snapshot "TOKEN_READY source=env"
    }
    else {
      $steps += @{ step = "clawdbot_core_token"; ok = $false; error = "ATLAS_CENTRAL_CORE/APPROVALS_CHAIN_SECRET missing" }
      Write-Snapshot "TOKEN_WARN missing_core_token"
    }

    Write-Snapshot "TASK_OK name=ATLAS_ToolsBootstrap"
  }
  catch {
    $ok = $false
    $steps += @{ step = "bootstrap"; ok = $false; error = $_.Exception.Message }
    Write-Snapshot "TASK_FAIL name=ATLAS_ToolsBootstrap error=$($_.Exception.Message)"
  }
}
else {
  Write-Snapshot "TASK_FAIL name=ATLAS_ToolsBootstrap error=snapshot_failed"
}

$result = @{
  ok = $ok
  updated_at = (Get-Date).ToString("o")
  steps = $steps
}

$result | ConvertTo-Json -Depth 8 -Compress

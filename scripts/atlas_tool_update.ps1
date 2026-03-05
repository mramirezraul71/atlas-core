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
  $venvPy = Join-Path $RepoRoot ".venv\Scripts\python.exe"
  if (Test-Path $venvPy) { return $venvPy }
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

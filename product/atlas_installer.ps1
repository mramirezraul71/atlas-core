# Atlas Product Pack - One-command installer (idempotent)
# Requires: PowerShell 5.1+, optional Admin for Program Files
param(
    [string]$TargetDir = "",
    [int]$Port = 8791,
    [switch]$SkipService,
    [switch]$Force
)
$ErrorActionPreference = "Stop"
$ProgressPreference = "SilentlyContinue"

$installRoot = if ($TargetDir) { $TargetDir } else {
    $isAdmin = ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
    if ($isAdmin) { "C:\Program Files\AtlasPush" } else { "C:\AtlasPush" }
}
$installRoot = $installRoot.TrimEnd("\")

# 1) Verify Python 3.11, Git, venv
Write-Host "[1/9] Verifying prerequisites..." -ForegroundColor Cyan
$py = Get-Command python -ErrorAction SilentlyContinue
if (-not $py) { $py = Get-Command python3 -ErrorAction SilentlyContinue }
if (-not $py) { Write-Host "ERROR: Python not found. Install Python 3.11." -ForegroundColor Red; exit 1 }
$pyVer = & $py.Path --version 2>&1
if ($pyVer -notmatch "3\.11") { Write-Host "WARN: Python 3.11 recommended. Found: $pyVer" -ForegroundColor Yellow }
$git = Get-Command git -ErrorAction SilentlyContinue
if (-not $git) { Write-Host "ERROR: Git not found." -ForegroundColor Red; exit 1 }
Write-Host "  Python: $pyVer | Git: $($git.Version)" -ForegroundColor Green

# 2) Create folder
Write-Host "[2/9] Target: $installRoot" -ForegroundColor Cyan
New-Item -ItemType Directory -Force -Path $installRoot | Out-Null
$sourceRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
if (-not (Test-Path (Join-Path $sourceRoot "atlas_adapter"))) {
    Write-Host "ERROR: Run installer from product/ inside ATLAS_PUSH repo." -ForegroundColor Red
    exit 1
}

# 3) Copy code (robocopy to avoid overwrite prompts)
Write-Host "[3/9] Copying code..." -ForegroundColor Cyan
$exclude = @("__pycache__", ".git", "node_modules", "logs", "*.pyc", ".venv", "venv")
robocopy $sourceRoot $installRoot /E /NFL /NDL /NJH /NJS /nc /ns /np /XD .git __pycache__ node_modules .venv venv logs /XF *.pyc 2>$null
if ($LASTEXITCODE -gt 7) { Write-Host "ERROR: Copy failed." -ForegroundColor Red; exit 1 }
Write-Host "  Done." -ForegroundColor Green

# 4) Create venv
$venvPath = Join-Path $installRoot ".venv"
Write-Host "[4/9] Creating venv..." -ForegroundColor Cyan
if (-not (Test-Path $venvPath) -or $Force) {
    & $py.Path -m venv $venvPath
    if ($LASTEXITCODE -ne 0) { Write-Host "ERROR: venv failed." -ForegroundColor Red; exit 1 }
}
$pip = Join-Path $venvPath "Scripts\pip.exe"
$pythonExe = Join-Path $venvPath "Scripts\python.exe"
Write-Host "  Venv: $venvPath" -ForegroundColor Green

# 5) Install deps
Write-Host "[5/9] Installing dependencies..." -ForegroundColor Cyan
$reqFile = Join-Path $installRoot "requirements.txt"
if (-not (Test-Path $reqFile)) {
    @("fastapi", "uvicorn", "httpx", "pydantic", "python-dotenv") | Set-Content $reqFile
}
& $pip install -q -r $reqFile 2>$null
& $pip install -q fastapi uvicorn httpx pydantic python-dotenv 2>$null
Write-Host "  Done." -ForegroundColor Green

# 6) Config template
$configDir = Join-Path $installRoot "config"
$envFile = Join-Path $configDir "atlas.env"
New-Item -ItemType Directory -Force -Path $configDir | Out-Null
if (-not (Test-Path $envFile)) {
    $example = Join-Path $configDir "atlas.env.example"
    if (Test-Path $example) { Copy-Item $example $envFile }
    Add-Content $envFile "`nPOLICY_ALLOWED_PATHS=$installRoot"
    Add-Content $envFile "SERVICE_PORT=$Port"
    Add-Content $envFile "ATLAS_REPO_PATH=$installRoot"
    Write-Host "[6/9] Config template: $envFile" -ForegroundColor Cyan
} else {
    Write-Host "[6/9] Config exists." -ForegroundColor Cyan
}

# 7) Smoke (quick)
Write-Host "[7/9] Starting API briefly for smoke..." -ForegroundColor Cyan
$env:POLICY_ALLOWED_PATHS = $installRoot
$env:ATLAS_REPO_PATH = $installRoot
$env:SERVICE_PORT = $Port
$job = Start-Job -ScriptBlock {
    Set-Location $using:installRoot
    & $using:pythonExe -m uvicorn atlas_adapter.atlas_http_api:app --host 127.0.0.1 --port $using:Port
} -ErrorAction SilentlyContinue
Start-Sleep -Seconds 8
try {
    $r = Invoke-RestMethod -Uri "http://127.0.0.1:$Port/version" -TimeoutSec 5 -ErrorAction Stop
    Write-Host "  Version: $($r.version)" -ForegroundColor Green
} catch {
    Write-Host "  WARN: Smoke check skipped (no response)." -ForegroundColor Yellow
}
Stop-Job $job -ErrorAction SilentlyContinue
Remove-Job $job -ErrorAction SilentlyContinue

# 8) Windows service
if (-not $SkipService) {
    Write-Host "[8/9] Service registration..." -ForegroundColor Cyan
    $svcName = "ATLAS_PUSH"
    $existing = Get-Service -Name $svcName -ErrorAction SilentlyContinue
    if ($existing) {
        Write-Host "  Service already exists." -ForegroundColor Green
    } else {
        Write-Host "  To install service run as Admin: sc create $svcName binPath= `"$pythonExe -m uvicorn atlas_adapter.atlas_http_api:app --host 127.0.0.1 --port $Port`" start= auto" -ForegroundColor Yellow
    }
} else {
    Write-Host "[8/9] Skip service." -ForegroundColor Cyan
}

# 9) Summary
Write-Host "[9/9] Summary" -ForegroundColor Cyan
Write-Host "  Install root: $installRoot" -ForegroundColor White
Write-Host "  URL: http://127.0.0.1:$Port" -ForegroundColor White
Write-Host "  UI:  http://127.0.0.1:$Port/ui" -ForegroundColor White
Write-Host "  Config: $envFile" -ForegroundColor White
Write-Host "  To start: cd $installRoot; .\.venv\Scripts\python.exe -m uvicorn atlas_adapter.atlas_http_api:app --host 127.0.0.1 --port $Port" -ForegroundColor Gray
Write-Host "Install complete." -ForegroundColor Green

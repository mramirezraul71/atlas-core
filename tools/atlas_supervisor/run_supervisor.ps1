param(
  [string]$Workspace = "C:\ATLAS_PUSH"
)

$ErrorActionPreference = "Stop"

Write-Host "== ATLAS Supervisor ==" -ForegroundColor Cyan
Write-Host "Workspace: $Workspace"

Set-Location $Workspace

# 1) Asegurar venv local (usa .temp_venv si ya la usas)
if (!(Test-Path ".temp_venv")) {
  Write-Host "Creando venv .temp_venv..." -ForegroundColor Yellow
  python -m venv .temp_venv
}

Write-Host "Activando venv..." -ForegroundColor Yellow
& ".\.temp_venv\Scripts\Activate.ps1"

# 2) Instalar watchdog (para vigilancia real)
Write-Host "Instalando dependencias del supervisor..." -ForegroundColor Yellow
python -m pip install --upgrade pip
python -m pip install watchdog
python -m pip install ruff black

# 3) Ejecutar daemon
Write-Host "Iniciando Supervisor Daemon..." -ForegroundColor Green
$env:ATLAS_WORKSPACE = $Workspace
if (-not $env:ATLAS_AUTOPUSH) { $env:ATLAS_AUTOPUSH = "0" }
if (-not $env:ATLAS_AUTOCOMMIT) { $env:ATLAS_AUTOCOMMIT = "0" }
if (-not $env:ATLAS_FORCE_DEV) { $env:ATLAS_FORCE_DEV = "1" }
Write-Host "ATLAS_AUTOPUSH=$env:ATLAS_AUTOPUSH | ATLAS_AUTOCOMMIT=$env:ATLAS_AUTOCOMMIT | ATLAS_FORCE_DEV=$env:ATLAS_FORCE_DEV" -ForegroundColor DarkCyan
python "tools\atlas_supervisor\supervisor_daemon.py"

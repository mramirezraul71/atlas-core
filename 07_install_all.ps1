# ATLAS one-command installer. Idempotent.
param(
  [Parameter(Mandatory=$false)][string]$RepoPath = (Get-Location).Path,
  [switch]$Service = $false
)

$ErrorActionPreference = "Stop"
Write-Host "== ATLAS_SETUP ==" -ForegroundColor Cyan
Write-Host "RepoPath: $RepoPath`n"

# 1) Prereqs
Write-Host "[1/6] Prereqs" -ForegroundColor Yellow
if (!(Test-Path $RepoPath)) { Write-Error "No existe: $RepoPath"; exit 1 }
python --version 2>$null
if ($LASTEXITCODE -ne 0) { Write-Error "Python no encontrado en PATH."; exit 1 }
git --version 2>$null
if ($LASTEXITCODE -ne 0) { Write-Warning "Git no en PATH (opcional para update)." }

# 2) Venv + deps
Write-Host "`n[2/6] Venv" -ForegroundColor Yellow
$venv = Join-Path $RepoPath ".venv"
if (!(Test-Path $venv)) {
  Set-Location $RepoPath
  python -m venv .venv
  if ($LASTEXITCODE -ne 0) { Write-Error "Fallo venv"; exit 1 }
}
$venvAct = Join-Path $RepoPath ".venv\Scripts\Activate.ps1"
& $venvAct
Set-Location $RepoPath

Write-Host "`n[3/6] Dependencias" -ForegroundColor Yellow
python -m pip install -U pip setuptools wheel -q
python -m pip install -U fastapi "uvicorn[standard]" requests python-dotenv httpx -q
$req = Join-Path $RepoPath "requirements.txt"
if (Test-Path $req) { python -m pip install -r $req -q }

# 4) Config atlas.env si falta
Write-Host "`n[4/6] Config" -ForegroundColor Yellow
$configDir = Join-Path $RepoPath "config"
$envFile = Join-Path $configDir "atlas.env"
$example = Join-Path $configDir "atlas.env.example"
if (!(Test-Path $envFile) -and (Test-Path $example)) {
  Copy-Item $example $envFile
  Write-Host "  Creado atlas.env desde plantilla." -ForegroundColor Gray
} elseif (Test-Path $envFile) {
  Write-Host "  atlas.env ya existe." -ForegroundColor Gray
}
(Join-Path $RepoPath "logs") | ForEach-Object { if (!(Test-Path $_)) { New-Item -ItemType Directory -Path $_ -Force | Out-Null } }

# 5) Smoke tests (solo si API puede arrancar en background breve o skip si no hay puerto)
Write-Host "`n[5/6] Smoke tests" -ForegroundColor Yellow
$port = 8791
$cons = Get-NetTCPConnection -LocalPort $port -ErrorAction SilentlyContinue
if ($cons) {
  Write-Host "  Puerto $port en uso. Omitiendo smoke (ejecuta después: .\04_smoke_tests.ps1 -RepoPath $RepoPath -AtlasPort $port)" -ForegroundColor Gray
} else {
  Write-Host "  Ejecuta smoke con API levantada: .\03_run_atlas_api.ps1 en una terminal y .\04_smoke_tests.ps1 en otra." -ForegroundColor Gray
}

# 6) Servicio opcional
if ($Service) {
  Write-Host "`n[6/6] Servicio Windows" -ForegroundColor Yellow
  $installScript = Join-Path $RepoPath "scripts\windows\install_service.ps1"
  if (Test-Path $installScript) {
    & $installScript -RepoPath $RepoPath -ServiceName "ATLAS_PUSH"
  } else {
    Write-Warning "scripts\windows\install_service.ps1 no encontrado."
  }
} else {
  Write-Host "`n[6/6] Servicio: omitido (usa -Service para instalar)" -ForegroundColor Gray
}

# Cómo usar
Write-Host "`n== Cómo usar ==" -ForegroundColor Green
Write-Host "  API:     .\03_run_atlas_api.ps1 -RepoPath $RepoPath -AtlasPort $port"
Write-Host "  UI:      http://127.0.0.1:$port/ui"
Write-Host "  Status:  http://127.0.0.1:$port/status"
Write-Host "  Version: http://127.0.0.1:$port/version"
Write-Host "  Approvals: GET/POST http://127.0.0.1:$port/approvals/list"
Write-Host "  Smoke:    .\04_smoke_tests.ps1 -RepoPath $RepoPath -AtlasPort $port"
Write-Host "`nListo." -ForegroundColor Green

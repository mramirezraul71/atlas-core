param(
  [Parameter(Mandatory=$true)][string]$RepoPath
)

Write-Host "== Install deps ==" -ForegroundColor Cyan
Set-Location $RepoPath

$venvAct = Join-Path $RepoPath ".venv\Scripts\Activate.ps1"
if (!(Test-Path $venvAct)) { Write-Error "No existe venv. Ejecuta 01_setup_venv.ps1 primero."; exit 1 }
& $venvAct

# Mínimos para que la API arranque
python -m pip install -U fastapi uvicorn[standard] requests python-dotenv httpx

# requirements.txt si existe
$req = Join-Path $RepoPath "requirements.txt"
if (Test-Path $req) {
  Write-Host "Instalando requirements.txt" -ForegroundColor Yellow
  python -m pip install -r $req
}

# pyproject.toml si existe
$pyproj = Join-Path $RepoPath "pyproject.toml"
if (Test-Path $pyproj) {
  Write-Host "Detectado pyproject.toml (intentando install editable)" -ForegroundColor Yellow
  python -m pip install -e .
}

Write-Host "Deps OK." -ForegroundColor Green

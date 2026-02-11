param(
  [Parameter(Mandatory=$true)][string]$RepoPath
)

$venv = Join-Path $RepoPath ".venv"
Write-Host "== Setup venv ==" -ForegroundColor Cyan
Write-Host "RepoPath: $RepoPath"
Set-Location $RepoPath

if (!(Test-Path $venv)) {
  Write-Host "Creando venv en $venv" -ForegroundColor Yellow
  python -m venv .venv
  if ($LASTEXITCODE -ne 0) { Write-Error "Fallo creando venv"; exit 1 }
}

Write-Host "Activando venv" -ForegroundColor Yellow
& (Join-Path $venv "Scripts\Activate.ps1")
if ($LASTEXITCODE -ne 0) { Write-Error "No pude activar venv"; exit 1 }

python -m pip install -U pip setuptools wheel
Write-Host "Venv OK." -ForegroundColor Green

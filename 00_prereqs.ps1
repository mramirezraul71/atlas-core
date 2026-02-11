param(
  [string]$RepoPath = "C:\ATLAS_PUSH"
)

Write-Host "== ATLAS prereqs ==" -ForegroundColor Cyan
Write-Host "RepoPath: $RepoPath"

Write-Host "`n[1/4] Python" -ForegroundColor Yellow
python --version 2>$null
if ($LASTEXITCODE -ne 0) { Write-Error "Python no encontrado en PATH."; exit 1 }

Write-Host "`n[2/4] Pip" -ForegroundColor Yellow
python -m pip --version 2>$null
if ($LASTEXITCODE -ne 0) { Write-Error "pip no disponible."; exit 1 }

Write-Host "`n[3/4] Git" -ForegroundColor Yellow
git --version 2>$null
if ($LASTEXITCODE -ne 0) { Write-Warning "git no encontrado. Si ya tienes el repo, puedes continuar igual." }

Write-Host "`n[4/4] Repo check" -ForegroundColor Yellow
if (!(Test-Path $RepoPath)) { Write-Error "No existe: $RepoPath"; exit 1 }
if (!(Test-Path (Join-Path $RepoPath ".git"))) { Write-Warning "No veo .git en $RepoPath (¿es el repo correcto?)" }

Write-Host "`nOK prereqs." -ForegroundColor Green

# Verificación de software necesario para actualizar el repositorio ATLAS
# Uso: .\scripts\verify_repo_update_prereqs.ps1

$ErrorActionPreference = "Continue"
$allOk = $true

Write-Host "=== Verificación para actualizar repo ATLAS ===" -ForegroundColor Cyan
Write-Host ""

# 1. Git (imprescindible para pull/fetch/clone)
Write-Host "[1/4] Git..." -NoNewline
try {
    $gitVer = git --version 2>&1
    if ($LASTEXITCODE -eq 0) {
        Write-Host " OK" -ForegroundColor Green
        Write-Host "    $gitVer"
    } else {
        Write-Host " FALTA" -ForegroundColor Red
        Write-Host "    Instalar: https://git-scm.com/download/win"
        $allOk = $false
    }
} catch {
    Write-Host " FALTA" -ForegroundColor Red
    Write-Host "    Instalar: https://git-scm.com/download/win"
    $allOk = $false
}

# 2. Python (para scripts del repo, tests, POTs)
Write-Host "[2/4] Python..." -NoNewline
try {
    $pyVer = python --version 2>&1
    if ($LASTEXITCODE -eq 0) {
        Write-Host " OK" -ForegroundColor Green
        Write-Host "    $pyVer"
    } else {
        $py3 = py -3 --version 2>&1
        if ($LASTEXITCODE -eq 0) { Write-Host " OK (py -3)" -ForegroundColor Green; Write-Host "    $py3" } else { Write-Host " FALTA" -ForegroundColor Red; $allOk = $false }
    }
} catch {
    Write-Host " FALTA" -ForegroundColor Red
    Write-Host "    Instalar: https://www.python.org/downloads/ (3.11+ recomendado)"
    $allOk = $false
}

# 3. pip (para dependencias del repo)
Write-Host "[3/4] pip..." -NoNewline
try {
    $pipVer = python -m pip --version 2>&1
    if ($LASTEXITCODE -eq 0) {
        Write-Host " OK" -ForegroundColor Green
        Write-Host "    $pipVer"
    } else {
        Write-Host " FALTA" -ForegroundColor Red
        Write-Host "    Ejecutar: python -m ensurepip --upgrade"
        $allOk = $false
    }
} catch {
    Write-Host " FALTA" -ForegroundColor Red
    $allOk = $false
}

# 4. Repo limpio o con cambios (solo informativo)
Write-Host "[4/4] Estado del repo..." -NoNewline
$branch = git rev-parse --abbrev-ref HEAD 2>&1
$status = git status --porcelain 2>&1
if ([string]::IsNullOrWhiteSpace($status)) {
    Write-Host " Limpio (sin cambios locales)" -ForegroundColor Green
} else {
    $lines = ($status -split "`n").Count
    Write-Host " Cambios locales ($lines entradas)" -ForegroundColor Yellow
    Write-Host "    Para actualizar: git stash -> git pull -> git stash pop"
    Write-Host "    O: commitear antes de git pull"
}

Write-Host ""
if ($allOk) {
    Write-Host "Software listo para actualizar el repo." -ForegroundColor Green
} else {
    Write-Host "Instala los componentes marcados como FALTA antes de actualizar." -ForegroundColor Red
}
Write-Host ""
Write-Host "Comandos típicos para actualizar:" -ForegroundColor Cyan
Write-Host "  git fetch origin"
Write-Host "  git status   # ver si hay commits nuevos en origin"
Write-Host "  git stash    # guardar cambios locales (opcional)"
Write-Host "  git pull [--rebase] origin <rama>"
Write-Host "  git stash pop   # recuperar cambios (si hiciste stash)"

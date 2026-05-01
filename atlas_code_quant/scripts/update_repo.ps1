# Atlas Code-Quant — sincronizar repo, verificar Fase 3 y tests (PowerShell)
# Uso: cd C:\ATLAS_PUSH\atlas_code_quant; powershell -ExecutionPolicy Bypass -File scripts\update_repo.ps1
# Nota: usa la rama actual (git rev-parse). Ajusta remoto con: git branch -vv

$ErrorActionPreference = "Stop"
$repoPath = Split-Path -Parent $PSScriptRoot
Set-Location $repoPath

$branch = (git rev-parse --abbrev-ref HEAD).Trim()
Write-Host "Rama actual: $branch" -ForegroundColor Cyan

Write-Host "`n1. Pull desde origin..." -ForegroundColor Green
git pull origin $branch
if ($LASTEXITCODE -ne 0) {
    Write-Host "ERROR: git pull fallo. Revisa conflictos o remoto." -ForegroundColor Red
    exit 1
}

Write-Host "`n2. Verificador manifest Fase 3..." -ForegroundColor Green
python scripts\verify_fase3_manifest.py
if ($LASTEXITCODE -ne 0) {
    Write-Host "ERROR: verify_fase3_manifest.py fallo (exit $LASTEXITCODE)" -ForegroundColor Red
    exit 1
}

Write-Host "`n3. Tests (suite completa puede tardar)..." -ForegroundColor Green
python -m pytest tests\ -q --tb=short
$testExit = $LASTEXITCODE
if ($testExit -ne 0) {
    Write-Host "ADVERTENCIA: pytest termino con codigo $testExit" -ForegroundColor Yellow
}

Write-Host "`n4. Estado git..." -ForegroundColor Green
git status -sb

$status = git status --porcelain
if ($status) {
    Write-Host "`nHay cambios locales. Revisa antes de commit/push (no automatizado aqui)." -ForegroundColor Yellow
} else {
    Write-Host "`nWorking tree limpio." -ForegroundColor Cyan
}

Write-Host "`nListo." -ForegroundColor Green
exit $testExit


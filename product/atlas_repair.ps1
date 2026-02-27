# Atlas Product Pack - Repair (recreate venv, reinstall deps, run selfcheck)
param([string]$TargetDir = "C:\AtlasPush", [int]$Port = 8791)
$ErrorActionPreference = "Stop"
$root = (Resolve-Path $TargetDir -ErrorAction Stop).Path
Write-Host "Repair: $root" -ForegroundColor Cyan
$venv = Join-Path $root ".venv"
if (Test-Path $venv) {
    Remove-Item $venv -Recurse -Force
    Write-Host "Removed old venv." -ForegroundColor Yellow
}
$py = Get-Command python -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Path
& $py -m venv $venv
$pip = Join-Path $venv "Scripts\pip.exe"
$req = Join-Path $root "requirements.txt"
if (Test-Path $req) { & $pip install -q -r $req }
& $pip install -q fastapi uvicorn httpx pydantic python-dotenv
Write-Host "Venv recreated." -ForegroundColor Green
& $PSScriptRoot\atlas_selfcheck.ps1 -BaseUrl "http://127.0.0.1:$Port"
exit $LASTEXITCODE

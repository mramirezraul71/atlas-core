# Atlas Product Pack - Start worker (light mode: cluster + gateway + dispatcher)
param([string]$TargetDir = "C:\AtlasPush", [int]$Port = 8792)
$ErrorActionPreference = "Stop"
$root = if ($TargetDir) { $TargetDir } else { "C:\AtlasPush" }
$env:WORKER_ONLY = "true"
$env:POLICY_ALLOWED_PATHS = $root
$env:ATLAS_REPO_PATH = $root
$env:SERVICE_PORT = $Port
$venvPy = Join-Path $root ".venv\Scripts\python.exe"
$py = if (Test-Path $venvPy) { $venvPy } else { (Get-Command python -ErrorAction Stop).Path }
Write-Host "Starting worker on port $Port (WORKER_ONLY=true)..." -ForegroundColor Cyan
Set-Location $root
& $py -m uvicorn atlas_adapter.atlas_http_api:app --host 127.0.0.1 --port $Port

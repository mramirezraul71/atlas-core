# ATLAS NEXUS - Quick Start Script
# Run: PowerShell -ExecutionPolicy Bypass -File start.ps1

param(
    [string]$mode = "api"
)

# Liberar puerto 8000 si está ocupado (evita Errno 10048)
$freePort = Join-Path (Split-Path (Split-Path $PSScriptRoot) -Parent) "scripts\free_port_8000.ps1"
if (Test-Path $freePort) { & $freePort -Kill | Out-Null }

Write-Host "🚀 Starting ATLAS NEXUS..." -ForegroundColor Cyan

# Activate venv
if (Test-Path "venv\Scripts\Activate.ps1") {
    & "venv\Scripts\Activate.ps1"
} else {
    Write-Host "✗ Virtual environment not found. Run install.ps1 first." -ForegroundColor Red
    exit 1
}

# Check config
if (-Not (Test-Path "config\.env")) {
    Write-Host "⚠ Config not found. Creating from example..." -ForegroundColor Yellow
    if (Test-Path ".env.example") {
        Copy-Item ".env.example" "config\.env"
    }
}

# Start ATLAS (usar Python del venv por ruta para evitar PATH del sistema)
$pythonExe = Join-Path $PSScriptRoot "venv\Scripts\python.exe"
if (-not (Test-Path $pythonExe)) { $pythonExe = "python" }

Write-Host "Mode: $mode" -ForegroundColor Green
Write-Host "API: http://localhost:8000" -ForegroundColor Green
Write-Host "Docs: http://localhost:8000/docs" -ForegroundColor Green
Write-Host ""

& $pythonExe nexus.py --mode $mode

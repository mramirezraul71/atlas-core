# Fuerza un ciclo de la Tríada (PyPI | GitHub | Hugging Face).
# Opción 1: vía API (PUSH en 8791) — POST /api/evolution/trigger
# Opción 2: directo — python evolution_daemon.py --run-once

param(
    [switch]$Api,
    [string]$BaseUrl = "http://127.0.0.1:8791"
)

$ErrorActionPreference = "Stop"
$root = Split-Path $PSScriptRoot -Parent
if (-not $root) { $root = (Get-Location).Path }

if ($Api) {
    try {
        $r = Invoke-RestMethod -Uri "$BaseUrl/api/evolution/trigger" -Method POST -TimeoutSec 5
        if ($r.triggered) {
            Write-Host "[OK] Ciclo Tríada iniciado en segundo plano. Ver $BaseUrl/api/evolution/status" -ForegroundColor Green
        } else {
            Write-Host "[ERROR] $($r.error)" -ForegroundColor Red
            exit 1
        }
    } catch {
        Write-Host "[ERROR] PUSH no disponible en $BaseUrl. Ejecutando ciclo directo..." -ForegroundColor Yellow
        Push-Location $root
        python evolution_daemon.py --run-once
        Pop-Location
    }
} else {
    Push-Location $root
    python evolution_daemon.py --run-once
    Pop-Location
}

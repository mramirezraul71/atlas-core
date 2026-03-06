# ATLAS PUSH - Arranque del cerebro + NEXUS + Robot
# Ejecutar desde ATLAS_PUSH: .\scripts\start_atlas.ps1

$NEXUS = if ($env:NEXUS_ATLAS_PATH) { $env:NEXUS_ATLAS_PATH } else { "C:\ATLAS_NEXUS\atlas_nexus" }
$ROBOT = if ($env:NEXUS_ROBOT_PATH) { $env:NEXUS_ROBOT_PATH } else { "C:\ATLAS_NEXUS\atlas_nexus_robot\backend" }
$PUSH  = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$RuntimeHelpers = Join-Path $PSScriptRoot "atlas_runtime.ps1"
if (-not (Test-Path $RuntimeHelpers)) {
    throw "Runtime helpers not found: $RuntimeHelpers"
}
. $RuntimeHelpers

$PY_PUSH = Resolve-AtlasPython -RepoRoot $PUSH -RequirePreflight
$PY_NEXUS = if (Test-Path (Join-Path $NEXUS "venv\Scripts\python.exe")) { Join-Path $NEXUS "venv\Scripts\python.exe" } else { $PY_PUSH }
$PY_ROBOT = if (Test-Path (Join-Path $ROBOT "venv\Scripts\python.exe")) { Join-Path $ROBOT "venv\Scripts\python.exe" } else { $PY_PUSH }

function Test-EndpointOk {
    param(
        [string[]]$Urls,
        [int]$TimeoutSec = 3
    )
    foreach ($url in $Urls) {
        try {
            $resp = Invoke-WebRequest -Uri $url -TimeoutSec $TimeoutSec -UseBasicParsing -ErrorAction Stop
            if ($resp.StatusCode -eq 200) { return $true }
        } catch {}
    }
    return $false
}

Write-Host "ATLAS - Arranque unificado" -ForegroundColor Cyan
Write-Host "NEXUS=$NEXUS | Robot=$ROBOT" -ForegroundColor Gray

# 1. NEXUS
$nexusAlive = Test-EndpointOk -Urls @("http://127.0.0.1:8000/health", "http://127.0.0.1:8000/status")
if ($nexusAlive) {
    Write-Host "NEXUS ya activo en 8000. Saltando arranque." -ForegroundColor Yellow
} elseif (Test-Path "$NEXUS\nexus.py") {
    Start-Process $PY_NEXUS -ArgumentList "nexus.py","--mode","api" -WorkingDirectory $NEXUS -WindowStyle Minimized
    Start-Sleep 3
}

# 2. Robot
$robotAlive = Test-EndpointOk -Urls @("http://127.0.0.1:8002/api/health", "http://127.0.0.1:8002/status")
if ($robotAlive) {
    Write-Host "Robot ya activo en 8002. Saltando arranque." -ForegroundColor Yellow
} elseif (Test-Path "$ROBOT\main.py") {
    Start-Process $PY_ROBOT -ArgumentList "main.py" -WorkingDirectory $ROBOT -WindowStyle Minimized
    Start-Sleep 3
}

# 3. PUSH - verificar si ya hay instancia antes de lanzar (anti-doble-arranque)
Set-Location $PUSH
$pushAlive = Test-EndpointOk -Urls @("http://127.0.0.1:8791/health")

if ($pushAlive) {
    Write-Host "PUSH ya activo en 8791 (instancia detectada). Saltando arranque." -ForegroundColor Yellow
} else {
    Write-Host "Iniciando PUSH en http://localhost:8791" -ForegroundColor Green
    & $PY_PUSH -m uvicorn atlas_adapter.atlas_http_api:app --host 0.0.0.0 --port 8791
}

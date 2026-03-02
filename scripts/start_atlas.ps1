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

$PY = Resolve-AtlasPython -RepoRoot $PUSH -RequirePreflight

Write-Host "ATLAS - Arranque unificado" -ForegroundColor Cyan
Write-Host "NEXUS=$NEXUS | Robot=$ROBOT" -ForegroundColor Gray

# 1. NEXUS
if (Test-Path "$NEXUS\nexus.py") {
    Start-Process $PY -ArgumentList "nexus.py","--mode","api" -WorkingDirectory $NEXUS -WindowStyle Minimized
    Start-Sleep 3
}
# 2. Robot
if (Test-Path "$ROBOT\main.py") {
    Start-Process $PY -ArgumentList "main.py" -WorkingDirectory $ROBOT -WindowStyle Minimized
    Start-Sleep 3
}
# 3. PUSH — verificar si ya hay instancia antes de lanzar (anti-doble-arranque)
Set-Location $PUSH
$_apiAlive = $false
try {
    $resp = Invoke-WebRequest -Uri "http://127.0.0.1:8791/health" -TimeoutSec 3 -UseBasicParsing -ErrorAction Stop
    if ($resp.StatusCode -eq 200) { $_apiAlive = $true }
} catch {}

if ($_apiAlive) {
    Write-Host "PUSH ya activo en 8791 (instancia detectada). Saltando arranque." -ForegroundColor Yellow
} else {
    Write-Host "Iniciando PUSH en http://localhost:8791" -ForegroundColor Green
    & $PY -m uvicorn atlas_adapter.atlas_http_api:app --host 0.0.0.0 --port 8791
}

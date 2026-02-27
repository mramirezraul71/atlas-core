# ATLAS PUSH - Arranque del cerebro + NEXUS + Robot
# Ejecutar desde ATLAS_PUSH: .\scripts\start_atlas.ps1

$NEXUS = if ($env:NEXUS_ATLAS_PATH) { $env:NEXUS_ATLAS_PATH } else { "C:\ATLAS_NEXUS\atlas_nexus" }
$ROBOT = if ($env:NEXUS_ROBOT_PATH) { $env:NEXUS_ROBOT_PATH } else { "C:\ATLAS_NEXUS\atlas_nexus_robot\backend" }
$PUSH  = $PSScriptRoot + "\.."

Write-Host "ATLAS - Arranque unificado" -ForegroundColor Cyan
Write-Host "NEXUS=$NEXUS | Robot=$ROBOT" -ForegroundColor Gray

# 1. NEXUS
if (Test-Path "$NEXUS\nexus.py") {
    Start-Process python -ArgumentList "nexus.py","--mode","api" -WorkingDirectory $NEXUS -WindowStyle Minimized
    Start-Sleep 3
}
# 2. Robot
if (Test-Path "$ROBOT\main.py") {
    Start-Process python -ArgumentList "main.py" -WorkingDirectory $ROBOT -WindowStyle Minimized
    Start-Sleep 3
}
# 3. PUSH (este proceso continúa en foreground)
Set-Location $PUSH
Write-Host "Iniciando PUSH en http://localhost:8791" -ForegroundColor Green
python -m uvicorn atlas_adapter.atlas_http_api:app --host 0.0.0.0 --port 8791

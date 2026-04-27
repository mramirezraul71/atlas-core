# Arrancar API Atlas (dashboard en http://127.0.0.1:8791/ui)
# ATLAS_ENABLE_RADAR_KALSHI=1 expone /ui/radar (predicciones) y /api/radar/*
$ErrorActionPreference = "Stop"
$Root = $PSScriptRoot + "\.."
Set-Location $Root
$env:PYTHONPATH = $Root
if (-not $env:ATLAS_ENABLE_RADAR_KALSHI) { $env:ATLAS_ENABLE_RADAR_KALSHI = "1" }
& python -m uvicorn atlas_adapter.atlas_http_api:app --host 127.0.0.1 --port 8791

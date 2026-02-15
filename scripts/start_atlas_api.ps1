# Arrancar API Atlas (dashboard en http://127.0.0.1:8791/ui)
$ErrorActionPreference = "Stop"
$Root = $PSScriptRoot + "\.."
Set-Location $Root
$env:PYTHONPATH = $Root
& python -m uvicorn atlas_adapter.atlas_http_api:app --host 127.0.0.1 --port 8791

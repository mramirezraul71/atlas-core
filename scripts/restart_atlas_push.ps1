# Reinicia Atlas PUSH (adaptador HTTP + /ui/radar) en :8791 sin interacción.
# Mata el proceso que escuche 8791 y arranca uvicorn en segundo plano.
$ErrorActionPreference = "Stop"
$Root = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$FreePort = Join-Path $PSScriptRoot "free_port.ps1"

if (Test-Path $FreePort) {
    & $FreePort -Port 8791 -Kill | Out-Null
} else {
    Write-Warning "free_port.ps1 no encontrado; si 8791 sigue ocupado, mata el proceso a mano."
}
Start-Sleep -Milliseconds 600

$env:PYTHONPATH = $Root
if (-not $env:ATLAS_ENABLE_RADAR_KALSHI) { $env:ATLAS_ENABLE_RADAR_KALSHI = "1" }

$py = (Get-Command python -ErrorAction SilentlyContinue).Source
if (-not $py) { throw "python no está en PATH" }

Start-Process -FilePath $py -ArgumentList @(
    "-m", "uvicorn", "atlas_adapter.atlas_http_api:app",
    "--host", "127.0.0.1", "--port", "8791"
) -WorkingDirectory $Root -WindowStyle Hidden

Write-Host "PUSH reiniciado en segundo plano: http://127.0.0.1:8791/ui/radar (espera 2-4 s)." -ForegroundColor Green

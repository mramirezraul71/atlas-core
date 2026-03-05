param(
    [string]$Workspace = "C:\ATLAS_PUSH",
    [string]$CoreToken = ""
)

$ErrorActionPreference = "Stop"

Write-Host "== ATLAS Comms Init ==" -ForegroundColor Cyan
Write-Host "Workspace: $Workspace"

Set-Location $Workspace

if ($CoreToken) {
    $env:ATLAS_CENTRAL_CORE = $CoreToken
    Write-Host "ATLAS_CENTRAL_CORE configured from parameter." -ForegroundColor Green
}

if (-not $env:ATLAS_CENTRAL_CORE) {
    Write-Warning "ATLAS_CENTRAL_CORE is not set. Encryption will fallback to machine seed."
}

if (!(Test-Path "logs")) {
    New-Item -ItemType Directory -Path "logs" -Force | Out-Null
}
if (!(Test-Path "reports")) {
    New-Item -ItemType Directory -Path "reports" -Force | Out-Null
}

Write-Host "Initializing atlas_comms_hub database..." -ForegroundColor Yellow
python -m modules.humanoid.comms.atlas_comms_hub --init-db
if ($LASTEXITCODE -ne 0) {
    Write-Error "atlas_comms_hub --init-db failed."
    exit $LASTEXITCODE
}

Write-Host "Current status:" -ForegroundColor Yellow
python -m modules.humanoid.comms.atlas_comms_hub --status
if ($LASTEXITCODE -ne 0) {
    Write-Error "atlas_comms_hub --status failed."
    exit $LASTEXITCODE
}

Write-Host ""
Write-Host "ATLAS Comms Hub initialized." -ForegroundColor Green
Write-Host "Recommended endpoints:"
Write-Host "  POST /api/comms/atlas/message"
Write-Host "  GET  /api/comms/atlas/history"
Write-Host "  GET  /api/comms/atlas/status"
Write-Host "  POST /api/comms/atlas/resync"
Write-Host "  POST /api/comms/atlas/inventory/adjust"

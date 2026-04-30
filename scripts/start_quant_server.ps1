<#
.SYNOPSIS
    Arranca el servidor Atlas-Quant en puerto 8795.
    Disenado para ser llamado por Task Scheduler.
    Si el servidor ya esta corriendo, no hace nada.
#>

$ErrorActionPreference = "Stop"

$RepoRoot = "C:\ATLAS_PUSH"
$Port = 8795
$AtlasQuantStart = Join-Path $RepoRoot "scripts\atlas_quant_start.ps1"
$HealthUrl = "http://127.0.0.1:$Port/health"

try {
    $resp = Invoke-WebRequest -Uri $HealthUrl -TimeoutSec 3 -ErrorAction Stop
    if ($resp.StatusCode -eq 200) {
        Write-Host "[$(Get-Date -Format 'HH:mm:ss')] Servidor Quant ya activo en :$Port - skip" -ForegroundColor Green
        exit 0
    }
} catch { }

if (-not (Test-Path $AtlasQuantStart)) {
    throw "Script de arranque central no encontrado: $AtlasQuantStart"
}

Write-Host "[$(Get-Date -Format 'HH:mm:ss')] Arrancando servidor Quant en :$Port ..." -ForegroundColor Cyan
& $AtlasQuantStart -Port $Port -MaxWaitSec 60
exit $LASTEXITCODE

# Arranca NEXUS (puerto 8000) desde el repo ATLAS_PUSH. Usado por heartbeat y "Reconectar NEXUS".
$ErrorActionPreference = "Stop"
# PSScriptRoot = directorio de este script (scripts\); repo = padre
$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$NexusDir = Join-Path $RepoRoot "nexus\atlas_nexus"
$StartPs1 = Join-Path $NexusDir "start.ps1"

# Si el puerto 8000 está ocupado, liberarlo (evita Errno 10048)
$FreePortScript = Join-Path $PSScriptRoot "free_port_8000.ps1"
if (Test-Path $FreePortScript) {
    $null = & $FreePortScript -Kill
}

if (Test-Path $StartPs1) {
    Set-Location $NexusDir
    & $StartPs1 -mode api
} elseif (Test-Path (Join-Path $NexusDir "nexus.py")) {
    Set-Location $NexusDir
    & python nexus.py --mode api
} else {
    Write-Error "NEXUS no encontrado en $NexusDir"
    exit 1
}

# Arranca NEXUS (puerto 8000) desde el repo ATLAS_PUSH. Usado por heartbeat y "Reconectar NEXUS".
$ErrorActionPreference = "Stop"
# PSScriptRoot = directorio de este script (scripts\); repo = padre
$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$NexusDir = Join-Path $RepoRoot "nexus\atlas_nexus"
$StartPs1 = Join-Path $NexusDir "start.ps1"

function Resolve-AtlasPython {
    param([string]$RepoRootPath)
    $candidates = @(
        (Join-Path $RepoRootPath ".venv\Scripts\python.exe"),
        (Join-Path $RepoRootPath "venv\Scripts\python.exe")
    )
    foreach ($p in $candidates) {
        if (Test-Path $p) { return $p }
    }
    $fallback = Get-Command python -ErrorAction SilentlyContinue
    if ($fallback) { return $fallback.Source }
    throw "Python no encontrado (ni en .venv ni en PATH)."
}

$PythonExe = Resolve-AtlasPython -RepoRootPath $RepoRoot

# Si el puerto 8000 está ocupado, liberarlo (evita Errno 10048)
$FreePortScript = Join-Path $PSScriptRoot "free_port_8000.ps1"
if (Test-Path $FreePortScript) {
    $null = & $FreePortScript -Kill
}

if (Test-Path $StartPs1) {
    Set-Location $NexusDir
    $env:ATLAS_PYTHON = $PythonExe
    & $StartPs1 -mode api
} elseif (Test-Path (Join-Path $NexusDir "nexus.py")) {
    Set-Location $NexusDir
    & $PythonExe nexus.py --mode api
} else {
    Write-Error "NEXUS no encontrado en $NexusDir"
    exit 1
}

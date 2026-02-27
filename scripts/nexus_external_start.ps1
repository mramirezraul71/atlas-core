param(
    [string]$NexusPath = $(if ($env:NEXUS_ATLAS_PATH) { $env:NEXUS_ATLAS_PATH } else { 'C:\ATLAS_NEXUS\atlas_nexus' })
)

$ErrorActionPreference = 'Stop'

$nexusDir = $NexusPath
$startPs1 = Join-Path $nexusDir 'start.ps1'
$nexusPy = Join-Path $nexusDir 'nexus.py'
$venvPy = Join-Path $nexusDir 'venv\Scripts\python.exe'

if (-not (Test-Path $nexusDir)) {
    Write-Output "NEXUS_EXTERNAL_INFO path_not_found=$nexusDir"
    Write-Output "Set NEXUS_ATLAS_PATH and retry."
    exit 0
}

try {
    $healthUrl = if ($env:NEXUS_BASE_URL) { ($env:NEXUS_BASE_URL.TrimEnd('/')) + '/health' } else { 'http://127.0.0.1:8000/health' }
    $h = Invoke-WebRequest -Uri $healthUrl -Method GET -TimeoutSec 2 -UseBasicParsing
    if ($h.StatusCode -eq 200) {
        Write-Output "NEXUS_EXTERNAL_INFO already_running url=$healthUrl status=200"
        exit 0
    }
} catch {
    # Continue to start path.
}

if (Test-Path $startPs1) {
    Start-Process powershell.exe -ArgumentList @('-NoProfile','-ExecutionPolicy','Bypass','-File',$startPs1,'-mode','api') -WorkingDirectory $nexusDir -WindowStyle Hidden
    Write-Output "NEXUS_EXTERNAL_START_OK mode=start_ps1 path=$startPs1"
    exit 0
}

if (Test-Path $nexusPy) {
    $py = if (Test-Path $venvPy) { $venvPy } else { 'python' }
    Start-Process $py -ArgumentList @('nexus.py','--mode','api') -WorkingDirectory $nexusDir -WindowStyle Hidden
    Write-Output "NEXUS_EXTERNAL_START_OK mode=python path=$nexusPy py=$py"
    exit 0
}

Write-Output "NEXUS_EXTERNAL_INFO missing_entrypoint nexusPy=$nexusPy"
Write-Output "Expected external runtime with nexus.py or start.ps1"
exit 0

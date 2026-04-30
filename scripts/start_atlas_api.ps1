# Arrancar API Atlas (dashboard en http://127.0.0.1:8791/ui)
$ErrorActionPreference = "Stop"
$Root = $PSScriptRoot + "\.."
Set-Location $Root
$env:PYTHONPATH = $Root
$env:ATLAS_SAFE_STARTUP = if ($env:ATLAS_SAFE_STARTUP) { $env:ATLAS_SAFE_STARTUP } else { "true" }
$env:ATLAS_MINIMAL_STARTUP = if ($env:ATLAS_MINIMAL_STARTUP) { $env:ATLAS_MINIMAL_STARTUP } else { "true" }

$RuntimeHelpers = Join-Path $PSScriptRoot "atlas_runtime.ps1"
$FreePortScript = Join-Path $PSScriptRoot "free_port.ps1"
if (-not (Test-Path $RuntimeHelpers)) {
    throw "Runtime helpers no encontrados: $RuntimeHelpers"
}
. $RuntimeHelpers

$LockPath = Join-Path $Root "logs\push_start.lock"
$LockHandle = Acquire-AtlasFileLock -Path $LockPath
if (-not $LockHandle) {
    throw "Ya hay otra rutina arrancando PUSH. Espera y reintenta."
}

try {
    Stop-AtlasPythonProcesses -Pattern 'atlas_adapter\.atlas_http_api:app'
    Start-Sleep -Milliseconds 300
    if (Test-Path $FreePortScript) {
        & $FreePortScript -Port 8791 -Kill | Out-Null
        Start-Sleep -Milliseconds 500
    }
    $Python = Resolve-AtlasPython -RepoRoot $Root -RequirePreflight
    & $Python -m uvicorn atlas_adapter.atlas_http_api:app --host 127.0.0.1 --port 8791
} finally {
    Release-AtlasFileLock -Handle $LockHandle
}

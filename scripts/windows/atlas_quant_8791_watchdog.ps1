[CmdletBinding()]
param()

$ErrorActionPreference = "Stop"

$repoRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot "..\.."))
$baseWatchdog = Join-Path $PSScriptRoot "atlas_quant_8795_watchdog.ps1"

if (-not (Test-Path $baseWatchdog)) {
    throw "No se encontró watchdog base: $baseWatchdog"
}

& powershell -ExecutionPolicy Bypass -File $baseWatchdog `
    -Silent `
    -BaseUrl "http://127.0.0.1:8791" `
    -Port 8791 `
    -AppModule "atlas_adapter.atlas_http_api:app" `
    -PythonExe ".venv_push\Scripts\python.exe" `
    -FunctionalPath "/health" `
    -HealthTimeoutSec 8 `
    -HealthProbeAttempts 2 `
    -PostStartRetries 240 `
    -PostStartDelaySeconds 2 `
    -WatchdogMutexName "Global\ATLAS_8791_Watchdog_Mutex"

exit $LASTEXITCODE

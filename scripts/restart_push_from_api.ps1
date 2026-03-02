# Reinicio de PUSH (8791) para ser invocado desde la API /update/restart.
# Espera unos segundos, mata el proceso en 8791 y arranca uvicorn de nuevo.
param([int]$DelaySeconds = 3)

$ErrorActionPreference = "Continue"
$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$FreePortScript = Join-Path $PSScriptRoot "free_port.ps1"

Set-Location $RepoRoot
Start-Sleep -Seconds $DelaySeconds

if (Test-Path $FreePortScript) {
    & $FreePortScript -Port 8791 -Kill | Out-Null
    Start-Sleep -Milliseconds 800
}

# Preferir Python 3.11 (el que tiene todos los paquetes de ATLAS); fallback a python del PATH
$Py311 = "C:\Users\r6957\AppData\Local\Programs\Python\Python311\python.exe"
$PyExe = if (Test-Path $Py311) { $Py311 } else { "python" }
Start-Process $PyExe -ArgumentList "-m", "uvicorn", "atlas_adapter.atlas_http_api:app", "--host", "0.0.0.0", "--port", "8791" -WorkingDirectory $RepoRoot -WindowStyle Normal

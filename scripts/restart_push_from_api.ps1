# Reinicio de PUSH (8791) para ser invocado desde la API /update/restart.
# Espera unos segundos, mata el proceso en 8791 y arranca uvicorn de nuevo.
param(
    [int]$DelaySeconds = 3,
    [int]$HealthTimeoutSec = 45
)

$ErrorActionPreference = "Continue"
$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$FreePortScript = Join-Path $PSScriptRoot "free_port.ps1"
$RuntimeHelpers = Join-Path $PSScriptRoot "atlas_runtime.ps1"
$HealthUrl = "http://127.0.0.1:8791/health"

if (-not (Test-Path $RuntimeHelpers)) {
    Write-Error "Runtime helpers not found: $RuntimeHelpers"
    exit 1
}
. $RuntimeHelpers

try {
    $PyExe = Resolve-AtlasPython -RepoRoot $RepoRoot -RequirePreflight
} catch {
    Write-Error $_.Exception.Message
    exit 1
}

Set-Location $RepoRoot
Start-Sleep -Seconds $DelaySeconds

if (Test-Path $FreePortScript) {
    & $FreePortScript -Port 8791 -Kill | Out-Null
    Start-Sleep -Milliseconds 800
}

$args = @("-m", "uvicorn", "atlas_adapter.atlas_http_api:app", "--host", "0.0.0.0", "--port", "8791")
Start-Process $PyExe -ArgumentList $args -WorkingDirectory $RepoRoot -WindowStyle Hidden | Out-Null

$sw = [System.Diagnostics.Stopwatch]::StartNew()
$healthy = $false
while ($sw.Elapsed.TotalSeconds -lt $HealthTimeoutSec) {
    try {
        $resp = Invoke-WebRequest -Uri $HealthUrl -TimeoutSec 4 -UseBasicParsing -ErrorAction Stop
        if ($resp.StatusCode -eq 200) {
            $healthy = $true
            break
        }
    } catch {}
    Start-Sleep -Seconds 2
}

if (-not $healthy) {
    Write-Error "PUSH did not become healthy on 8791 within ${HealthTimeoutSec}s."
    exit 1
}

Write-Host "PUSH restart OK on 8791 (python: $PyExe)."
exit 0

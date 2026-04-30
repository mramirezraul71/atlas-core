# Relanza PUSH (8791) en segundo plano y deja salida en logs para diagnostico.
param(
    [int]$HealthTimeoutSec = 120
)

$ErrorActionPreference = "Stop"
$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$FreePortScript = Join-Path $PSScriptRoot "free_port.ps1"
$RuntimeHelpers = Join-Path $PSScriptRoot "atlas_runtime.ps1"
$HealthUrl = "http://127.0.0.1:8791/health"
$LogDir = Join-Path $RepoRoot "logs"
$StdOutLog = Join-Path $LogDir "push_hidden_stdout.log"
$StdErrLog = Join-Path $LogDir "push_hidden_stderr.log"
$LockFile = Join-Path $LogDir "push_start.lock"
$LockHandle = $null

if (-not (Test-Path $RuntimeHelpers)) {
    throw "Runtime helpers no encontrados: $RuntimeHelpers"
}
. $RuntimeHelpers

$LockHandle = Acquire-AtlasFileLock -Path $LockFile
if (-not $LockHandle) {
    throw "Ya hay otra rutina arrancando PUSH."
}

New-Item -ItemType Directory -Path $LogDir -Force | Out-Null

try {
    $Python = Resolve-AtlasPython -RepoRoot $RepoRoot -RequirePreflight
    $env:ATLAS_SAFE_STARTUP = if ($env:ATLAS_SAFE_STARTUP) { $env:ATLAS_SAFE_STARTUP } else { "true" }
    $env:ATLAS_MINIMAL_STARTUP = if ($env:ATLAS_MINIMAL_STARTUP) { $env:ATLAS_MINIMAL_STARTUP } else { "true" }

    Stop-AtlasPythonProcesses -Pattern 'atlas_adapter\.atlas_http_api:app'
    Start-Sleep -Milliseconds 300

    if (Test-Path $FreePortScript) {
        & $FreePortScript -Port 8791 -Kill | Out-Null
    }

    $args = @("-m", "uvicorn", "atlas_adapter.atlas_http_api:app", "--host", "127.0.0.1", "--port", "8791", "--log-level", "info")
    $proc = Start-Process `
        -FilePath $Python `
        -ArgumentList $args `
        -WorkingDirectory $RepoRoot `
        -WindowStyle Hidden `
        -RedirectStandardOutput $StdOutLog `
        -RedirectStandardError $StdErrLog `
        -PassThru

    Start-Sleep -Milliseconds 750
    if ($proc.HasExited) {
        Write-Host "STDOUT log: $StdOutLog"
        Write-Host "STDERR log: $StdErrLog"
        throw "PUSH salio inmediatamente tras iniciar (exit=$($proc.ExitCode))."
    }

    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    while ($sw.Elapsed.TotalSeconds -lt $HealthTimeoutSec) {
        try {
            $resp = Invoke-WebRequest -Uri $HealthUrl -TimeoutSec 5 -UseBasicParsing -ErrorAction Stop
            if ($resp.StatusCode -eq 200) {
                Write-Host "PUSH OK on 8791 (PID: $($proc.Id))."
                exit 0
            }
        } catch {}
        Start-Sleep -Seconds 2
    }

    Write-Host "STDOUT log: $StdOutLog"
    Write-Host "STDERR log: $StdErrLog"
    throw "PUSH no respondio en 8791 dentro de ${HealthTimeoutSec}s."
} finally {
    Release-AtlasFileLock -Handle $LockHandle
}

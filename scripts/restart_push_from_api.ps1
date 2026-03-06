# Reinicio de PUSH (8791) para ser invocado desde la API /update/restart.
# Espera unos segundos, mata el proceso en 8791 y arranca uvicorn de nuevo.
param(
    [int]$DelaySeconds = 3,
    [int]$HealthTimeoutSec = 120
)

$ErrorActionPreference = "Continue"
$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$FreePortScript = Join-Path $PSScriptRoot "free_port.ps1"
$RuntimeHelpers = Join-Path $PSScriptRoot "atlas_runtime.ps1"
$HealthUrl = "http://127.0.0.1:8791/health"
$LogDir = Join-Path $RepoRoot "logs"
$LockFile = Join-Path $LogDir "push_restart.lock"
$script:LockHandle = $null

New-Item -ItemType Directory -Path $LogDir -Force | Out-Null

function Acquire-RestartLock {
    try {
        $script:LockHandle = [System.IO.File]::Open(
            $LockFile,
            [System.IO.FileMode]::OpenOrCreate,
            [System.IO.FileAccess]::ReadWrite,
            [System.IO.FileShare]::None
        )
        return $true
    } catch {
        return $false
    }
}

function Release-RestartLock {
    try {
        if ($script:LockHandle) {
            $script:LockHandle.Close()
            $script:LockHandle.Dispose()
            $script:LockHandle = $null
        }
    } catch {}
}

if (-not (Acquire-RestartLock)) {
    Write-Host "Restart lock active. Skipping duplicate restart request."
    exit 2
}

if (-not (Test-Path $RuntimeHelpers)) {
    Write-Error "Runtime helpers not found: $RuntimeHelpers"
    exit 1
}
. $RuntimeHelpers

function Stop-StalePushProcesses {
    try {
        $stale = Get-CimInstance Win32_Process -Filter "Name='python.exe'" -ErrorAction SilentlyContinue |
            Where-Object { $_.CommandLine -and $_.CommandLine -match 'atlas_adapter\.atlas_http_api:app' }
        foreach ($proc in $stale) {
            try {
                Stop-Process -Id $proc.ProcessId -Force -ErrorAction SilentlyContinue
            } catch {}
        }
    } catch {}
}

try {
    $PyExe = Resolve-AtlasPython -RepoRoot $RepoRoot -RequirePreflight
} catch {
    Write-Error $_.Exception.Message
    exit 1
}

Set-Location $RepoRoot
Start-Sleep -Seconds $DelaySeconds
Stop-StalePushProcesses
Start-Sleep -Milliseconds 500

if (Test-Path $FreePortScript) {
    & $FreePortScript -Port 8791 -Kill | Out-Null
    Start-Sleep -Milliseconds 800
}

$args = @("-m", "uvicorn", "atlas_adapter.atlas_http_api:app", "--host", "127.0.0.1", "--port", "8791")
$pushProc = Start-Process $PyExe -ArgumentList $args -WorkingDirectory $RepoRoot -WindowStyle Hidden -PassThru
Start-Sleep -Milliseconds 400
if ($pushProc.HasExited) {
    Release-RestartLock
    Write-Error "PUSH process exited immediately after start (exit=$($pushProc.ExitCode))."
    exit 1
}

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
    Release-RestartLock
    Write-Error "PUSH did not become healthy on 8791 within ${HealthTimeoutSec}s."
    exit 1
}

Release-RestartLock
Write-Host "PUSH restart OK on 8791 (python: $PyExe)."
exit 0

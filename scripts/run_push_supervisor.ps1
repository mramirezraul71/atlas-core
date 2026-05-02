# Supervisor estable para PUSH (uvicorn :8791): reinicia ante caida, logs dedicados.
# Uso (consolidated de ejemplo):
#   powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_push_supervisor.ps1 -RepoPath C:\dev\atlas\atlas-consolidated
# Env: ATLAS_REPO_ROOT como -RepoPath por defecto; SERVICE_PORT / SERVICE_BIND / SERVICE_APP_IMPORT opcionales.

param(
    [string]$RepoPath = $(if ($env:ATLAS_REPO_ROOT) { $env:ATLAS_REPO_ROOT } else { "" }),
    [string]$BindHost = $(if ($env:SERVICE_BIND) { $env:SERVICE_BIND } else { "127.0.0.1" }),
    [int]$Port = $(if ($env:SERVICE_PORT) { [int]$env:SERVICE_PORT } else { 8791 }),
    [string]$AppImport = $(if ($env:SERVICE_APP_IMPORT) { $env:SERVICE_APP_IMPORT } else { "atlas_adapter.atlas_http_api:app" }),
    [int]$RestartSeconds = 8,
    [switch]$SkipIfHealthy = $true,
    [switch]$TakePort
)

$ErrorActionPreference = "Continue"
if (-not $RepoPath -or -not (Test-Path -LiteralPath $RepoPath)) {
    Write-Error "Indica -RepoPath o define ATLAS_REPO_ROOT al directorio raiz del repo (donde estan tools\ y atlas_adapter\)."
    exit 1
}

$RepoPath = (Resolve-Path -LiteralPath $RepoPath).Path
$launcher = Join-Path $RepoPath "tools\service_launcher.py"
if (-not (Test-Path -LiteralPath $launcher)) {
    Write-Error "No existe tools\service_launcher.py en $RepoPath"
    exit 1
}

$venvPy = Join-Path $RepoPath ".venv\Scripts\python.exe"
if (-not (Test-Path -LiteralPath $venvPy)) {
    Write-Error "No hay .venv\Scripts\python.exe. Crea el venv en este repo antes del supervisor."
    exit 1
}

$logDir = Join-Path $RepoPath "logs"
if (-not (Test-Path -LiteralPath $logDir)) { New-Item -ItemType Directory -Path $logDir -Force | Out-Null }
$superLog = Join-Path $logDir "push_supervisor.log"

function Write-SupervisorLog([string]$Msg) {
    $line = "{0:yyyy-MM-dd HH:mm:ss} {1}" -f (Get-Date), $Msg
    Add-Content -LiteralPath $superLog -Value $line -Encoding UTF8
    Write-Host $line
}

function Test-PushHealthy {
    try {
        $uri = "http://${BindHost}:${Port}/health"
        $r = Invoke-WebRequest -Uri $uri -UseBasicParsing -TimeoutSec 4 -ErrorAction Stop
        return ($r.StatusCode -eq 200)
    }
    catch {
        return $false
    }
}

function Clear-ListenersOnPort {
    $listeners = Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction SilentlyContinue
    if (-not $listeners) { return }
    $pids = @($listeners | Select-Object -ExpandProperty OwningProcess | Sort-Object -Unique)
    foreach ($procId in $pids) {
        if ($procId -and $procId -gt 0) {
            Write-SupervisorLog "Puerto $Port ocupado por PID $procId. Stop-Process (-TakePort)."
            Stop-Process -Id $procId -Force -ErrorAction SilentlyContinue
        }
    }
    Start-Sleep -Seconds 1
}

if ($SkipIfHealthy -and (Test-PushHealthy)) {
    Write-SupervisorLog "PUSH ya responde en http://${BindHost}:${Port}/health. Supervisor no arranca otro proceso."
    exit 0
}

$busy = $false
try {
    $listeners = Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction SilentlyContinue
    $busy = [bool]$listeners
}
catch { $busy = $false }

if ($busy) {
    if ($TakePort) {
        Clear-ListenersOnPort
    }
    elseif (-not (Test-PushHealthy)) {
        Write-SupervisorLog "Puerto $Port en uso y /health no OK. Usa -TakePort para liberar o revisa el proceso."
        exit 1
    }
}

$env:PYTHONPATH = $RepoPath
$env:SERVICE_BIND = $BindHost
$env:SERVICE_PORT = "$Port"
$env:SERVICE_APP_IMPORT = $AppImport
Set-Location -LiteralPath $RepoPath

Write-SupervisorLog "Supervisor PUSH inicio (repo=$RepoPath bind=$BindHost port=$Port app=$AppImport). Log servicio: logs\service.log"

while ($true) {
    Write-SupervisorLog "Lanzando service_launcher..."
    try {
        & $venvPy $launcher
        $code = $LASTEXITCODE
    }
    catch {
        $code = 1
        Write-SupervisorLog "Excepcion al lanzar: $($_.Exception.Message)"
    }
    Write-SupervisorLog "Proceso terminado (exit=$code). Reinicio en ${RestartSeconds}s."
    Start-Sleep -Seconds $RestartSeconds
}

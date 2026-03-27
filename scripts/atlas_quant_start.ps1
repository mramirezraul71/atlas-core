
<#
.SYNOPSIS
    ATLAS-Quant startup script -- levanta el servidor y opcionalmente activa el auto-cycle loop.

.DESCRIPTION
    1. Verifica si ya hay un servidor Quant corriendo en el puerto objetivo.
    2. Si no hay servidor, lo lanza via uvicorn en un proceso separado.
    3. Espera a que el health endpoint responda (hasta MaxWaitSec segundos).
    4. Si -AutoCycle es especificado, activa el loop autonomo via REST.
    5. Registra todo en ops_bus.log y snapshot_safe_diagnostic.log.

.PARAMETER RepoRoot
    Raiz del repositorio ATLAS. Por defecto: directorio padre del script.

.PARAMETER Port
    Puerto en el que levantar el servidor Quant. Por defecto: 8795.

.PARAMETER ApiKey
    API key del servidor Quant. Por defecto: atlas-quant-local.

.PARAMETER AutoCycle
    Si se especifica, activa el auto-cycle loop despues del startup.

.PARAMETER CycleIntervalSec
    Intervalo en segundos entre ciclos del auto-cycle loop. Por defecto: 120.

.PARAMETER MaxWaitSec
    Maximo de segundos esperando a que el servidor responda. Por defecto: 60.

.PARAMETER LogOnly
    No lanza servidor si no esta corriendo -- solo verifica y loguea el estado.

.PARAMETER EnableReload
    Activa uvicorn --reload para desarrollo interactivo. Por defecto desactivado en runbooks operacionales.

.EXAMPLE
    .\atlas_quant_start.ps1
    .\atlas_quant_start.ps1 -Port 8795 -AutoCycle -CycleIntervalSec 90
    .\atlas_quant_start.ps1 -LogOnly
#>
param(
    [string]$RepoRoot        = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path,
    [int]$Port               = 8795,
    [string]$ApiKey          = "atlas-quant-local",
    [switch]$AutoCycle,
    [int]$CycleIntervalSec   = 120,
    [int]$MaxWaitSec         = 60,
    [switch]$LogOnly,
    [switch]$EnableReload
)

$ErrorActionPreference = "Stop"

# ── Paths ─────────────────────────────────────────────────────────────────────
$_LOG_DIR    = Join-Path $RepoRoot "logs"
$_OPS_BUS    = Join-Path $_LOG_DIR "ops_bus.log"
$_DIAG_LOG   = Join-Path $_LOG_DIR "snapshot_safe_diagnostic.log"
$_VENV_PY    = Join-Path $RepoRoot "venv\Scripts\python.exe"
$_QUANT_DIR  = Join-Path $RepoRoot "atlas_code_quant"
$_PYTHONPATH_PARTS = @($RepoRoot, $_QUANT_DIR)
if ($env:PYTHONPATH) { $_PYTHONPATH_PARTS += $env:PYTHONPATH }
$_QUANT_PYTHONPATH = [string]::Join(";", ($_PYTHONPATH_PARTS | Where-Object { $_ -and $_.Trim() -ne "" } | Select-Object -Unique))
$_API_BASE   = "http://127.0.0.1:$Port"
$_API_HEALTH = "$_API_BASE/health"

if (-not (Test-Path $_LOG_DIR)) { New-Item -ItemType Directory -Path $_LOG_DIR -Force | Out-Null }

# ── Logging helpers ───────────────────────────────────────────────────────────
function _Ts { (Get-Date).ToUniversalTime().ToString("yyyy-MM-ddTHH:mm:ssZ") }

function _OpsLog([string]$msg, [string]$level = "info") {
    try {
        "$(_Ts) [$($level.ToUpper().PadRight(8))] [QUANT_START] $msg" |
            Out-File -FilePath $_OPS_BUS -Append -Encoding utf8
    } catch {}
}

function _DiagLog([string]$msg) {
    try { "$(_Ts) QUANT_START $msg" | Out-File -FilePath $_DIAG_LOG -Append -Encoding utf8 } catch {}
}

# ── Port/process probe ────────────────────────────────────────────────────────
function Test-PortListen([int]$p) {
    $conn = Get-NetTCPConnection -LocalPort $p -State Listen -ErrorAction SilentlyContinue
    return $null -ne $conn
}

function Get-PortOwners([int]$p) {
    $owners = Get-NetTCPConnection -LocalPort $p -ErrorAction SilentlyContinue |
        Select-Object -ExpandProperty OwningProcess -Unique
    return @($owners | Where-Object { $_ -and $_ -gt 0 })
}

function Test-HealthEndpoint([string]$url, [int]$timeoutMs = 4000) {
    try {
        $wr = [System.Net.WebRequest]::Create($url)
        $wr.Timeout = $timeoutMs
        $wr.Method  = "GET"
        $resp = $wr.GetResponse()
        $ok = ([int]$resp.StatusCode -eq 200)
        $resp.Close()
        return $ok
    } catch { return $false }
}

# ── REST helper ───────────────────────────────────────────────────────────────
function Invoke-QuantApi([string]$method, [string]$path, [hashtable]$body = @{}) {
    $url = "$_API_BASE$path"
    $headers = @{ "x-api-key" = $ApiKey; "Content-Type" = "application/json" }
    try {
        if ($method -eq "GET") {
            $r = Invoke-RestMethod -Uri $url -Method GET -Headers $headers -TimeoutSec 10
        } else {
            $json = ($body | ConvertTo-Json -Compress)
            $r = Invoke-RestMethod -Uri $url -Method POST -Headers $headers -Body $json -TimeoutSec 10
        }
        return $r
    } catch {
        return $null
    }
}

# ─────────────────────────────────────────────────────────────────────────────
_OpsLog "Iniciando ATLAS-Quant en puerto $Port (AutoCycle=$AutoCycle CycleIntervalSec=$CycleIntervalSec)"
_DiagLog "START port=$Port AutoCycle=$AutoCycle"
Write-Host "[quant-start] ATLAS-Quant startup -- puerto $Port"

# ── Step 1: Check if already running ─────────────────────────────────────────
$portOwners = Get-PortOwners -p $Port
$alreadyListening = Test-PortListen -p $Port
$healthOk = Test-HealthEndpoint -url $_API_HEALTH

if (-not $healthOk -and $portOwners.Count -gt 0 -and -not $LogOnly) {
    _OpsLog "Puerto $Port ocupado sin health; limpiando procesos: $($portOwners -join ', ')" "warn"
    Write-Host "[quant-start] Puerto $Port ocupado sin health. Limpiando proceso(s): $($portOwners -join ', ')"
    foreach ($ownerPid in $portOwners) {
        try {
            Stop-Process -Id $ownerPid -Force -ErrorAction Stop
        } catch {
            _OpsLog ("No se pudo detener PID {0} en puerto {1}: {2}" -f $ownerPid, $Port, $_.Exception.Message) "warn"
        }
    }
    Start-Sleep -Seconds 2
    $portOwners = Get-PortOwners -p $Port
    $alreadyListening = Test-PortListen -p $Port
    $healthOk = Test-HealthEndpoint -url $_API_HEALTH
}

if ($healthOk) {
    _OpsLog "Servidor ya corriendo en puerto $Port -- saltando lanzamiento"
    Write-Host "[quant-start] Servidor ya activo en $Port"
} elseif ($portOwners.Count -gt 0) {
    _OpsLog "Puerto $Port sigue ocupado tras limpieza; procesos: $($portOwners -join ', ')" "high"
    Write-Warning "[quant-start] Puerto $Port sigue ocupado por: $($portOwners -join ', ')"
    _DiagLog "FINISH status=error reason=port_busy port=$Port owners=$($portOwners -join ',')"
    exit 2
} elseif ($LogOnly) {
    _OpsLog "LogOnly=true -- servidor NO esta corriendo en puerto $Port" "warn"
    Write-Host "[quant-start] Modo LogOnly: servidor no activo."
    _DiagLog "FINISH status=not_running"
    exit 0
} else {
    # ── Step 2: Launch server ─────────────────────────────────────────────────
    if (-not (Test-Path $_VENV_PY)) {
        _OpsLog "Python venv no encontrado en: $_VENV_PY" "high"
        Write-Error "[quant-start] ERROR: Python venv no encontrado."
        _DiagLog "FINISH status=error reason=venv_missing"
        exit 1
    }
    if (-not (Test-Path $_QUANT_DIR)) {
        _OpsLog "Directorio atlas_code_quant no encontrado: $_QUANT_DIR" "high"
        Write-Error "[quant-start] ERROR: atlas_code_quant no encontrado."
        _DiagLog "FINISH status=error reason=quant_dir_missing"
        exit 1
    }

    $uvicornArgs = @(
        "-m", "uvicorn",
        "api.main:app",
        "--host", "0.0.0.0",
        "--port", "$Port"
    )
    if ($EnableReload) {
        $uvicornArgs += "--reload"
    }

    _OpsLog "Lanzando uvicorn en puerto $Port"
    Write-Host "[quant-start] Lanzando uvicorn -- puerto $Port"
    $env:PYTHONPATH = $_QUANT_PYTHONPATH
    Start-Process -FilePath $_VENV_PY `
        -ArgumentList $uvicornArgs `
        -WorkingDirectory $_QUANT_DIR `
        -WindowStyle Minimized `
        -PassThru | Out-Null
}

# ── Step 3: Wait for health ───────────────────────────────────────────────────
if (-not $healthOk) {
    $waited  = 0
    $stepSec = 3
    Write-Host "[quant-start] Esperando health endpoint (max ${MaxWaitSec}s)..."
    while ($waited -lt $MaxWaitSec) {
        Start-Sleep -Seconds $stepSec
        $waited += $stepSec
        $healthOk = Test-HealthEndpoint -url $_API_HEALTH
        if ($healthOk) {
            _OpsLog "Servidor respondio en /health tras ${waited}s"
            Write-Host "[quant-start] Servidor activo tras ${waited}s"
            break
        }
        Write-Host "[quant-start]   ... ${waited}s esperados"
    }
    if (-not $healthOk) {
        _OpsLog "Servidor NO respondio en ${MaxWaitSec}s en puerto $Port" "high"
        Write-Warning "[quant-start] Servidor no respondio en tiempo. Verifica logs del proceso."
        _DiagLog "FINISH status=timeout port=$Port waited=${waited}s"
        exit 2
    }
}

# ── Step 4: Log scanner + operation status ────────────────────────────────────
$opStatus = Invoke-QuantApi -method "GET" -path "/api/v2/quant/operation/status"
if ($opStatus) {
    $autonMode = $opStatus.data.config.auton_mode
    $scanRunning = if ($null -ne $opStatus.data.scanner) { $opStatus.data.scanner.running } else { $null }
    _OpsLog "Estado -- auton_mode=$autonMode scanner_running=$scanRunning"
    Write-Host "[quant-start] auton_mode=$autonMode  scanner_running=$scanRunning"
} else {
    _OpsLog "No se pudo obtener operation/status (API key o endpoint no disponible)" "warn"
}

# Vision diagnose
$visionDiag = Invoke-QuantApi -method "GET" -path "/api/v2/quant/operation/vision"
if ($visionDiag -and $visionDiag.data) {
    $providerReady = $visionDiag.data.status.provider_ready
    $provider      = $visionDiag.data.status.provider
    _OpsLog "Vision -- provider=$provider provider_ready=$providerReady"
    Write-Host "[quant-start] Vision: provider=$provider  ready=$providerReady"
    if (-not $providerReady -and $visionDiag.data.diagnose.suggestion) {
        Write-Host "[quant-start] Sugerencia vision: $($visionDiag.data.diagnose.suggestion)"
    }
}

# ── Step 5: Auto-cycle (opcional) ─────────────────────────────────────────────
if ($AutoCycle) {
    $loopBody = @{ interval_sec = $CycleIntervalSec; max_per_cycle = 1 }
    $loopResp = Invoke-QuantApi -method "POST" -path "/api/v2/quant/operation/loop/start" -body $loopBody
    if ($loopResp -and $loopResp.ok) {
        _OpsLog "Auto-cycle loop activado -- interval=${CycleIntervalSec}s"
        Write-Host "[quant-start] Auto-cycle activado. Intervalo: ${CycleIntervalSec}s"
    } else {
        $loopErr = if ($loopResp) { $loopResp.error } else { "sin respuesta" }
        _OpsLog "Auto-cycle NO pudo activarse: $loopErr" "warn"
        Write-Warning "[quant-start] Auto-cycle error: $loopErr"
    }
}

_OpsLog "Startup completado -- puerto $Port"
_DiagLog "FINISH status=ok port=$Port AutoCycle=$AutoCycle"
Write-Host "[quant-start] Completado. Servidor en http://127.0.0.1:$Port/ui"
exit 0

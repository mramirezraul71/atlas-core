
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

.PARAMETER FullStartup
    Equivalente a forzar QUANT_LIGHTWEIGHT_STARTUP=false (arranque completo).

.PARAMETER Lightweight
    Activa QUANT_LIGHTWEIGHT_STARTUP=true (boot mínimo). El dashboard puede mostrar métricas incompletas o SYNC degradado hasta warm-up.

.PARAMETER SkipMonitoring
    Omite asegurar la cadena Prometheus/Grafana tras levantar Quant.

.EXAMPLE
    .\atlas_quant_start.ps1
    .\atlas_quant_start.ps1 -Port 8795 -AutoCycle -CycleIntervalSec 90
    .\atlas_quant_start.ps1 -LogOnly
    .\atlas_quant_start.ps1 -Lightweight
#>
param(
    [string]$RepoRoot        = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path,
    [int]$Port               = 8795,
    [string]$ApiKey          = "atlas-quant-local",
    [switch]$AutoCycle,
    [int]$CycleIntervalSec   = 120,
    [int]$MaxWaitSec         = 60,
    [switch]$LogOnly,
    [switch]$EnableReload,
    [switch]$FullStartup,
    [switch]$Lightweight,
    [switch]$SkipMonitoring
)

$ErrorActionPreference = "Stop"

# ── Paths ─────────────────────────────────────────────────────────────────────
$_LOG_DIR    = Join-Path $RepoRoot "logs"
$_OPS_BUS    = Join-Path $_LOG_DIR "ops_bus.log"
$_DIAG_LOG   = Join-Path $_LOG_DIR "snapshot_safe_diagnostic.log"
$_UVICORN_STDOUT = Join-Path $_LOG_DIR "quant_uvicorn_stdout.log"
$_UVICORN_STDERR = Join-Path $_LOG_DIR "quant_uvicorn_stderr.log"
$_VENV_PY    = Join-Path $RepoRoot "venv\Scripts\python.exe"
$_QUANT_DIR  = Join-Path $RepoRoot "atlas_code_quant"
$_PYTHONPATH_PARTS = @($RepoRoot, $_QUANT_DIR)
if ($env:PYTHONPATH) { $_PYTHONPATH_PARTS += $env:PYTHONPATH }
$_QUANT_PYTHONPATH = [string]::Join(";", ($_PYTHONPATH_PARTS | Where-Object { $_ -and $_.Trim() -ne "" } | Select-Object -Unique))
$_API_BASE   = "http://127.0.0.1:$Port"
$_API_HEALTH = "$_API_BASE/health"
$_START_MONITORING = Join-Path $RepoRoot "scripts\start_monitoring.ps1"

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
        Where-Object { $_.State -eq "Listen" } |
        Select-Object -ExpandProperty OwningProcess -Unique
    return @($owners | Where-Object { $_ -and $_ -gt 0 })
}

function Get-QuantProcessCandidates([int]$p) {
    $needle = "--port $p"
    $matches = Get-CimInstance Win32_Process -ErrorAction SilentlyContinue |
        Where-Object {
            $_.Name -like "python*" -and
            $_.CommandLine -and
            $_.CommandLine -like "*uvicorn*api.main:app*" -and
            $_.CommandLine -like "*$needle*"
        } |
        Select-Object -ExpandProperty ProcessId -Unique
    return @($matches | Where-Object { $_ -and $_ -gt 0 })
}

function Expand-QuantPidClosure([int[]]$pids) {
    $seed = @($pids | Where-Object { $_ -and $_ -gt 0 } | Select-Object -Unique)
    if ($seed.Count -eq 0) { return @() }

    $allQuant = Get-CimInstance Win32_Process -ErrorAction SilentlyContinue |
        Where-Object {
            $_.Name -like "python*" -and
            $_.CommandLine -and
            $_.CommandLine -like "*uvicorn*api.main:app*" -and
            $_.CommandLine -like "*--port $Port*"
        } |
        Select-Object ProcessId, ParentProcessId

    $closure = New-Object System.Collections.Generic.HashSet[int]
    $queue = New-Object System.Collections.Generic.Queue[int]
    foreach ($candidatePid in $seed) {
        if ($closure.Add([int]$candidatePid)) { $queue.Enqueue([int]$candidatePid) }
    }

    while ($queue.Count -gt 0) {
        $current = $queue.Dequeue()
        foreach ($proc in $allQuant) {
            $childPid = [int]$proc.ProcessId
            $parentPid = [int]$proc.ParentProcessId
            if ($childPid -eq $current -and $parentPid -gt 0 -and $closure.Add($parentPid)) {
                $queue.Enqueue($parentPid)
            }
            if ($parentPid -eq $current -and $childPid -gt 0 -and $closure.Add($childPid)) {
                $queue.Enqueue($childPid)
            }
        }
    }

    return @([int[]]$closure | Sort-Object)
}

function Stop-QuantProcesses([int[]]$pids, [int[]]$keepPids = @()) {
    $targets = @($pids | Where-Object { $_ -and ($_ -notin $keepPids) } | Select-Object -Unique)
    foreach ($targetPid in $targets) {
        try {
            Stop-Process -Id $targetPid -Force -ErrorAction Stop
            _OpsLog "Proceso Quant duplicado detenido: PID=$targetPid"
        } catch {
            _OpsLog ("No se pudo detener PID {0}: {1}" -f $targetPid, $_.Exception.Message) "warn"
        }
    }
    return $targets
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
function Invoke-QuantApi([string]$method, [string]$path, [hashtable]$body = @{}, [int]$timeoutSec = 10) {
    $url = "$_API_BASE$path"
    $headers = @{ "x-api-key" = $ApiKey; "Content-Type" = "application/json" }
    try {
        if ($method -eq "GET") {
            $r = Invoke-RestMethod -Uri $url -Method GET -Headers $headers -TimeoutSec $timeoutSec
        } else {
            $json = ($body | ConvertTo-Json -Compress)
            $r = Invoke-RestMethod -Uri $url -Method POST -Headers $headers -Body $json -TimeoutSec $timeoutSec
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

if ($FullStartup) {
    $env:QUANT_LIGHTWEIGHT_STARTUP = "false"
    _OpsLog "FullStartup solicitado; QUANT_LIGHTWEIGHT_STARTUP=false"
    Write-Host "[quant-start] Full startup solicitado."
} elseif ($Lightweight) {
    $env:QUANT_LIGHTWEIGHT_STARTUP = "true"
    $env:QUANT_STARTUP_VISUAL_CONNECT_ENABLED = "false"
    if (-not $env:QUANT_DASHBOARD_WS_LIMIT) { $env:QUANT_DASHBOARD_WS_LIMIT = "2" }
    if (-not $env:TRADIER_LIVE_UPDATE_INTERVAL_SEC) { $env:TRADIER_LIVE_UPDATE_INTERVAL_SEC = "10" }
    _OpsLog "Lightweight startup explícito (-Lightweight)"
    Write-Host "[quant-start] Modo -Lightweight: boot rápido; métricas ricas pueden verse incompletas hasta que termine el warm-up."
} elseif (-not $env:QUANT_LIGHTWEIGHT_STARTUP) {
    $env:QUANT_LIGHTWEIGHT_STARTUP = "false"
    _OpsLog "Arranque estándar: QUANT_LIGHTWEIGHT_STARTUP=false (métricas canónicas)"
    Write-Host "[quant-start] Arranque estándar (métricas completas). Usa -Lightweight solo si necesitas boot mínimo."
}
$lightweightRequested = ($env:QUANT_LIGHTWEIGHT_STARTUP -eq "true")

# ── Step 1: Check if already running ─────────────────────────────────────────
$portOwners = Get-PortOwners -p $Port
$quantCandidates = Get-QuantProcessCandidates -p $Port
$alreadyListening = Test-PortListen -p $Port
$healthOk = Test-HealthEndpoint -url $_API_HEALTH
$launchedThisRun = $false

if (-not $healthOk -and (@($portOwners).Count -gt 0 -or @($quantCandidates).Count -gt 0) -and -not $LogOnly) {
    $staleCandidates = Expand-QuantPidClosure -pids (@($portOwners) + @($quantCandidates))
    _OpsLog "Puerto $Port sin health; limpiando candidatos Quant: $($staleCandidates -join ', ')" "warn"
    Write-Host "[quant-start] Puerto $Port sin health. Limpiando candidatos Quant: $($staleCandidates -join ', ')"
    [void](Stop-QuantProcesses -pids $staleCandidates)
    Start-Sleep -Seconds 2
    $portOwners = Get-PortOwners -p $Port
    $quantCandidates = Get-QuantProcessCandidates -p $Port
    $alreadyListening = Test-PortListen -p $Port
    $healthOk = Test-HealthEndpoint -url $_API_HEALTH
}

if ($healthOk) {
    if ($portOwners.Count -gt 0 -and $quantCandidates.Count -gt 1) {
        $keepPid = @($portOwners | Select-Object -First 1)
        $keepClosure = Expand-QuantPidClosure -pids $keepPid
        $allClosure = Expand-QuantPidClosure -pids $quantCandidates
        $stopped = Stop-QuantProcesses -pids $allClosure -keepPids $keepClosure
        if ($stopped.Count -gt 0) {
            Write-Host "[quant-start] Limpieza preventiva de procesos Quant duplicados: $($stopped -join ', ')"
        }
    }
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

    _OpsLog "Lanzando uvicorn en puerto $Port (LightweightStartup=$($env:QUANT_LIGHTWEIGHT_STARTUP))"
    Write-Host "[quant-start] Lanzando uvicorn -- puerto $Port"
    try {
        "" | Out-File -FilePath $_UVICORN_STDOUT -Encoding utf8
        "" | Out-File -FilePath $_UVICORN_STDERR -Encoding utf8
    } catch {}
    $env:PYTHONPATH = $_QUANT_PYTHONPATH
    Start-Process -FilePath $_VENV_PY `
        -WorkingDirectory $_QUANT_DIR `
        -ArgumentList $uvicornArgs `
        -RedirectStandardOutput $_UVICORN_STDOUT `
        -RedirectStandardError $_UVICORN_STDERR `
        -WindowStyle Hidden `
        -PassThru | Out-Null
    _OpsLog "Logs uvicorn redirect -- stdout=$_UVICORN_STDOUT stderr=$_UVICORN_STDERR"
    $launchedThisRun = $true
    Start-Sleep -Milliseconds 500
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
            $portOwners = Get-PortOwners -p $Port
            $quantCandidates = Get-QuantProcessCandidates -p $Port
            if (-not $launchedThisRun -and $portOwners.Count -gt 0 -and $quantCandidates.Count -gt 1) {
                $keepPid = @($portOwners | Select-Object -First 1)
                $stopped = Stop-QuantProcesses -pids $quantCandidates -keepPids $keepPid
                if ($stopped.Count -gt 0) {
                    Write-Host "[quant-start] Procesos Quant duplicados limpiados tras health: $($stopped -join ', ')"
                }
            }
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
$opStatus = Invoke-QuantApi -method "GET" -path "/api/v2/quant/operation/status/lite" -timeoutSec 20
$loopStatus = Invoke-QuantApi -method "GET" -path "/api/v2/quant/operation/loop/status" -timeoutSec 20
if ($opStatus) {
    $autonMode = $opStatus.data.config.auton_mode
    $killSwitch = $opStatus.data.config.kill_switch_active
    $selectorMode = if ($null -ne $opStatus.data.selector_session) { $opStatus.data.selector_session.mode } else { $null }
    _OpsLog "Estado -- auton_mode=$autonMode kill_switch=$killSwitch selector_mode=$selectorMode"
    Write-Host "[quant-start] auton_mode=$autonMode  kill_switch=$killSwitch  selector_mode=$selectorMode"
    $loopRunning = $false
    if ($loopStatus -and $loopStatus.ok -and $loopStatus.data) {
        $loopRunning = [bool]$loopStatus.data.running
        _OpsLog "Loop status -- running=$loopRunning task_alive=$($loopStatus.data.task_alive) cycle_count=$($loopStatus.data.cycle_count)"
    } else {
        _OpsLog "No se pudo obtener operation/loop/status durante startup" "warn"
    }

    if ($AutoCycle -and $autonMode -and $autonMode -ne "off" -and -not $loopRunning) {
        $reconcileBody = @{ interval_sec = $CycleIntervalSec; max_per_cycle = 1 }
        if ($selectorMode) {
            $reconcileBody["selector_session_mode"] = $selectorMode
        }
        $loopResp = Invoke-QuantApi -method "POST" -path "/api/v2/quant/operation/loop/start" -body $reconcileBody
        if ($loopResp -and $loopResp.ok) {
            _OpsLog "Loop reconciliado en startup -- interval=${CycleIntervalSec}s selector_mode=$selectorMode"
            Write-Host "[quant-start] Loop reconciliado en startup. Intervalo: ${CycleIntervalSec}s"
        } else {
            $loopErr = if ($loopResp) { $loopResp.error } else { "sin respuesta" }
            _OpsLog "Loop NO pudo reconciliarse en startup: $loopErr" "warn"
            Write-Warning "[quant-start] Loop reconcile error: $loopErr"
        }
    } elseif (-not $AutoCycle) {
        _OpsLog "AutoCycle no solicitado: no se reconcilia operation loop en startup"
        Write-Host "[quant-start] AutoCycle no solicitado: no se inicia operation loop."
    }
} else {
    _OpsLog "No se pudo obtener operation/status/lite (API key o endpoint no disponible)" "warn"
}

# Vision status
$visionStatus = $null
if ($opStatus -and $opStatus.data -and $opStatus.data.vision) {
    $visionStatus = $opStatus.data.vision
}
if ($visionStatus) {
    $providerReady = $visionStatus.provider_ready
    $provider      = $visionStatus.provider
    _OpsLog "Vision -- provider=$provider provider_ready=$providerReady"
    Write-Host "[quant-start] Vision: provider=$provider  ready=$providerReady"
} elseif (-not $lightweightRequested -and -not ($env:QUANT_LIGHTWEIGHT_STARTUP -eq "true")) {
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

if (-not $SkipMonitoring -and -not $LogOnly) {
    if (Test-Path $_START_MONITORING) {
        try {
            _OpsLog "Asegurando monitoring stack (Prometheus/Grafana)..."
            Write-Host "[quant-start] Asegurando monitoring stack..."
            & $_START_MONITORING -QuantPort $Port -GrafanaPort 3002 -SkipOpenBrowser
            _OpsLog "Monitoring stack verificado"
        } catch {
            _OpsLog ("Monitoring stack con incidencias: {0}" -f $_.Exception.Message) "warn"
            Write-Warning "[quant-start] Monitoring stack con incidencias: $($_.Exception.Message)"
        }
    } else {
        _OpsLog "Script start_monitoring.ps1 no encontrado; se omite verificacion de monitoring" "warn"
    }
}

_OpsLog "Startup completado -- puerto $Port"
_DiagLog "FINISH status=ok port=$Port AutoCycle=$AutoCycle"
Write-Host "[quant-start] Completado. Servidor en http://127.0.0.1:$Port/ui"
exit 0

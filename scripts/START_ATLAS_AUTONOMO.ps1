<#
.SYNOPSIS
    ATLAS-Quant — Arranque autónomo completo para sesión de trading.

.DESCRIPTION
    Secuencia completa:
    1. Verifica credenciales Tradier paper
    2. Levanta servidor Quant en puerto 8795
    3. Espera health OK
    4. Confirma auton_mode = paper_autonomous
    5. Activa auto-cycle loop (interval=90s)
    6. Muestra tablero de estado final

    Paper trading: cuenta sandbox Tradier (sin dinero real).
    Modo: paper_autonomous → el loop evalúa Y envía órdenes al paper broker.

.EXAMPLE
    .\scripts\START_ATLAS_AUTONOMO.ps1
    .\scripts\START_ATLAS_AUTONOMO.ps1 -CycleIntervalSec 120
    .\scripts\START_ATLAS_AUTONOMO.ps1 -DryRun
#>
param(
    [int]$Port               = 8795,
    [string]$ApiKey          = "atlas-quant-local",
    [int]$CycleIntervalSec   = 90,
    [int]$MaxWaitSec         = 90,
    [switch]$DryRun
)

$ErrorActionPreference = "Stop"
$_ROOT     = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$_VENV_PY  = Join-Path $_ROOT "venv\Scripts\python.exe"
$_QUANT    = Join-Path $_ROOT "atlas_code_quant"
$_LOGS     = Join-Path $_ROOT "logs"
$_OPS_BUS  = Join-Path $_LOGS "ops_bus.log"
$_API      = "http://127.0.0.1:$Port"
$_CREDS    = "C:\dev\credenciales.txt"

if (-not (Test-Path $_LOGS)) { New-Item -ItemType Directory -Path $_LOGS -Force | Out-Null }

function _Ts { (Get-Date).ToUniversalTime().ToString("yyyy-MM-ddTHH:mm:ssZ") }
function _Log([string]$msg, [string]$color = "Cyan") {
    $ts = (Get-Date).ToString("HH:mm:ss")
    Write-Host "[$ts] $msg" -ForegroundColor $color
    try { "$(_Ts) [INFO ] [AUTONOMO] $msg" | Out-File -FilePath $_OPS_BUS -Append -Encoding utf8 } catch {}
}
function _OK([string]$msg)   { Write-Host "  [OK] $msg" -ForegroundColor Green }
function _WARN([string]$msg) { Write-Host "  [!!] $msg" -ForegroundColor Yellow }
function _ERR([string]$msg)  { Write-Host "  [XX] $msg" -ForegroundColor Red }

function Test-Health([string]$url) {
    try {
        $r = [System.Net.WebRequest]::Create($url)
        $r.Timeout = 5000; $r.Method = "GET"
        $resp = $r.GetResponse()
        $ok = ([int]$resp.StatusCode -eq 200)
        $resp.Close(); return $ok
    } catch { return $false }
}

function Invoke-Api([string]$method, [string]$path, [hashtable]$body = @{}) {
    $url = "$_API$path"
    $headers = @{ "x-api-key" = $ApiKey; "Content-Type" = "application/json" }
    try {
        if ($method -eq "GET") {
            return Invoke-RestMethod -Uri $url -Method GET -Headers $headers -TimeoutSec 10
        } else {
            $json = ($body | ConvertTo-Json -Compress)
            return Invoke-RestMethod -Uri $url -Method POST -Headers $headers -Body $json -TimeoutSec 10
        }
    } catch { return $null }
}

# ─────────────────────────────────────────────────────────────────────────────
Write-Host ""
Write-Host "  ╔══════════════════════════════════════════════════╗" -ForegroundColor Cyan
Write-Host "  ║       ATLAS-Quant  MODO AUTÓNOMO  2026-03-24    ║" -ForegroundColor Cyan
Write-Host "  ║          Paper Trading — Tradier Sandbox         ║" -ForegroundColor Cyan
Write-Host "  ╚══════════════════════════════════════════════════╝" -ForegroundColor Cyan
Write-Host ""

if ($DryRun) { _WARN "DryRun=true — no se lanzará servidor ni se activará loop" }

# ── PASO 1: Verificar credenciales ───────────────────────────────────────────
_Log "PASO 1: Verificando credenciales Tradier paper..."
if (Test-Path $_CREDS) {
    $credContent = Get-Content $_CREDS -Raw
    if ($credContent -match "PAPER_ACCOUNT" -and $credContent -match "ACCESS_TOKEN") {
        _OK "Credenciales paper encontradas en $( $_CREDS)"
    } else {
        _WARN "Archivo de credenciales existe pero no tiene sección [PAPER_ACCOUNT] completa"
    }
} else {
    _ERR "No se encontró C:\dev\credenciales.txt — el broker no tendrá token"
    _WARN "Continúa en modo simulación local (sin broker real)"
}

# ── PASO 2: Verificar / Levantar servidor ────────────────────────────────────
_Log "PASO 2: Verificando servidor Quant en puerto $Port..."

$healthOk = Test-Health "$_API/health"
if ($healthOk) {
    _OK "Servidor ya activo en $Port"
} elseif ($DryRun) {
    _WARN "DryRun: se omitiría lanzar servidor"
} else {
    if (-not (Test-Path $_VENV_PY)) {
        _ERR "Python venv no encontrado: $_VENV_PY"
        exit 1
    }
    _Log "Lanzando uvicorn en puerto $Port..."
    Start-Process -FilePath $_VENV_PY `
        -ArgumentList @("-m","uvicorn","api.main:app","--host","0.0.0.0","--port","$Port") `
        -WorkingDirectory $_QUANT `
        -NoNewWindow `
        -PassThru | Out-Null

    _Log "Esperando health endpoint (max ${MaxWaitSec}s)..."
    $waited = 0
    while ($waited -lt $MaxWaitSec) {
        Start-Sleep -Seconds 4
        $waited += 4
        $healthOk = Test-Health "$_API/health"
        if ($healthOk) {
            _OK "Servidor respondio tras ${waited}s"
            break
        }
        Write-Host "     ... ${waited}/${MaxWaitSec}s" -ForegroundColor DarkGray
    }
    if (-not $healthOk) {
        _ERR "Servidor no respondio en ${MaxWaitSec}s. Verifica: cd atlas_code_quant && ..\venv\Scripts\uvicorn api.main:app --port $Port"
        exit 2
    }
}

# ── PASO 3: Verificar + forzar auton_mode=paper_autonomous ───────────────────
_Log "PASO 3: Verificando modo autónomo..."
$opStatus = Invoke-Api -method "GET" -path "/api/v2/quant/operation/status"
if ($opStatus -and $opStatus.data) {
    $currentMode = $opStatus.data.auton_mode
    _Log "  auton_mode actual: $currentMode"

    if ($currentMode -ne "paper_autonomous") {
        _Log "  Cambiando a paper_autonomous via API..."
        $patch = Invoke-Api -method "POST" -path "/api/v2/quant/operation/config" `
            -body @{ auton_mode = "paper_autonomous" }
        if ($patch -and $patch.ok) {
            _OK "auton_mode = paper_autonomous activado"
        } else {
            _WARN "API config no respondio — estado ya estaba en JSON, el API leerá al cargar"
        }
    } else {
        _OK "auton_mode = paper_autonomous (ya configurado)"
    }

    $scanRunning = $opStatus.data.scanner.running
    $visionMode  = $opStatus.data.vision_mode
    _Log "  scanner_running=$scanRunning  vision_mode=$visionMode"
} else {
    _WARN "No se pudo consultar operation/status"
}

# ── PASO 4: Verificar vision (desktop_capture, no bloquea sin NEXUS) ─────────
_Log "PASO 4: Verificando vision..."
$vis = Invoke-Api -method "GET" -path "/api/v2/quant/operation/vision"
if ($vis -and $vis.data) {
    $providerReady = $vis.data.status.provider_ready
    $provider = $vis.data.status.provider
    _Log "  vision provider=$provider  ready=$providerReady"
    if (-not $providerReady -and $provider -eq "direct_nexus") {
        _Log "  Cambiando vision a desktop_capture (NEXUS robot no necesario)..."
        $vr = Invoke-Api -method "POST" -path "/api/v2/quant/operation/vision/provider" `
            -body @{ provider = "desktop_capture" }
        if ($vr -and $vr.ok) { _OK "Vision = desktop_capture" }
        else { _WARN "No se pudo cambiar vision — usando estado del JSON" }
    } elseif ($providerReady) {
        _OK "Vision ready: $provider"
    }
}

# ── PASO 5: Activar auto-cycle loop ──────────────────────────────────────────
_Log "PASO 5: Activando auto-cycle loop (interval=${CycleIntervalSec}s)..."
if ($DryRun) {
    _WARN "DryRun: se omitiría activar loop"
} else {
    # Detener loop si ya corría (para reiniciar con parámetros frescos)
    Invoke-Api -method "POST" -path "/api/v2/quant/operation/loop/stop" | Out-Null
    Start-Sleep -Seconds 1

    $loopResp = Invoke-Api -method "POST" -path "/api/v2/quant/operation/loop/start" `
        -body @{ interval_sec = $CycleIntervalSec; max_per_cycle = 1 }
    if ($loopResp -and $loopResp.ok) {
        _OK "Auto-cycle loop activo — intervalo ${CycleIntervalSec}s — action=SUBMIT"
    } else {
        _ERR "Error activando loop: $( if($loopResp) { $loopResp.error } else { 'sin respuesta' } )"
    }
}

# ── PASO 6: Estado final ──────────────────────────────────────────────────────
_Log "PASO 6: Estado final del sistema..."
Start-Sleep -Seconds 2
$loopStatus = Invoke-Api -method "GET" -path "/api/v2/quant/operation/loop/status"
$opFinal    = Invoke-Api -method "GET" -path "/api/v2/quant/operation/status"

Write-Host ""
Write-Host "  ┌─────────────────────────────────────────────┐" -ForegroundColor Green
Write-Host "  │          ATLAS AUTÓNOMO — ESTADO FINAL       │" -ForegroundColor Green
Write-Host "  ├─────────────────────────────────────────────┤" -ForegroundColor Green
if ($opFinal -and $opFinal.data) {
    $d = $opFinal.data
    Write-Host ("  │  auton_mode   : {0,-30}│" -f $d.auton_mode) -ForegroundColor Green
    Write-Host ("  │  account_scope: {0,-30}│" -f $d.account_scope) -ForegroundColor Green
    Write-Host ("  │  vision_mode  : {0,-30}│" -f $d.vision_mode) -ForegroundColor Green
    Write-Host ("  │  scanner      : {0,-30}│" -f $(if($d.scanner.running){"running"}else{"stopped"})) -ForegroundColor Green
}
if ($loopStatus -and $loopStatus.data) {
    $l = $loopStatus.data
    Write-Host ("  │  loop         : {0,-30}│" -f $(if($l.running){"ACTIVO (interval=$($l.loop_interval_sec)s)"}else{"detenido"})) -ForegroundColor Green
    Write-Host ("  │  ciclos       : {0,-30}│" -f $l.cycle_count) -ForegroundColor Green
}
Write-Host "  ├─────────────────────────────────────────────┤" -ForegroundColor Green
Write-Host "  │  Dashboard: http://127.0.0.1:$Port/ui          │" -ForegroundColor Green
Write-Host "  │  Log live : logs/atlas_live_loop.log         │" -ForegroundColor Green
Write-Host "  │  Ops bus  : logs/ops_bus.log                 │" -ForegroundColor Green
Write-Host "  └─────────────────────────────────────────────┘" -ForegroundColor Green
Write-Host ""
_Log "ATLAS autónomo listo. Para monitorear:" "Green"
Write-Host "    Get-Content logs\atlas_live_loop.log -Wait -Tail 20" -ForegroundColor Yellow
Write-Host "    Get-Content logs\ops_bus.log -Wait -Tail 20" -ForegroundColor Yellow
Write-Host ""

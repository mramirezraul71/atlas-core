<#
.SYNOPSIS
    ATLAS-Quant - arranque autonomo completo para sesion de trading.

.DESCRIPTION
    Secuencia completa:
    1. Verifica credenciales Tradier paper
    2. Levanta servidor Quant en puerto 8795
    3. Espera health OK
    4. Confirma auton_mode = paper_autonomous
    5. Resetea emergency stop si esta activo
    6. Activa auto-cycle loop
    7. Muestra tablero de estado final

.EXAMPLE
    .\scripts\START_ATLAS_AUTONOMO.ps1

.EXAMPLE
    .\scripts\START_ATLAS_AUTONOMO.ps1 -CycleIntervalSec 120

.EXAMPLE
    .\scripts\START_ATLAS_AUTONOMO.ps1 -DryRun
#>
param(
    [int]$Port = 8795,
    [string]$ApiKey = "atlas-quant-local",
    [int]$CycleIntervalSec = 90,
    [int]$MaxWaitSec = 90,
    [switch]$DryRun
)

$ErrorActionPreference = "Stop"
$_ROOT = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$_QUANT_START = Join-Path $_ROOT "scripts\atlas_quant_start.ps1"
$_LOGS = Join-Path $_ROOT "logs"
$_OPS_BUS = Join-Path $_LOGS "ops_bus.log"
$_API = "http://127.0.0.1:$Port"
$_CREDS = "C:\dev\credenciales.txt"

if (-not (Test-Path $_LOGS)) {
    New-Item -ItemType Directory -Path $_LOGS -Force | Out-Null
}

function _Ts {
    (Get-Date).ToUniversalTime().ToString("yyyy-MM-ddTHH:mm:ssZ")
}

function _Log([string]$Message, [string]$Color = "Cyan") {
    $ts = (Get-Date).ToString("HH:mm:ss")
    Write-Host "[$ts] $Message" -ForegroundColor $Color
    try {
        "$(_Ts) [INFO ] [AUTONOMO] $Message" | Out-File -FilePath $_OPS_BUS -Append -Encoding utf8
    } catch {}
}

function _OK([string]$Message) {
    Write-Host "  [OK] $Message" -ForegroundColor Green
}

function _WARN([string]$Message) {
    Write-Host "  [!!] $Message" -ForegroundColor Yellow
}

function _ERR([string]$Message) {
    Write-Host "  [XX] $Message" -ForegroundColor Red
}

function Test-Health([string]$Url) {
    try {
        $request = [System.Net.WebRequest]::Create($Url)
        $request.Timeout = 5000
        $request.Method = "GET"
        $response = $request.GetResponse()
        $ok = ([int]$response.StatusCode -eq 200)
        $response.Close()
        return $ok
    } catch {
        return $false
    }
}

function Invoke-Api([string]$Method, [string]$Path, [hashtable]$Body = @{}) {
    $url = "$_API$Path"
    $headers = @{ "x-api-key" = $ApiKey; "Content-Type" = "application/json" }
    try {
        if ($Method -eq "GET") {
            return Invoke-RestMethod -Uri $url -Method GET -Headers $headers -TimeoutSec 15
        }
        $json = ($Body | ConvertTo-Json -Compress)
        return Invoke-RestMethod -Uri $url -Method POST -Headers $headers -Body $json -TimeoutSec 15
    } catch {
        return $null
    }
}

Write-Host ""
Write-Host "======================================================" -ForegroundColor Cyan
Write-Host " ATLAS-QUANT AUTONOMO - PAPER TRADING" -ForegroundColor Cyan
Write-Host "======================================================" -ForegroundColor Cyan
Write-Host ""

if ($DryRun) {
    _WARN "DryRun=true - no se lanzara servidor ni se activara loop"
}

_Log "PASO 1: Verificando credenciales Tradier paper..."
if (Test-Path $_CREDS) {
    $credContent = Get-Content $_CREDS -Raw
    if ($credContent -match "PAPER_ACCOUNT" -and $credContent -match "ACCESS_TOKEN") {
        _OK "Credenciales paper encontradas en $_CREDS"
    } else {
        _WARN "El archivo existe pero no contiene la seccion [PAPER_ACCOUNT] completa"
    }
} else {
    _ERR "No se encontro C:\dev\credenciales.txt"
    _WARN "Continuando en modo simulacion local"
}

_Log "PASO 2: Verificando servidor Quant en puerto $Port..."
$healthOk = Test-Health "$_API/health"
if ($healthOk) {
    _OK "Servidor Quant ya activo en $Port"
} elseif ($DryRun) {
    _WARN "DryRun: se omite lanzar servidor"
} else {
    if (-not (Test-Path $_QUANT_START)) {
        _ERR "Script de arranque Quant no encontrado: $_QUANT_START"
        exit 1
    }
    _Log "Lanzando servidor Quant con atlas_quant_start.ps1..."
    & $_QUANT_START -Port $Port -ApiKey $ApiKey -MaxWaitSec $MaxWaitSec
    $healthOk = ($LASTEXITCODE -eq 0) -and (Test-Health "$_API/health")
    if (-not $healthOk) {
        _ERR "Servidor Quant no respondio en ${MaxWaitSec}s"
        exit 2
    }
    _OK "Servidor Quant operativo en $Port"
}

_Log "PASO 3: Verificando operation/status..."
$opStatus = Invoke-Api -Method "GET" -Path "/api/v2/quant/operation/status"
if ($opStatus -and $opStatus.data) {
    $currentMode = $opStatus.data.config.auton_mode
    $killSwitchActive = [bool]$opStatus.data.config.kill_switch_active
    $visionMode = $opStatus.data.config.vision_mode
    _Log "  auton_mode actual: $currentMode"
    _Log "  kill_switch_active: $killSwitchActive"
    _Log "  vision_mode: $visionMode"

    if ($currentMode -ne "paper_autonomous") {
        _Log "  Cambiando a paper_autonomous via API..."
        $patch = Invoke-Api -Method "POST" -Path "/api/v2/quant/operation/config" -Body @{ auton_mode = "paper_autonomous" }
        if ($patch -and $patch.ok) {
            _OK "auton_mode = paper_autonomous activado"
        } else {
            _WARN "No se pudo confirmar el cambio de auton_mode"
        }
    } else {
        _OK "auton_mode = paper_autonomous"
    }

    if ($killSwitchActive -and -not $DryRun) {
        _Log "  Reseteando emergency stop via API..."
        $reset = Invoke-Api -Method "POST" -Path "/api/v2/quant/emergency/reset"
        if ($reset -and $reset.ok) {
            _OK "Emergency stop reseteado"
        } else {
            _WARN "No se pudo resetear el emergency stop"
        }
    } elseif ($killSwitchActive) {
        _WARN "Kill switch activo; DryRun no lo resetea"
    }
} else {
    _WARN "No se pudo consultar operation/status"
}

_Log "PASO 4: Verificando vision..."
$visionPayload = Invoke-Api -Method "GET" -Path "/api/v2/quant/operation/vision"
if ($visionPayload -and $visionPayload.data) {
    $providerReady = [bool]$visionPayload.data.status.provider_ready
    $provider = $visionPayload.data.status.provider
    _Log "  provider=$provider ready=$providerReady"
    if (-not $providerReady -and $provider -eq "direct_nexus") {
        _Log "  Cambiando a desktop_capture..."
        $providerUpdate = Invoke-Api -Method "POST" -Path "/api/v2/quant/operation/vision/provider" -Body @{ provider = "desktop_capture" }
        if ($providerUpdate -and $providerUpdate.ok) {
            _OK "Vision = desktop_capture"
        } else {
            _WARN "No se pudo cambiar el provider de vision"
        }
    } elseif ($providerReady) {
        _OK "Vision ready: $provider"
    } else {
        _WARN "Vision no esta lista: $provider"
    }
}

_Log "PASO 5: Activando auto-cycle loop (interval=${CycleIntervalSec}s)..."
if ($DryRun) {
    _WARN "DryRun: se omite activar loop"
} else {
    Invoke-Api -Method "POST" -Path "/api/v2/quant/operation/loop/stop" | Out-Null
    Start-Sleep -Seconds 1

    $loopResp = Invoke-Api -Method "POST" -Path "/api/v2/quant/operation/loop/start" -Body @{
        interval_sec = $CycleIntervalSec
        max_per_cycle = 1
    }
    if ($loopResp -and $loopResp.ok) {
        _OK "Auto-cycle loop activo"
    } else {
        $loopError = if ($loopResp) { $loopResp.error } else { "sin respuesta" }
        _ERR "Error activando loop: $loopError"
    }
}

_Log "PASO 6: Estado final del sistema..."
Start-Sleep -Seconds 2
$loopStatus = Invoke-Api -Method "GET" -Path "/api/v2/quant/operation/loop/status"
$opFinal = Invoke-Api -Method "GET" -Path "/api/v2/quant/operation/status"

Write-Host ""
Write-Host "======================================================" -ForegroundColor Green
Write-Host " ATLAS AUTONOMO - ESTADO FINAL" -ForegroundColor Green
Write-Host "======================================================" -ForegroundColor Green
if ($opFinal -and $opFinal.data) {
    $config = $opFinal.data.config
    Write-Host ("  auton_mode    : {0}" -f $config.auton_mode) -ForegroundColor Green
    Write-Host ("  account_scope : {0}" -f $config.account_scope) -ForegroundColor Green
    Write-Host ("  vision_mode   : {0}" -f $config.vision_mode) -ForegroundColor Green
    Write-Host ("  kill_switch   : {0}" -f $config.kill_switch_active) -ForegroundColor Green
}
if ($loopStatus -and $loopStatus.data) {
    $loopData = $loopStatus.data
    Write-Host ("  loop_running  : {0}" -f $loopData.running) -ForegroundColor Green
    Write-Host ("  interval_sec  : {0}" -f $loopData.loop_interval_sec) -ForegroundColor Green
    Write-Host ("  cycle_count   : {0}" -f $loopData.cycle_count) -ForegroundColor Green
}
Write-Host "  dashboard     : http://127.0.0.1:$Port/ui" -ForegroundColor Green
Write-Host "  ops_bus       : logs/ops_bus.log" -ForegroundColor Green
Write-Host "======================================================" -ForegroundColor Green
Write-Host ""

_Log "ATLAS autonomo listo. Para monitorear:" "Green"
Write-Host "    Get-Content logs\atlas_live_loop.log -Wait -Tail 20" -ForegroundColor Yellow
Write-Host "    Get-Content logs\ops_bus.log -Wait -Tail 20" -ForegroundColor Yellow
Write-Host ""

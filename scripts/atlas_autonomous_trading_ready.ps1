<#
.SYNOPSIS
    Runbook maestro para dejar ATLAS en modo de trading autonomo operativo.

.DESCRIPTION
    Orquesta el arranque y verificacion de:
    - NEXUS API (8000)
    - Robot backend (8002)
    - PUSH / dashboard (8791)
    - Quant API + auto-cycle (8795)
    - Metrics + Prometheus + Grafana local (9090/9091/3003)

    Reutiliza los entrypoints ya validados del repo para evitar duplicar logica.

.EXAMPLE
    .\scripts\atlas_autonomous_trading_ready.ps1

.EXAMPLE
    .\scripts\atlas_autonomous_trading_ready.ps1 -StatusOnly

.EXAMPLE
    .\scripts\atlas_autonomous_trading_ready.ps1 -ForcePushStart -OpenDashboards
#>
param(
    [int]$QuantPort = 8795,
    [string]$QuantApiKey = "atlas-quant-local",
    [int]$CycleIntervalSec = 90,
    [int]$MaxWaitSec = 90,
    [switch]$ForcePushStart,
    [switch]$OpenDashboards,
    [switch]$StatusOnly
)

$ErrorActionPreference = "Stop"

$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$VenvPy = Join-Path $RepoRoot "venv\Scripts\python.exe"
$StartNexus = Join-Path $RepoRoot "scripts\start_nexus_services.py"
$StartMonitoring = Join-Path $RepoRoot "scripts\start_monitoring.ps1"
$StartQuantAutonomous = Join-Path $RepoRoot "scripts\START_ATLAS_AUTONOMO.ps1"
$StartPush = Join-Path $RepoRoot "tools\service_launcher.py"
$OpsBus = Join-Path $RepoRoot "logs\ops_bus.log"

$NexusUrl = "http://127.0.0.1:8000/health"
$RobotUrl = "http://127.0.0.1:8002/api/health"
$PushUrl = "http://127.0.0.1:8791/health"
$QuantUrl = "http://127.0.0.1:$QuantPort/health"
$PromUrl = "http://127.0.0.1:9091/-/healthy"
$GrafanaUrl = "http://127.0.0.1:3003/api/health"

function Write-Step([string]$Message) {
    Write-Host "`n[STEP] $Message" -ForegroundColor Cyan
}

function Write-Ok([string]$Message) {
    Write-Host "  [OK]  $Message" -ForegroundColor Green
}

function Write-Warn([string]$Message) {
    Write-Host "  [!!]  $Message" -ForegroundColor Yellow
}

function Write-Fail([string]$Message) {
    Write-Host "  [XX]  $Message" -ForegroundColor Red
}

function Write-Info([string]$Message) {
    Write-Host "  [--]  $Message" -ForegroundColor DarkGray
}

function Log-Ops([string]$Message) {
    try {
        $ts = (Get-Date).ToUniversalTime().ToString("yyyy-MM-ddTHH:mm:ssZ")
        "$ts [INFO ] [ATLAS_READY] $Message" | Out-File -FilePath $OpsBus -Append -Encoding utf8
    } catch {}
}

function Test-PortListening([int]$Port) {
    $match = netstat -ano | Select-String ":$Port" | Select-String "LISTENING" | Select-Object -First 1
    return ($null -ne $match)
}

function Test-HttpOk([string]$Url, [int]$TimeoutSec = 5) {
    try {
        $resp = Invoke-WebRequest -Uri $Url -UseBasicParsing -TimeoutSec $TimeoutSec
        return ($resp.StatusCode -ge 200 -and $resp.StatusCode -lt 300)
    } catch {
        return $false
    }
}

function Wait-HttpOk([string]$Url, [int]$MaxWaitSec = 45, [string]$Label = "servicio") {
    $elapsed = 0
    while ($elapsed -lt $MaxWaitSec) {
        if (Test-HttpOk -Url $Url) {
            return $true
        }
        Start-Sleep -Seconds 3
        $elapsed += 3
        Write-Info "Esperando $Label ($elapsed/$MaxWaitSec s)"
    }
    return $false
}

function Show-ServiceState([string]$Name, [int]$Port, [string]$Url) {
    $portOk = Test-PortListening -Port $Port
    $httpOk = Test-HttpOk -Url $Url -TimeoutSec 4
    if ($portOk -or $httpOk) {
        Write-Host ("  {0,-12} :{1,-5} ONLINE" -f $Name, $Port) -ForegroundColor Green
    } else {
        Write-Host ("  {0,-12} :{1,-5} OFFLINE" -f $Name, $Port) -ForegroundColor Red
    }
}

function Ensure-RepoEnv {
    Set-Location $RepoRoot
    $env:ATLAS_PYTHON = $VenvPy
    $env:PYTHONIOENCODING = "utf-8"
    $env:PYTHONUTF8 = "1"
    $env:NEXUS_ATLAS_PATH = Join-Path $RepoRoot "nexus\atlas_nexus"
    $env:NEXUS_ROBOT_PATH = Join-Path $RepoRoot "nexus\atlas_nexus_robot\backend"
}

function Start-PushIfNeeded {
    if (Test-HttpOk -Url $PushUrl) {
        Write-Ok "PUSH ya responde en 8791"
        return
    }

    if (-not $ForcePushStart) {
        Write-Warn "PUSH no responde en 8791 y no se forzo arranque."
        Write-Info "Usa -ForcePushStart para levantarlo desde el runbook."
        return
    }

    if (-not (Test-Path $StartPush)) {
        throw "No se encontro tools\service_launcher.py"
    }

    Write-Info "Arrancando PUSH en segundo plano..."
    $env:PYTHONPATH = $RepoRoot
    $env:SERVICE_PORT = "8791"
    Start-Process -FilePath $VenvPy `
        -ArgumentList $StartPush `
        -WorkingDirectory $RepoRoot `
        -WindowStyle Minimized | Out-Null

    if (Wait-HttpOk -Url $PushUrl -MaxWaitSec 45 -Label "PUSH :8791") {
        Write-Ok "PUSH operativo en 8791"
    } else {
        Write-Fail "PUSH no respondio tras el arranque"
    }
}

function Show-FinalSummary {
    Write-Host "`n============================================================" -ForegroundColor Green
    Write-Host " ATLAS AUTONOMOUS TRADING READY - RESUMEN FINAL" -ForegroundColor Green
    Write-Host "============================================================" -ForegroundColor Green
    Show-ServiceState -Name "NEXUS" -Port 8000 -Url $NexusUrl
    Show-ServiceState -Name "PUSH" -Port 8791 -Url $PushUrl
    Show-ServiceState -Name "ROBOT" -Port 8002 -Url $RobotUrl
    Show-ServiceState -Name "QUANT" -Port $QuantPort -Url $QuantUrl
    Show-ServiceState -Name "METRICS" -Port 9090 -Url "http://127.0.0.1:9090/metrics"
    Show-ServiceState -Name "PROM" -Port 9091 -Url $PromUrl
    Show-ServiceState -Name "GRAFANA" -Port 3003 -Url $GrafanaUrl
    Write-Host "============================================================" -ForegroundColor Green
    Write-Host " PUSH UI   : http://127.0.0.1:8791/ui"
    Write-Host " Quant UI  : http://127.0.0.1:$QuantPort/ui"
    Write-Host " Grafana   : http://127.0.0.1:3003"
    Write-Host " Ops bus   : logs/ops_bus.log"
    Write-Host "============================================================" -ForegroundColor Green
}

Ensure-RepoEnv
Log-Ops "Runbook maestro invocado (StatusOnly=$StatusOnly ForcePushStart=$ForcePushStart)"

Write-Host ""
Write-Host "============================================================" -ForegroundColor Cyan
Write-Host " ATLAS AUTONOMOUS TRADING READY" -ForegroundColor Cyan
Write-Host " Repo: $RepoRoot" -ForegroundColor DarkGray
Write-Host "============================================================" -ForegroundColor Cyan

Write-Step "Estado inicial"
Show-ServiceState -Name "NEXUS" -Port 8000 -Url $NexusUrl
Show-ServiceState -Name "PUSH" -Port 8791 -Url $PushUrl
Show-ServiceState -Name "ROBOT" -Port 8002 -Url $RobotUrl
Show-ServiceState -Name "QUANT" -Port $QuantPort -Url $QuantUrl
Show-ServiceState -Name "GRAFANA" -Port 3003 -Url $GrafanaUrl

if ($StatusOnly) {
    Write-Info "StatusOnly=true: no se arranca ningun servicio."
    exit 0
}

if (-not (Test-Path $VenvPy)) {
    throw "Python del venv no encontrado: $VenvPy"
}
if (-not (Test-Path $StartNexus)) {
    throw "No se encontro scripts\start_nexus_services.py"
}
if (-not (Test-Path $StartMonitoring)) {
    throw "No se encontro scripts\start_monitoring.ps1"
}
if (-not (Test-Path $StartQuantAutonomous)) {
    throw "No se encontro scripts\START_ATLAS_AUTONOMO.ps1"
}

Write-Step "NEXUS + Robot"
$nexusStart = & $VenvPy $StartNexus --include-nexus 2>&1
Write-Info ("start_nexus_services.py => " + (($nexusStart | Out-String).Trim()))
if (Wait-HttpOk -Url $NexusUrl -MaxWaitSec $MaxWaitSec -Label "NEXUS :8000") {
    Write-Ok "NEXUS operativo en 8000"
} else {
    Write-Fail "NEXUS no respondio en 8000"
}
if (Wait-HttpOk -Url $RobotUrl -MaxWaitSec $MaxWaitSec -Label "Robot :8002") {
    Write-Ok "Robot operativo en 8002"
} else {
    Write-Fail "Robot no respondio en 8002"
}

Write-Step "PUSH"
Start-PushIfNeeded

Write-Step "Quant autonomo"
& $StartQuantAutonomous -Port $QuantPort -ApiKey $QuantApiKey -CycleIntervalSec $CycleIntervalSec -MaxWaitSec $MaxWaitSec
if ($LASTEXITCODE -ne 0) {
    Write-Fail "START_ATLAS_AUTONOMO.ps1 devolvio codigo $LASTEXITCODE"
} elseif (Wait-HttpOk -Url $QuantUrl -MaxWaitSec 15 -Label "Quant :$QuantPort") {
    Write-Ok "Quant autonomo operativo en $QuantPort"
} else {
    Write-Fail "Quant no respondio tras el arranque autonomo"
}

Write-Step "Monitoring canonico"
& $StartMonitoring
if ($LASTEXITCODE -eq 0 -or (Test-HttpOk -Url $GrafanaUrl -TimeoutSec 5)) {
    Write-Ok "Monitoring listo (9090/9091/3003)"
} else {
    Write-Fail "Monitoring no quedo sano"
}

Write-Step "Health checks finales"
foreach ($pair in @(
    @{ Name = "NEXUS"; Url = $NexusUrl },
    @{ Name = "PUSH"; Url = $PushUrl },
    @{ Name = "ROBOT"; Url = $RobotUrl },
    @{ Name = "QUANT"; Url = $QuantUrl },
    @{ Name = "PROM"; Url = $PromUrl },
    @{ Name = "GRAFANA"; Url = $GrafanaUrl }
)) {
    if (Test-HttpOk -Url $pair.Url -TimeoutSec 5) {
        Write-Ok "$($pair.Name) responde OK"
    } else {
        Write-Warn "$($pair.Name) no responde OK"
    }
}

Show-FinalSummary

if ($OpenDashboards) {
    Write-Step "Abriendo paneles"
    Start-Process "http://127.0.0.1:8791/ui"
    Start-Process "http://127.0.0.1:$QuantPort/ui"
    Start-Process "http://127.0.0.1:3003"
}

Log-Ops "Runbook maestro completado"

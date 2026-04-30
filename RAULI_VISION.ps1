<#
.SYNOPSIS
    Un solo clic: arranca RAULI-VISION completo (espejo + dashboard + watchdog).
    Ejecuta como Administrador la primera vez para registrar la tarea de inicio.
#>

$WatchdogScript = "$PSScriptRoot\scripts\rauli_vision_watchdog.ps1"
$TaskName       = "RAULI-VISION-Watchdog"
$EspejoPort     = 8080   # espejo.exe — Go backend
$ProxyPort      = 3000   # simple-server.py — Python proxy → :8080
$DashPort       = 5174   # dashboard React/Vite
$EspejoHealth   = "http://127.0.0.1:$EspejoPort/api/health"
$ProxyHealth    = "http://127.0.0.1:$ProxyPort/health"
$DashHealth     = "http://127.0.0.1:$DashPort/"
$LogFile        = "$PSScriptRoot\logs\rauli_vision_watchdog.log"

function Write-Step([string]$msg) {
    Write-Host "[$(Get-Date -Format 'HH:mm:ss')] $msg" -ForegroundColor Cyan
}
function Write-Ok([string]$msg) {
    Write-Host "[$(Get-Date -Format 'HH:mm:ss')] OK $msg" -ForegroundColor Green
}
function Write-Warn([string]$msg) {
    Write-Host "[$(Get-Date -Format 'HH:mm:ss')] ! $msg" -ForegroundColor Yellow
}

function Test-Health([string]$url, [int]$timeoutSec = 5) {
    try {
        $r = Invoke-WebRequest -Uri $url -UseBasicParsing -TimeoutSec $timeoutSec
        return ($r.StatusCode -ge 200 -and $r.StatusCode -lt 400)
    } catch {
        return $false
    }
}

# ── Verificar si el watchdog ya corre (Get-Process no expone CommandLine en WinPS 5.1)
$wdRunning = Get-CimInstance Win32_Process -ErrorAction SilentlyContinue |
    Where-Object {
        ($_.Name -eq 'powershell.exe' -or $_.Name -eq 'pwsh.exe') -and
        $_.CommandLine -and ($_.CommandLine -like '*rauli_vision_watchdog*')
    }

if ($wdRunning) {
    $wdPids = @($wdRunning | ForEach-Object { $_.ProcessId }) -join ','
    Write-Ok "Watchdog ya esta corriendo (PID=$wdPids)"
} else {
    Write-Step "Lanzando watchdog en background..."
    Start-Process powershell.exe `
        -ArgumentList "-NonInteractive -WindowStyle Hidden -File `"$WatchdogScript`"" `
        -WindowStyle Hidden
    Write-Ok "Watchdog iniciado"
}

# ── Esperar que los puertos abran (max 35s) ────────────────────────────────────
Write-Step "Esperando servicios..."
$deadline = (Get-Date).AddSeconds(40)
$espejoOk = $false
$proxyOk  = $false
$dashOk   = $false

while ((Get-Date) -lt $deadline -and (-not ($espejoOk -and $proxyOk -and $dashOk))) {
    Start-Sleep -Seconds 2
    $espejoOk = Test-Health $EspejoHealth 4
    $proxyOk  = Test-Health $ProxyHealth 4
    $dashOk   = Test-Health $DashHealth 5
    $eIcon = if ($espejoOk) { '+' } else { '-' }
    $pIcon = if ($proxyOk)  { '+' } else { '-' }
    $dIcon = if ($dashOk)   { '+' } else { '-' }
    Write-Host "`r  Espejo :$EspejoPort $eIcon   Proxy :$ProxyPort $pIcon   Dashboard :$DashPort $dIcon   " -NoNewline
}
Write-Host ""

if ($espejoOk -and $proxyOk -and $dashOk) {
    Write-Ok "Todos los servicios activos"
    Write-Host ""
    Write-Host "  Abriendo RAULI-VISION..." -ForegroundColor Magenta
    Start-Process "http://localhost:$DashPort"
} elseif ($dashOk) {
    Write-Warn "Dashboard OK - espejo/proxy pueden tardar unos segundos mas"
    Start-Process "http://localhost:$DashPort"
} else {
    Write-Warn "Los servicios aun estan iniciando, abre manualmente:"
    Write-Host "  -> http://localhost:$DashPort" -ForegroundColor Yellow
    Write-Host "  Log: $LogFile" -ForegroundColor DarkGray
}

Write-Host ""
Write-Host "  RAULI-VISION se mantendra activo automaticamente." -ForegroundColor DarkGray
Write-Host "  Para ver el log en tiempo real:" -ForegroundColor DarkGray
Write-Host "  Get-Content $LogFile -Wait -Tail 30" -ForegroundColor DarkGray

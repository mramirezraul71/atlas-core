<#
.SYNOPSIS
    Un solo clic: arranca RAULI-VISION completo (espejo + dashboard + watchdog).
    Ejecuta como Administrador la primera vez para registrar la tarea de inicio.
#>

$WatchdogScript = "$PSScriptRoot\scripts\rauli_vision_watchdog.ps1"
$TaskName       = "RAULI-VISION-Watchdog"
$DashPort       = 5173
$EspejoPort     = 3000
$LogFile        = "$PSScriptRoot\logs\rauli_vision_watchdog.log"

function Write-Step([string]$msg) {
    Write-Host "[$(Get-Date -Format 'HH:mm:ss')] $msg" -ForegroundColor Cyan
}
function Write-Ok([string]$msg) {
    Write-Host "[$(Get-Date -Format 'HH:mm:ss')] ✓ $msg" -ForegroundColor Green
}
function Write-Warn([string]$msg) {
    Write-Host "[$(Get-Date -Format 'HH:mm:ss')] ! $msg" -ForegroundColor Yellow
}

# ── Verificar si el watchdog ya corre ─────────────────────────────────────────
$wdRunning = Get-Process -Name "powershell" -ErrorAction SilentlyContinue |
    Where-Object { $_.CommandLine -like "*rauli_vision_watchdog*" }

if ($wdRunning) {
    Write-Ok "Watchdog ya está corriendo (PID=$($wdRunning.Id))"
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
$dashOk   = $false

while ((Get-Date) -lt $deadline -and (-not ($espejoOk -and $dashOk))) {
    Start-Sleep -Seconds 2
    $espejoOk = (Get-NetTCPConnection -State Listen -LocalPort $EspejoPort -EA SilentlyContinue) -ne $null
    $dashOk   = (Get-NetTCPConnection -State Listen -LocalPort $DashPort   -EA SilentlyContinue) -ne $null
    $eIcon = if ($espejoOk) { "✓" } else { "○" }
    $dIcon = if ($dashOk)   { "✓" } else { "○" }
    Write-Host "`r  Espejo :$EspejoPort $eIcon   Dashboard :$DashPort $dIcon   " -NoNewline
}
Write-Host ""

if ($espejoOk -and $dashOk) {
    Write-Ok "Ambos servicios activos"
    Write-Host ""
    Write-Host "  Abriendo RAULI-VISION..." -ForegroundColor Magenta
    Start-Process "http://localhost:$DashPort"
} elseif ($dashOk) {
    Write-Warn "Dashboard OK — espejo puede tardar unos segundos más"
    Start-Process "http://localhost:$DashPort"
} else {
    Write-Warn "Los servicios aún están iniciando, abre manualmente:"
    Write-Host "  → http://localhost:$DashPort" -ForegroundColor Yellow
    Write-Host "  → Log: $LogFile" -ForegroundColor DarkGray
}

Write-Host ""
Write-Host "  RAULI-VISION se mantendrá activo automáticamente." -ForegroundColor DarkGray
Write-Host "  Para ver el log en tiempo real:" -ForegroundColor DarkGray
Write-Host "  Get-Content $LogFile -Wait -Tail 30" -ForegroundColor DarkGray

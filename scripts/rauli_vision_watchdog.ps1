<#
.SYNOPSIS
    RAULI-VISION Watchdog — mantiene espejo + dashboard 24/7 sin desconexiones.

.DESCRIPTION
    Monitorea dos servicios cada $CheckIntervalSec segundos:

      1. Espejo (Go backend)  → http://localhost:3000/api/health
      2. Dashboard (React)    → http://localhost:5173

    Si alguno no responde: lo reinicia automáticamente.
    Si el binario se colgó: mata el proceso y relanza.

    Primer arranque: construye el dashboard si dist/ no existe o tiene > 12h.

.PARAMETER CheckIntervalSec
    Segundos entre cada ciclo de health-check (default 30).

.PARAMETER NoAutoRegister
    No registrar la tarea en Task Scheduler (solo correr una vez).

.EXAMPLE
    # Arrancar watchdog en segundo plano (modo normal)
    .\scripts\rauli_vision_watchdog.ps1

    # Solo health-check sin registrar en scheduler
    .\scripts\rauli_vision_watchdog.ps1 -NoAutoRegister
#>
param(
    [int]   $CheckIntervalSec = 30,
    [switch]$NoAutoRegister
)

$ErrorActionPreference = "SilentlyContinue"

# ── Rutas ──────────────────────────────────────────────────────────────────────
$RvRoot      = "C:\ATLAS_PUSH\_external\RAULI-VISION"
$EspejoDir   = "$RvRoot\espejo"
$EspejoExe   = "$EspejoDir\espejo.exe"
$DashDir     = "$RvRoot\dashboard"
$LogFile     = "C:\ATLAS_PUSH\logs\rauli_vision_watchdog.log"
$PidFile     = "C:\ATLAS_PUSH\logs\rauli_vision.pids"

# ── Puertos ────────────────────────────────────────────────────────────────────
$EspejoPort  = 3000
$DashPort    = 5173

# ── Credenciales espejo (no-secretas en dev local) ─────────────────────────────
$EspejoEnv = @{
    PORT         = "$EspejoPort"
    JWT_SECRET   = if ($env:RAULI_JWT_SECRET)  { $env:RAULI_JWT_SECRET }  else { "rauli-vision-local-secret" }
    ADMIN_TOKEN  = if ($env:RAULI_ADMIN_TOKEN) { $env:RAULI_ADMIN_TOKEN } else { "rauli-admin-local" }
    ACCESS_STORE = "data\access-store.json"
    GEMINI_API_KEY = if ($env:GEMINI_API_KEY)  { $env:GEMINI_API_KEY }   else { "" }
}

# ── Logger ─────────────────────────────────────────────────────────────────────
$null = New-Item -ItemType Directory -Path (Split-Path $LogFile) -Force
function Log([string]$msg, [string]$color = "Cyan") {
    $line = "[$(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')] $msg"
    Add-Content -Path $LogFile -Value $line -ErrorAction SilentlyContinue
    Write-Host $line -ForegroundColor $color
}

# ── Health check via HTTP ──────────────────────────────────────────────────────
function Test-Http([string]$url, [int]$timeoutSec = 4) {
    try {
        $r = Invoke-WebRequest -Uri $url -TimeoutSec $timeoutSec `
                               -UseBasicParsing -ErrorAction Stop
        return $r.StatusCode -lt 500
    } catch { return $false }
}

# ── Verifica si un puerto está escuchando (más rápido que HTTP) ────────────────
function Test-Port([int]$port) {
    $conn = Get-NetTCPConnection -State Listen -LocalPort $port -ErrorAction SilentlyContinue
    return ($null -ne $conn)
}

# ── Mata proceso que ocupa un puerto ──────────────────────────────────────────
function Kill-Port([int]$port) {
    $conn = Get-NetTCPConnection -State Listen -LocalPort $port -ErrorAction SilentlyContinue
    if ($conn) {
        $pid_ = $conn | Select-Object -First 1 -ExpandProperty OwningProcess
        if ($pid_ -and $pid_ -ne $PID) {
            Log "  → Matando proceso PID=$pid_ que ocupaba :$port" "Yellow"
            Stop-Process -Id $pid_ -Force -ErrorAction SilentlyContinue
            Start-Sleep -Milliseconds 800
        }
    }
}

# ── Construir dashboard (una vez, o si dist/ tiene >12h) ──────────────────────
function Build-Dashboard {
    $distIndex = "$DashDir\dist\index.html"
    $needBuild = $true
    if (Test-Path $distIndex) {
        $age = (Get-Date) - (Get-Item $distIndex).LastWriteTime
        if ($age.TotalHours -lt 12) { $needBuild = $false }
    }
    if (-not $needBuild) {
        Log "Dashboard dist/ OK (< 12h)" "Green"
        return $true
    }
    Log "Construyendo dashboard (npm run build)..." "Cyan"
    $nodeVer = (node -v 2>$null)
    if (-not $nodeVer) {
        Log "ERROR: Node.js no disponible en PATH" "Red"
        return $false
    }
    Push-Location $DashDir
    try {
        $result = npm run build 2>&1
        if ($LASTEXITCODE -ne 0) {
            Log "ERROR en build: $($result | Select-Object -Last 5 | Out-String)" "Red"
            return $false
        }
        Log "Dashboard build OK" "Green"
        return $true
    } finally {
        Pop-Location
    }
}

# ── Arrancar Espejo ────────────────────────────────────────────────────────────
function Start-Espejo {
    if (-not (Test-Path $EspejoExe)) {
        Log "ERROR: espejo.exe no encontrado en $EspejoDir" "Red"
        return $false
    }
    Kill-Port $EspejoPort

    # Preparar variables de entorno para el proceso hijo
    $envBlock = $EspejoEnv.GetEnumerator() | ForEach-Object { "$($_.Key)=$($_.Value)" }
    $psi = New-Object System.Diagnostics.ProcessStartInfo
    $psi.FileName               = $EspejoExe
    $psi.WorkingDirectory       = $EspejoDir
    $psi.UseShellExecute        = $false
    $psi.RedirectStandardOutput = $true
    $psi.RedirectStandardError  = $true
    $psi.CreateNoWindow         = $true
    foreach ($kv in $EspejoEnv.GetEnumerator()) {
        $psi.EnvironmentVariables[$kv.Key] = $kv.Value
    }

    $proc = [System.Diagnostics.Process]::Start($psi)
    if (-not $proc) {
        Log "ERROR: no se pudo iniciar espejo.exe" "Red"
        return $false
    }

    # Redirigir stdout/stderr al log en background
    $logPath = $LogFile
    $proc.BeginOutputReadLine()
    $proc.BeginErrorReadLine()
    Register-ObjectEvent -InputObject $proc -EventName OutputDataReceived -Action {
        if ($Event.SourceEventArgs.Data) {
            Add-Content -Path $using:logPath -Value "[ESPEJO] $($Event.SourceEventArgs.Data)" -EA SilentlyContinue
        }
    } | Out-Null
    Register-ObjectEvent -InputObject $proc -EventName ErrorDataReceived -Action {
        if ($Event.SourceEventArgs.Data) {
            Add-Content -Path $using:logPath -Value "[ESPEJO-ERR] $($Event.SourceEventArgs.Data)" -EA SilentlyContinue
        }
    } | Out-Null

    # Esperar hasta 15s que el puerto abra
    $deadline = (Get-Date).AddSeconds(15)
    while ((Get-Date) -lt $deadline) {
        Start-Sleep -Milliseconds 500
        if (Test-Port $EspejoPort) {
            Log "Espejo OK :$EspejoPort (PID=$($proc.Id))" "Green"
            return $true
        }
    }
    Log "WARN: espejo no respondió en 15s — puede seguir iniciando" "Yellow"
    return $true
}

# ── Arrancar Dashboard (Vite preview — sirve dist/ estático) ──────────────────
function Start-Dashboard {
    Kill-Port $DashPort

    $nodeExe = (Get-Command node -ErrorAction SilentlyContinue)?.Source
    if (-not $nodeExe) {
        Log "ERROR: Node.js no disponible" "Red"
        return $false
    }

    # Servir dist/ con Vite preview — no requiere hot-reload, es más estable
    $proc = Start-Process -FilePath "cmd.exe" `
        -ArgumentList "/c", "npm run preview -- --port $DashPort --host 0.0.0.0" `
        -WorkingDirectory $DashDir `
        -RedirectStandardOutput "$($LogFile).dash.log" `
        -RedirectStandardError  "$($LogFile).dash.err.log" `
        -WindowStyle Hidden `
        -PassThru

    if (-not $proc) {
        Log "ERROR: no se pudo iniciar Vite preview" "Red"
        return $false
    }

    $deadline = (Get-Date).AddSeconds(20)
    while ((Get-Date) -lt $deadline) {
        Start-Sleep -Milliseconds 600
        if (Test-Port $DashPort) {
            Log "Dashboard OK :$DashPort (PID=$($proc.Id))" "Green"
            return $true
        }
    }
    Log "WARN: dashboard no respondió en 20s" "Yellow"
    return $true
}

# ── Registrar en Task Scheduler (auto-start al encender PC) ──────────────────
function Register-StartupTask {
    $taskName = "RAULI-VISION-Watchdog"
    $existing = schtasks /query /tn $taskName 2>&1
    if ($LASTEXITCODE -eq 0) {
        Log "Task Scheduler '$taskName' ya registrada — skip" "Green"
        return
    }
    $scriptPath = $PSCommandPath
    $cmd = "powershell.exe -NonInteractive -WindowStyle Hidden -File `"$scriptPath`" -NoAutoRegister"
    schtasks /create `
        /tn   $taskName `
        /tr   $cmd `
        /sc   ONLOGON `
        /rl   HIGHEST `
        /f | Out-Null
    if ($LASTEXITCODE -eq 0) {
        Log "Task Scheduler registrada: '$taskName' (ONLOGON, admin)" "Green"
    } else {
        Log "WARN: no se pudo registrar tarea (intenta ejecutar como Admin)" "Yellow"
    }
}

# ══════════════════════════════════════════════════════════════════════════════
# INICIO
# ══════════════════════════════════════════════════════════════════════════════
Log "═══ RAULI-VISION Watchdog iniciado (intervalo=${CheckIntervalSec}s) ═══" "Magenta"

# Registrar en Task Scheduler la primera vez
if (-not $NoAutoRegister) { Register-StartupTask }

# Build inicial del dashboard si es necesario
$buildOk = Build-Dashboard
if (-not $buildOk) {
    Log "Fallando a npm run dev como fallback..." "Yellow"
}

# Primer arranque de ambos servicios
if (-not (Test-Port $EspejoPort)) { Start-Espejo | Out-Null }
else { Log "Espejo ya escucha en :$EspejoPort — skip" "Green" }

if (-not (Test-Port $DashPort)) {
    if ($buildOk) { Start-Dashboard | Out-Null }
    else {
        # Fallback: dev server si preview no funciona
        Start-Process -FilePath "cmd.exe" `
            -ArgumentList "/c npm run dev -- --port $DashPort --host 0.0.0.0" `
            -WorkingDirectory $DashDir `
            -WindowStyle Hidden | Out-Null
        Log "Dashboard (dev fallback) iniciado en :$DashPort" "Yellow"
    }
} else {
    Log "Dashboard ya escucha en :$DashPort — skip" "Green"
}

Start-Sleep -Seconds 3
Log "Servicios iniciales levantados. URL: http://localhost:$DashPort" "Green"

# ══════════════════════════════════════════════════════════════════════════════
# LOOP DE MONITOREO CONTINUO
# ══════════════════════════════════════════════════════════════════════════════
$espejo_failures  = 0
$dash_failures    = 0
$MAX_FAILURES     = 2    # reintentos antes de reiniciar

while ($true) {
    Start-Sleep -Seconds $CheckIntervalSec

    # ── Health Espejo ──────────────────────────────────────────────────────────
    $espejoOk = Test-Port $EspejoPort
    if ($espejoOk) {
        $espejo_failures = 0
    } else {
        $espejo_failures++
        Log "Espejo :$EspejoPort NO responde (fallo $espejo_failures/$MAX_FAILURES)" "Yellow"
        if ($espejo_failures -ge $MAX_FAILURES) {
            Log "REINICIANDO Espejo..." "Red"
            Start-Espejo | Out-Null
            $espejo_failures = 0
        }
    }

    # ── Health Dashboard ───────────────────────────────────────────────────────
    $dashOk = Test-Port $DashPort
    if ($dashOk) {
        $dash_failures = 0
    } else {
        $dash_failures++
        Log "Dashboard :$DashPort NO responde (fallo $dash_failures/$MAX_FAILURES)" "Yellow"
        if ($dash_failures -ge $MAX_FAILURES) {
            Log "REINICIANDO Dashboard..." "Red"
            if ($buildOk) { Start-Dashboard | Out-Null }
            else {
                Kill-Port $DashPort
                Start-Process -FilePath "cmd.exe" `
                    -ArgumentList "/c npm run dev -- --port $DashPort --host 0.0.0.0" `
                    -WorkingDirectory $DashDir `
                    -WindowStyle Hidden | Out-Null
                Log "Dashboard (dev) relanzado en :$DashPort" "Yellow"
            }
            $dash_failures = 0
        }
    }

    # ── Log periódico (cada 5 minutos) ─────────────────────────────────────────
    $now = Get-Date
    if ($now.Second -lt $CheckIntervalSec -and ($now.Minute % 5 -eq 0)) {
        $eStatus = if ($espejoOk)  { "OK" } else { "CAIDO" }
        $dStatus = if ($dashOk)    { "OK" } else { "CAIDO" }
        Log "HEARTBEAT espejo=$eStatus dashboard=$dStatus | url=http://localhost:$DashPort" "DarkGray"
    }
}

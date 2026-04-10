<#
.SYNOPSIS
    RAULI-VISION Watchdog — mantiene espejo + proxy + dashboard 24/7 sin desconexiones.

.DESCRIPTION
    Monitorea tres servicios cada $CheckIntervalSec segundos:

      1. Espejo (Go backend)    → puerto 8080  (espejo.exe)
      2. Proxy API (Python)     → puerto 3000  (simple-server.py, proxea a :8080)
      3. Dashboard (React/Vite) → puerto 5174  (npm run preview / dev)

    Si alguno no responde: lo reinicia automáticamente.

    Arquitectura real:
      espejo.exe (:8080) ← simple-server.py (:3000) ← browser dashboard (:5174)

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
$ProxyDir    = "$RvRoot\cliente-local"
$ProxyScript = "$ProxyDir\simple-server.py"
$DashDir     = "$RvRoot\dashboard"
$LogFile     = "C:\ATLAS_PUSH\logs\rauli_vision_watchdog.log"

# ── Puertos ────────────────────────────────────────────────────────────────────
$EspejoPort  = 8080   # espejo.exe — Go backend (run_espejo.bat usa PORT=8080)
$ProxyPort   = 3000   # simple-server.py — proxy Python que apunta a :8080
$DashPort    = 5174   # dashboard React/Vite

# ── Credenciales espejo ────────────────────────────────────────────────────────
$EspejoEnv = @{
    PORT           = "$EspejoPort"
    JWT_SECRET     = if ($env:RAULI_JWT_SECRET)  { $env:RAULI_JWT_SECRET }  else { "rauli-vision-local-secret" }
    ADMIN_TOKEN    = if ($env:RAULI_ADMIN_TOKEN) { $env:RAULI_ADMIN_TOKEN } else { "rauli-admin-local" }
    ACCESS_STORE   = "data\access-store.json"
    GEMINI_API_KEY = if ($env:GEMINI_API_KEY)    { $env:GEMINI_API_KEY }    else { "" }
}

# ── Logger ─────────────────────────────────────────────────────────────────────
$null = New-Item -ItemType Directory -Path (Split-Path $LogFile) -Force
function Log([string]$msg, [string]$color = "Cyan") {
    $line = "[$(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')] $msg"
    Add-Content -Path $LogFile -Value $line -ErrorAction SilentlyContinue
    Write-Host $line -ForegroundColor $color
}

# ── Verifica si un puerto está escuchando ─────────────────────────────────────
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
            Log "  >> Matando proceso PID=$pid_ que ocupaba :$port" "Yellow"
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

# ── Arrancar Espejo (Go backend) ───────────────────────────────────────────────
function Start-Espejo {
    if (-not (Test-Path $EspejoExe)) {
        Log "ERROR: espejo.exe no encontrado en $EspejoDir" "Red"
        return $false
    }
    Kill-Port $EspejoPort

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

    $deadline = (Get-Date).AddSeconds(15)
    while ((Get-Date) -lt $deadline) {
        Start-Sleep -Milliseconds 500
        if (Test-Port $EspejoPort) {
            Log "Espejo OK :$EspejoPort (PID=$($proc.Id))" "Green"
            return $true
        }
    }
    Log "WARN: espejo no respondio en 15s - puede seguir iniciando" "Yellow"
    return $true
}

# ── Arrancar Proxy API (Python simple-server.py → proxea a espejo :8080) ───────
function Start-Proxy {
    if (-not (Test-Path $ProxyScript)) {
        Log "ERROR: simple-server.py no encontrado en $ProxyDir" "Red"
        return $false
    }
    # Usar el venv de ATLAS que tiene el módulo 'requests' requerido por simple-server.py
    $venvPython = "C:\ATLAS_PUSH\venv\Scripts\python.exe"
    $pythonExe = if (Test-Path $venvPython) { $venvPython } else { "python" }

    Kill-Port $ProxyPort

    $proc = Start-Process -FilePath $pythonExe `
        -ArgumentList $ProxyScript `
        -WorkingDirectory $ProxyDir `
        -RedirectStandardOutput "$($LogFile).proxy.log" `
        -RedirectStandardError  "$($LogFile).proxy.err.log" `
        -WindowStyle Hidden `
        -PassThru

    if (-not $proc) {
        Log "ERROR: no se pudo iniciar simple-server.py" "Red"
        return $false
    }

    $deadline = (Get-Date).AddSeconds(12)
    while ((Get-Date) -lt $deadline) {
        Start-Sleep -Milliseconds 500
        if (Test-Port $ProxyPort) {
            Log "Proxy API OK :$ProxyPort (PID=$($proc.Id))" "Green"
            return $true
        }
    }
    Log "WARN: proxy API no respondio en 12s" "Yellow"
    return $true
}

# ── Arrancar Dashboard (Vite preview — sirve dist/ estático) ──────────────────
function Start-Dashboard {
    Kill-Port $DashPort

    $nodeCmd = Get-Command node -ErrorAction SilentlyContinue
    if (-not $nodeCmd) {
        Log "ERROR: Node.js no disponible" "Red"
        return $false
    }

    $proc = Start-Process -FilePath "cmd.exe" `
        -ArgumentList "/c", "npm run preview -- --port $DashPort --host 127.0.0.1" `
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
    Log "WARN: dashboard no respondio en 20s" "Yellow"
    return $true
}

# ── Registrar en Task Scheduler (auto-start al encender PC) ──────────────────
function Register-StartupTask {
    $taskName = "RAULI-VISION-Watchdog"
    $existing = schtasks /query /tn $taskName 2>&1
    if ($LASTEXITCODE -eq 0) {
        Log "Task Scheduler tarea $taskName ya registrada - skip" "Green"
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
        Log "Task Scheduler registrada: $taskName [ONLOGON elevado]" "Green"
    } else {
        Log "WARN: no se pudo registrar tarea (intenta ejecutar como Admin)" "Yellow"
    }
}

# ══════════════════════════════════════════════════════════════════════════════
# INICIO
# ══════════════════════════════════════════════════════════════════════════════
Log "=== RAULI-VISION Watchdog iniciado (intervalo=${CheckIntervalSec}s) ===" "Magenta"
Log "    Puertos: espejo=:$EspejoPort  proxy=:$ProxyPort  dashboard=:$DashPort" "DarkGray"

if (-not $NoAutoRegister) { Register-StartupTask }

# Build inicial del dashboard si es necesario
$buildOk = Build-Dashboard
if (-not $buildOk) {
    Log "Build fallo - se usara npm run dev como fallback" "Yellow"
}

# ── Primer arranque de los tres servicios ─────────────────────────────────────
if (-not (Test-Port $EspejoPort)) {
    Log "Iniciando espejo.exe en :$EspejoPort..." "Cyan"
    Start-Espejo | Out-Null
} else {
    Log "Espejo ya escucha en :$EspejoPort - skip" "Green"
}

if (-not (Test-Port $ProxyPort)) {
    Log "Iniciando proxy API en :$ProxyPort..." "Cyan"
    Start-Proxy | Out-Null
} else {
    Log "Proxy API ya escucha en :$ProxyPort - skip" "Green"
}

if (-not (Test-Port $DashPort)) {
    if ($buildOk) {
        Log "Iniciando dashboard (preview) en :$DashPort..." "Cyan"
        Start-Dashboard | Out-Null
    } else {
        Start-Process -FilePath "cmd.exe" `
            -ArgumentList "/c npm run dev -- --port $DashPort --host 0.0.0.0" `
            -WorkingDirectory $DashDir `
            -WindowStyle Hidden | Out-Null
        Log "Dashboard (dev fallback) iniciado en :$DashPort" "Yellow"
    }
} else {
    Log "Dashboard ya escucha en :$DashPort - skip" "Green"
}

Start-Sleep -Seconds 3
Log "Servicios iniciales levantados. URL dashboard: http://localhost:$DashPort" "Green"

# ══════════════════════════════════════════════════════════════════════════════
# LOOP DE MONITOREO CONTINUO
# ══════════════════════════════════════════════════════════════════════════════
$espejo_failures = 0
$proxy_failures  = 0
$dash_failures   = 0
$MAX_FAILURES    = 2

while ($true) {
    Start-Sleep -Seconds $CheckIntervalSec

    # ── Health Espejo (:8080) ───────────────────────────────────────────────────
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

    # ── Health Proxy API (:3000) ────────────────────────────────────────────────
    $proxyOk = Test-Port $ProxyPort
    if ($proxyOk) {
        $proxy_failures = 0
    } else {
        $proxy_failures++
        Log "Proxy API :$ProxyPort NO responde (fallo $proxy_failures/$MAX_FAILURES)" "Yellow"
        if ($proxy_failures -ge $MAX_FAILURES) {
            Log "REINICIANDO Proxy API..." "Red"
            Start-Proxy | Out-Null
            $proxy_failures = 0
        }
    }

    # ── Health Dashboard (:5174) ────────────────────────────────────────────────
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
        $eStatus = if ($espejoOk) { "OK" } else { "CAIDO" }
        $pStatus = if ($proxyOk)  { "OK" } else { "CAIDO" }
        $dStatus = if ($dashOk)   { "OK" } else { "CAIDO" }
        Log "HEARTBEAT espejo=$eStatus proxy=$pStatus dashboard=$dStatus" "DarkGray"
    }
}

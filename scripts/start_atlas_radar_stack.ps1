<#
.SYNOPSIS
  Arranque controlado: Atlas Code Quant (:8792) + PUSH (:8791) con radar enlazado a Quant.

.DESCRIPTION
  Valida rutas y ATLAS_QUANT_API_KEY, detiene listeners en 8791/8792 y procesos Python conocidos,
  levanta Quant y luego PUSH, escribe logs y ejecuta smoke tests HTTP.

.PARAMETER RepoRoot
  Raíz del repo (por defecto: padre de scripts/).

.PARAMETER DisableCamera
  Si está presente, ENABLE_CAMERA=false en Quant.

.PARAMETER FullStartup
  Si está presente, fuerza QUANT_LIGHTWEIGHT_STARTUP=false (arranque completo para métricas canónicas).

.PARAMETER UseInsta360
  Fuerza QUANT_DEFAULT_VISION_PROVIDER=insta360. Si no se especifica, se usa desktop_capture por estabilidad.

.PARAMETER SkipKill
  No intenta matar procesos previos (solo útil si ya liberaste puertos).

.PARAMETER QuantWaitSec
  Segundos de espera tras arrancar Quant antes del health check (default 5).

.PARAMETER PushWaitSec
  Segundos de espera tras arrancar PUSH (default 3).

.EXAMPLE
  $env:ATLAS_QUANT_API_KEY = "tu-clave"
  .\scripts\start_atlas_radar_stack.ps1
#>
param(
    [string]$RepoRoot = "",
    [switch]$DisableCamera,
    [switch]$FullStartup,
    [switch]$UseInsta360,
    [switch]$SkipKill,
    [int]$QuantWaitSec = 12,
    [int]$PushWaitSec = 10
)

# Sin StrictMode Latest: en algunos hosts PS 5.1 ciertos nombres de variable de log chocan con el analizador.
Set-StrictMode -Off
$ErrorActionPreference = "Stop"

if (-not $RepoRoot) {
    $RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
}
$QuantRoot = Join-Path $RepoRoot "atlas_code_quant"
$AdapterRoot = Join-Path $RepoRoot "atlas_adapter"
$LogDir = Join-Path $RepoRoot "logs"
$RuntimeHelpers = Join-Path $PSScriptRoot "atlas_runtime.ps1"
$FreePortScript = Join-Path $PSScriptRoot "free_port.ps1"

function Write-Step([string]$msg) { Write-Host "[ATLAS-RADAR] $msg" -ForegroundColor Cyan }
function Write-Ok([string]$msg) { Write-Host "[OK] $msg" -ForegroundColor Green }
function Write-Fail([string]$msg) { Write-Host "[FAIL] $msg" -ForegroundColor Red }

function Stop-PythonByPatterns {
    param([string[]]$Patterns, [int]$TimeoutSec = 12)
    $deadline = [datetime]::UtcNow.AddSeconds($TimeoutSec)
    foreach ($pat in $Patterns) {
        try {
            $matches = Get-CimInstance Win32_Process -Filter "Name='python.exe'" -ErrorAction SilentlyContinue |
                Where-Object { $_.CommandLine -and $_.CommandLine -match $pat }
            foreach ($proc in $matches) {
                try {
                    Stop-Process -Id $proc.ProcessId -Force -ErrorAction SilentlyContinue
                } catch { }
            }
        } catch { }
    }
    while ([datetime]::UtcNow -lt $deadline) {
        $still = $false
        foreach ($pat in $Patterns) {
            $n = @(Get-CimInstance Win32_Process -Filter "Name='python.exe'" -ErrorAction SilentlyContinue |
                Where-Object { $_.CommandLine -and $_.CommandLine -match $pat }).Count
            if ($n -gt 0) { $still = $true; break }
        }
        if (-not $still) { break }
        Start-Sleep -Milliseconds 400
    }
}

function Test-HttpOk {
    param([string]$Url, [hashtable]$Headers = @{}, [int]$TimeoutSec = 8)
    try {
        $resp = Invoke-WebRequest -Uri $Url -Headers $Headers -TimeoutSec $TimeoutSec -UseBasicParsing -ErrorAction Stop
        return @{ Ok = ($resp.StatusCode -eq 200); Status = $resp.StatusCode; Body = $resp.Content }
    } catch {
        return @{ Ok = $false; Status = 0; Body = ""; Error = $_.Exception.Message }
    }
}

function Stop-ListenerOnPort {
    param([int]$Port)
    try {
        $pids = @(Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction SilentlyContinue |
            Select-Object -ExpandProperty OwningProcess -Unique |
            Where-Object { $_ -and $_ -gt 0 })
        foreach ($procId in $pids) {
            Stop-Process -Id $procId -Force -ErrorAction SilentlyContinue
        }
    } catch { }
}

function Wait-PortListenerGone {
    param([int]$Port, [int]$MaxSec = 20)
    $until = [datetime]::UtcNow.AddSeconds($MaxSec)
    while ([datetime]::UtcNow -lt $until) {
        $ln = @(Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction SilentlyContinue).Count
        if ($ln -eq 0) { return $true }
        Stop-ListenerOnPort -Port $Port
        Start-Sleep -Milliseconds 500
    }
    return (@(Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction SilentlyContinue).Count -eq 0)
}

function Get-SseHeadBytes {
    param([string]$Url, [int]$MaxSec = 4)
    if (-not (Get-Command curl.exe -ErrorAction SilentlyContinue)) {
        return ""
    }
    $tmp = Join-Path $env:TEMP ("atlas_sse_" + [guid]::NewGuid().ToString("n") + ".txt")
    try {
        $p = Start-Process -FilePath "curl.exe" `
            -ArgumentList @("-sS", "--max-time", $MaxSec.ToString(), $Url) `
            -RedirectStandardOutput $tmp `
            -RedirectStandardError (Join-Path $env:TEMP "atlas_sse_err.txt") `
            -NoNewWindow -PassThru -Wait
        if (Test-Path $tmp) {
            $raw = [System.IO.File]::ReadAllText($tmp)
            if ($raw.Length -gt 8000) { return $raw.Substring(0, 8000) }
            return $raw
        }
    } finally {
        Remove-Item -Path $tmp -Force -ErrorAction SilentlyContinue
    }
    return ""
}

# --- 1. Validar entorno ---
Write-Step "Validando entorno..."
if (-not (Test-Path $QuantRoot)) { Write-Fail "No existe atlas_code_quant: $QuantRoot"; exit 2 }
if (-not (Test-Path $AdapterRoot)) { Write-Fail "No existe atlas_adapter: $AdapterRoot"; exit 2 }
if (-not $env:ATLAS_QUANT_API_KEY -or [string]::IsNullOrWhiteSpace($env:ATLAS_QUANT_API_KEY)) {
    Write-Warning "ATLAS_QUANT_API_KEY no definida; usando clave local por defecto 'atlas-quant-local' para stack local."
    $env:ATLAS_QUANT_API_KEY = "atlas-quant-local"
}
New-Item -ItemType Directory -Path $LogDir -Force | Out-Null
Write-Ok "Rutas y API key OK."

# --- 2. Parar servicios existentes ---
if (-not $SkipKill) {
    Write-Step "Liberando puertos 8792 y 8791 y deteniendo procesos Python relacionados..."
    if (Test-Path $FreePortScript) {
        try { & $FreePortScript -Port 8792 -Kill | Out-Null } catch { }
        try { & $FreePortScript -Port 8791 -Kill | Out-Null } catch { }
    }
    Stop-ListenerOnPort -Port 8792
    Stop-ListenerOnPort -Port 8791
    Start-Sleep -Seconds 2
    if (-not (Wait-PortListenerGone -Port 8792 -MaxSec 25)) {
        Write-Warning "Puerto 8792 sigue en LISTEN tras limpieza; se reintenta Stop-Listener."
        Stop-ListenerOnPort -Port 8792
    }
    if (-not (Wait-PortListenerGone -Port 8791 -MaxSec 25)) {
        Write-Warning "Puerto 8791 sigue en LISTEN tras limpieza; se reintenta Stop-Listener."
        Stop-ListenerOnPort -Port 8791
    }
    # free_port.ps1 puede dejar LASTEXITCODE=1 sin afectar el flujo de este script
    $global:LASTEXITCODE = 0
    if (Test-Path $RuntimeHelpers) {
        . $RuntimeHelpers
        Stop-AtlasPythonProcesses -Pattern 'atlas_adapter\.atlas_http_api:app'
        Stop-AtlasPythonProcesses -Pattern 'atlas_adapter\.atlas_http_api'
    }
    Stop-PythonByPatterns -Patterns @(
        'atlas_code_quant[\\/]main\.py',
        'atlas_code_quant\\main\.py',
        'uvicorn.*8792',
        'uvicorn.*:8792'
    ) -TimeoutSec 14
    Start-Sleep -Milliseconds 500
    Write-Ok "Stop best-effort completado."
}

if (-not (Test-Path $RuntimeHelpers)) {
    Write-Fail "Falta scripts/atlas_runtime.ps1 (Resolve-AtlasPython)."; exit 2
}
. $RuntimeHelpers
# atlas_runtime.ps1 fuerza StrictMode Latest; relajar para el resto de este script (variables de log, etc.)
Set-StrictMode -Off
$venvPython = Join-Path $RepoRoot "venv\Scripts\python.exe"
if (Test-Path $venvPython) {
    $Python = $venvPython
} else {
    $Python = Resolve-AtlasPython -RepoRoot $RepoRoot
}

# --- 3. Quant primero ---
$QuantLog = Join-Path $LogDir "quant.log"
$QuantErr = Join-Path $LogDir "quant.stderr.log"
Remove-Item $QuantLog, $QuantErr -Force -ErrorAction SilentlyContinue

$env:PYTHONPATH = $RepoRoot
if ($DisableCamera) {
    $env:ENABLE_CAMERA = "false"
    $env:QUANT_STARTUP_VISUAL_CONNECT_ENABLED = "false"
    $env:QUANT_DEFAULT_VISION_PROVIDER = "off"
} else {
    $env:ENABLE_CAMERA = "true"
    if ($UseInsta360) {
        $env:QUANT_DEFAULT_VISION_PROVIDER = "insta360"
        $env:QUANT_STARTUP_VISUAL_CONNECT_ENABLED = "true"
    } else {
        # Default operativo y estable para hosts sin captura USB/RTMP confiable.
        if (-not $env:QUANT_DEFAULT_VISION_PROVIDER -or [string]::IsNullOrWhiteSpace($env:QUANT_DEFAULT_VISION_PROVIDER)) {
            $env:QUANT_DEFAULT_VISION_PROVIDER = "desktop_capture"
        }
        $env:QUANT_STARTUP_VISUAL_CONNECT_ENABLED = "false"
    }
}
# La clave ya validada; Quant y PUSH deben compartirla
$env:ATLAS_QUANT_API_KEY = $env:ATLAS_QUANT_API_KEY.Trim()
# Quant valida X-Api-Key contra settings.api_key (QUANT_API_KEY), no solo ATLAS_QUANT_API_KEY
$env:QUANT_API_KEY = $env:ATLAS_QUANT_API_KEY
# Puerto HTTP del motor: 8792 para alinear con QUANT_BASE_URL del radar/PUSH
$env:QUANT_API_PORT = "8792"
$env:ATLAS_MINIMAL_STARTUP = if ($env:ATLAS_MINIMAL_STARTUP) { $env:ATLAS_MINIMAL_STARTUP } else { "true" }
# Quant lee QUANT_* (no ATLAS_MINIMAL_*).
# Por defecto mantenemos lightweight para estabilidad; con -FullStartup se activa boot completo.
$env:QUANT_LIGHTWEIGHT_STARTUP = if ($FullStartup) { "false" } else { "true" }
$visionProviderForLog = if ($env:QUANT_DEFAULT_VISION_PROVIDER) { $env:QUANT_DEFAULT_VISION_PROVIDER } else { "<auto>" }
Write-Step ("Configuración Quant: ENABLE_CAMERA={0}, QUANT_STARTUP_VISUAL_CONNECT_ENABLED={1}, QUANT_LIGHTWEIGHT_STARTUP={2}, QUANT_DEFAULT_VISION_PROVIDER={3}" -f `
    $env:ENABLE_CAMERA, `
    $env:QUANT_STARTUP_VISUAL_CONNECT_ENABLED, `
    $env:QUANT_LIGHTWEIGHT_STARTUP, `
    $visionProviderForLog)

Write-Step "Arrancando Quant (8792)..."
if (-not (Wait-PortListenerGone -Port 8792 -MaxSec 8)) {
    Write-Fail "Puerto 8792 sigue ocupado; no se puede arrancar Quant."
    exit 2
}
$quantMain = Join-Path $QuantRoot "main.py"
if (-not (Test-Path $quantMain)) { Write-Fail "No se encontró $quantMain"; exit 2 }

$quantProc = Start-Process -FilePath $Python `
    -ArgumentList @("main.py") `
    -WorkingDirectory $QuantRoot `
    -WindowStyle Hidden `
    -RedirectStandardOutput $QuantLog `
    -RedirectStandardError $QuantErr `
    -PassThru

Start-Sleep -Seconds ([Math]::Max(3, $QuantWaitSec - 7))
$hq = @{ Ok = $false }
$deadline = [datetime]::UtcNow.AddSeconds(35)
while ([datetime]::UtcNow -lt $deadline) {
    $hq = Test-HttpOk -Url "http://127.0.0.1:8792/health" -TimeoutSec 10
    if ($hq.Ok) { break }
    Start-Sleep -Seconds 1
}
if (-not $hq.Ok) {
    Write-Fail "Quant no responde en /health tras espera extendida. Ver $QuantLog y $QuantErr"
    exit 3
}
Write-Ok "Quant UP (PID $($quantProc.Id)) - /health HTTP $($hq.Status)."

# --- 4. PUSH después ---
$AtlasRadarPushOutLog = Join-Path $LogDir "push.log"
$AtlasRadarPushErrLog = Join-Path $LogDir "push.stderr.log"
Remove-Item -Path $AtlasRadarPushOutLog, $AtlasRadarPushErrLog -Force -ErrorAction SilentlyContinue

$env:PYTHONPATH = $RepoRoot
$env:QUANT_BASE_URL = "http://127.0.0.1:8792"
$env:ATLAS_RADAR_QUANT_HTTP = "1"
# Evita que el daemon de self-healing interprete falsos positivos y lance otro uvicorn sobre :8791.
$env:ATLAS_SELF_HEALING_ENABLED = "false"
$env:ATLAS_MINIMAL_STARTUP = if ($env:ATLAS_MINIMAL_STARTUP) { $env:ATLAS_MINIMAL_STARTUP } else { "true" }
$env:ATLAS_SAFE_STARTUP = if ($env:ATLAS_SAFE_STARTUP) { $env:ATLAS_SAFE_STARTUP } else { "true" }

Write-Step "Arrancando PUSH (8791) con modulo atlas_adapter.atlas_http_api..."
$pushProc = Start-Process -FilePath $Python `
    -ArgumentList @("-m", "atlas_adapter.atlas_http_api") `
    -WorkingDirectory $RepoRoot `
    -WindowStyle Hidden `
    -RedirectStandardOutput $AtlasRadarPushOutLog `
    -RedirectStandardError $AtlasRadarPushErrLog `
    -PassThru

Start-Sleep -Seconds ([Math]::Max(2, $PushWaitSec - 5))
$hp = @{ Ok = $false }
$pdead = [datetime]::UtcNow.AddSeconds(50)
while ([datetime]::UtcNow -lt $pdead) {
    $hp = Test-HttpOk -Url "http://127.0.0.1:8791/health" -TimeoutSec 10
    if ($hp.Ok) { break }
    Start-Sleep -Seconds 1
}
if (-not $hp.Ok) {
    Write-Fail "PUSH no responde en /health tras espera extendida. Ver $AtlasRadarPushOutLog y $AtlasRadarPushErrLog"
    exit 4
}
Write-Ok "PUSH UP (PID $($pushProc.Id)) - /health HTTP $($hp.Status)."

# --- 5. Smoke tests ---
Write-Step "Smoke tests..."
$apiKey = $env:ATLAS_QUANT_API_KEY
$hdr = @{ "X-Api-Key" = $apiKey }

$hqSmoke = Test-HttpOk -Url "http://127.0.0.1:8792/health" -TimeoutSec 10
if (-not $hqSmoke.Ok) {
    Write-Fail "Quant dejo de responder en /health antes del smoke radar (revisa logs de Quant)."
    exit 3
}

$sum = Test-HttpOk -Url "http://127.0.0.1:8791/api/radar/dashboard/summary?symbol=SPY" -TimeoutSec 150
if (-not $sum.Ok) {
    Write-Fail "dashboard/summary no 200."
    exit 7
}
if ($sum.Body -match 'demonstration_without_engine') {
    Write-Fail "summary sigue en modo stub (demonstration_without_engine). Revisa QUANT_BASE_URL, ATLAS_QUANT_API_KEY y logs de PUSH."
    exit 8
}
Write-Ok "dashboard/summary sin stub de motor (no demonstration_without_engine)."

$sym = Test-HttpOk -Url "http://127.0.0.1:8791/api/radar/symbols/search?q=AAP&limit=5" -TimeoutSec 45
if (-not $sym.Ok) {
    Write-Fail "symbols/search no 200. Cuerpo: $($sym.Body.Substring(0, [Math]::Min(400, $sym.Body.Length)))"
    exit 6
}
Write-Ok "PUSH /api/radar/symbols/search OK."

if (-not $DisableCamera) {
    $cam = Test-HttpOk -Url "http://127.0.0.1:8792/camera/health" -Headers $hdr -TimeoutSec 45
    if (-not $cam.Ok) {
        Write-Warning "camera/health lento o no OK (continuando): $($cam.Error)"
    } else {
        Write-Ok "Quant /camera/health OK."
    }
} else {
    Write-Ok "Smoke camera/health omitido (-DisableCamera)."
}

$sse = Get-SseHeadBytes -Url "http://127.0.0.1:8791/api/radar/stream?symbol=SPY" -MaxSec 10
if ($sse -notmatch '"sequence"') {
    Write-Warning "SSE: no se detecto JSON con sequence en los primeros bytes (Quant puede estar saturado). Fragmento: $($sse.Substring(0, [Math]::Min(200, $sse.Length)))"
} else {
    Write-Ok "SSE contiene campo sequence (Bloque 4)."
}

$camStatus = Test-HttpOk -Url "http://127.0.0.1:8791/api/radar/camera/status" -TimeoutSec 45
if (-not $camStatus.Ok) {
    Write-Warning "camera/status PUSH no 200 (opcional según build): $($camStatus.Error)"
} else {
    Write-Ok "PUSH /api/radar/camera/status OK."
}

# --- 6. Reporte ---
Write-Host ""
Write-Step "URLs listas:"
Write-Host "  V4 UI:           http://127.0.0.1:8791/ui"
Write-Host "  Radar dashboard: http://127.0.0.1:8791/radar/dashboard"
Write-Host ""
Write-Step "Logs:"
Write-Host "  Quant: $QuantLog (+ errores: $QuantErr)"
Write-Host "  PUSH:  $AtlasRadarPushOutLog (+ errores: $AtlasRadarPushErrLog)"
Write-Host ""
Write-Ok "Stack radar listo."
exit 0

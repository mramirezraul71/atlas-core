# ============================================================================
# ATLAS -- Arranque completo con verificacion de salud
# ============================================================================
# Uso:
#   .\scripts\atlas_start_full.ps1                  # arranque normal
#   .\scripts\atlas_start_full.ps1 -CleanCache       # limpia cache antes de arrancar
#   .\scripts\atlas_start_full.ps1 -Robot            # solo verifica/arranca Robot
#   .\scripts\atlas_start_full.ps1 -StatusOnly       # solo muestra estado actual
#   .\scripts\atlas_start_full.ps1 -CleanCache -All  # limpieza agresiva + arranque
#
# Lo que hace:
#   1. Verificacion previa del estado de los 3 servicios
#   2. Limpieza opcional de cache Python / WAL / locks
#   3. Arranque secuencial: Robot (8002) luego PUSH (8791)
#   4. Verificacion post-arranque con reintentos
#   5. Reporte de salud final en consola
# ============================================================================

param(
    [switch]$CleanCache,   # Limpiar __pycache__, WAL huerfanos, git locks antes de arrancar
    [switch]$All,          # Limpieza agresiva (implica -CleanCache)
    [switch]$Robot,        # Solo verifica/arranca el Robot backend
    [switch]$StatusOnly,   # Solo mostrar estado, no arrancar nada
    [switch]$Force         # Forzar reinicio aunque el servicio ya este arriba
)

$ErrorActionPreference = "SilentlyContinue"

# ---- Rutas ------------------------------------------------------------------
$RepoRoot   = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$ScriptsDir = Join-Path $RepoRoot "scripts"
$FreePort   = Join-Path $ScriptsDir "free_port.ps1"
$CleanPy    = Join-Path $ScriptsDir "atlas_clean_cache.py"
$StartSvc   = Join-Path $ScriptsDir "start_nexus_services.py"
$Python     = if ($env:PYTHON) { $env:PYTHON } else { "python" }

$PUSH_URL   = "http://127.0.0.1:8791/health"
$ROBOT_URL1 = "http://127.0.0.1:8002/api/health"
$ROBOT_URL2 = "http://127.0.0.1:8002/status"

# ---- Helpers de salida ------------------------------------------------------
function Write-Ok($msg)   { Write-Host "  [OK]  $msg" -ForegroundColor Green  }
function Write-Warn($msg) { Write-Host "  [!!]  $msg" -ForegroundColor Yellow }
function Write-Fail($msg) { Write-Host "  [XX]  $msg" -ForegroundColor Red    }
function Write-Info($msg) { Write-Host "  [--]  $msg" -ForegroundColor Cyan   }
function Write-Sep()      { Write-Host ("-" * 60)    -ForegroundColor DarkGray }

# ---- Health probe -----------------------------------------------------------
function Test-Endpoint {
    param([string[]]$Urls, [int]$TimeoutSec = 8)
    foreach ($url in $Urls) {
        try {
            $resp = Invoke-WebRequest -Uri $url -TimeoutSec $TimeoutSec -UseBasicParsing -EA Stop
            if ($resp.StatusCode -eq 200) { return $true }
        } catch {}
    }
    return $false
}

function Wait-Endpoint {
    param([string[]]$Urls, [int]$MaxWaitSec = 45, [int]$PollSec = 3, [string]$Label = "Servicio")
    $elapsed = 0
    while ($elapsed -lt $MaxWaitSec) {
        if (Test-Endpoint -Urls $Urls) { return $true }
        Write-Host "    ... esperando $Label ($elapsed/$MaxWaitSec s)" -ForegroundColor DarkGray
        Start-Sleep -Seconds $PollSec
        $elapsed += $PollSec
    }
    return $false
}

# ---- Estado actual -----------------------------------------------------------
function Get-ServiceStatus {
    $robot = Test-Endpoint -Urls @($ROBOT_URL1, $ROBOT_URL2)
    $push  = Test-Endpoint -Urls @($PUSH_URL)
    return @{ robot = $robot; push = $push }
}

function Show-Status {
    param([hashtable]$Status)
    Write-Sep
    if ($Status.robot) {
        Write-Host "  Robot Backend   [ONLINE]  :8002" -ForegroundColor Green
    } else {
        Write-Host "  Robot Backend   [OFFLINE] :8002" -ForegroundColor Red
    }
    if ($Status.push) {
        Write-Host "  PUSH Dashboard  [ONLINE]  :8791" -ForegroundColor Green
    } else {
        Write-Host "  PUSH Dashboard  [OFFLINE] :8791" -ForegroundColor Red
    }
    Write-Sep
}

# ============================================================================
# INICIO
# ============================================================================
$ts = Get-Date -Format "yyyy-MM-dd HH:mm:ss"
Write-Host ""
Write-Host ("=" * 60) -ForegroundColor Cyan
Write-Host "  ATLAS -- Arranque Full  ($ts)"     -ForegroundColor Cyan
Write-Host "  Repo: $RepoRoot"                    -ForegroundColor DarkGray
Write-Host ("=" * 60) -ForegroundColor Cyan

# ---- 1. Estado inicial ------------------------------------------------------
Write-Host ""
Write-Info "Verificando estado actual..."
$initial = Get-ServiceStatus
Show-Status -Status $initial

if ($StatusOnly) {
    Write-Host "  [StatusOnly] Saliendo sin modificar nada." -ForegroundColor DarkGray
    exit 0
}

# ---- 2. Limpieza de cache ---------------------------------------------------
if ($CleanCache -or $All) {
    Write-Host ""
    Write-Info "Limpiando cache del sistema..."

    # Borrar git lock si existe
    $gitLock = Join-Path $RepoRoot ".git\index.lock"
    if (Test-Path $gitLock) {
        Remove-Item $gitLock -Force -EA SilentlyContinue
        Write-Warn "git index.lock eliminado"
    }

    # Ejecutar limpiador Python
    if (Test-Path $CleanPy) {
        $cleanArgs = @("--execute", "--logs-days", "3")
        if ($All) { $cleanArgs += "--all" }
        $cleanResult = & $Python $CleanPy @cleanArgs 2>&1
        $cleanResult | ForEach-Object { Write-Host "    $_" -ForegroundColor DarkGray }
        Write-Ok "Limpieza completada"
    } else {
        # Fallback: solo __pycache__
        Write-Warn "atlas_clean_cache.py no encontrado. Limpieza basica..."
        $cacheDirs = @("atlas_adapter","modules","core","brain","nexus","scripts")
        foreach ($d in $cacheDirs) {
            $full = Join-Path $RepoRoot $d
            if (Test-Path $full) {
                Get-ChildItem -Path $full -Recurse -Directory -Filter "__pycache__" -EA SilentlyContinue |
                    Remove-Item -Recurse -Force -EA SilentlyContinue
            }
        }
        $tempCache = Join-Path $RepoRoot "temp_models_cache"
        if (Test-Path $tempCache) {
            Remove-Item -Path "$tempCache\*" -Recurse -Force -EA SilentlyContinue
        }
        Write-Ok "__pycache__ y temp_models_cache limpiados"
    }
}

# ---- 3. Arranque de servicios -----------------------------------------------

# Robot (8002)
if (-not $initial.robot -or $Force) {
    Write-Host ""
    Write-Info "Arrancando Robot backend (puerto 8002)..."

    # Liberar puerto si esta ocupado por proceso zombie
    if (Test-Path $FreePort) {
        & $FreePort -Port 8002 -Kill | Out-Null
        Start-Sleep -Milliseconds 800
    }

    # Usar el script Python de arranque (robusto, mismo que usa el POT)
    if (Test-Path $StartSvc) {
        $out = & $Python $StartSvc "--robot-only" 2>&1
        Write-Host "    Robot start: $out" -ForegroundColor DarkGray
    } else {
        # Fallback directo
        $robotPath = if ($env:NEXUS_ROBOT_PATH) { $env:NEXUS_ROBOT_PATH } `
                     else { Join-Path $RepoRoot "nexus\atlas_nexus_robot\backend" }
        if (Test-Path "$robotPath\main.py") {
            Start-Process $Python -ArgumentList "main.py" -WorkingDirectory $robotPath -WindowStyle Hidden
        }
    }

    # Esperar hasta que responda
    $robotOk = Wait-Endpoint -Urls @($ROBOT_URL1, $ROBOT_URL2) -MaxWaitSec 45 -Label "Robot :8002"
    if ($robotOk) {
        Write-Ok "Robot backend ONLINE :8002"
    } else {
        Write-Fail "Robot backend no respondio en 45s -- revisar logs/robot_backend.log"
    }

} else {
    Write-Ok "Robot backend ya esta ONLINE :8002 (sin reinicio)"
}

# Si es modo Robot-only, terminar
if ($Robot) {
    Write-Host ""
    Write-Info "Modo -Robot: finalizado."
    $final = Get-ServiceStatus
    Show-Status -Status $final
    exit 0
}

# PUSH (8791) -- Solo verificar; PUSH generalmente ya corre en foreground
if (-not $initial.push -or $Force) {
    Write-Host ""
    Write-Info "PUSH Dashboard (8791) no detectado."

    if (-not $Force) {
        Write-Warn "PUSH normalmente corre en foreground. Ejecutalo manualmente si no esta arriba:"
        Write-Host "    cd $RepoRoot" -ForegroundColor DarkGray
        Write-Host "    python -m uvicorn atlas_adapter.atlas_http_api:app --host 0.0.0.0 --port 8791" -ForegroundColor DarkGray
    } else {
        # Force: arrancar en background (solo para testing, no recomendado en produccion)
        Write-Warn "Forzando arranque de PUSH en segundo plano..."
        if (Test-Path $FreePort) {
            & $FreePort -Port 8791 -Kill | Out-Null
            Start-Sleep -Milliseconds 800
        }
        $pushArgs = "-m uvicorn atlas_adapter.atlas_http_api:app --host 0.0.0.0 --port 8791"
        Start-Process $Python -ArgumentList $pushArgs -WorkingDirectory $RepoRoot -WindowStyle Hidden
        $pushOk = Wait-Endpoint -Urls @($PUSH_URL) -MaxWaitSec 30 -Label "PUSH :8791"
        if ($pushOk) {
            Write-Ok "PUSH Dashboard ONLINE :8791"
        } else {
            Write-Fail "PUSH no respondio en 30s"
        }
    }
} else {
    Write-Ok "PUSH Dashboard ya esta ONLINE :8791"
}

# ---- 4. Verificacion final --------------------------------------------------
Write-Host ""
Write-Info "Verificacion final de salud..."
Start-Sleep -Seconds 2
$final = Get-ServiceStatus
Show-Status -Status $final

$allOk = $final.robot -and $final.push
if ($allOk) {
    Write-Host "  [ATLAS OPERATIVO] Todos los servicios responden." -ForegroundColor Green
    Write-Host "     Dashboard: http://localhost:8791/ui"            -ForegroundColor DarkGray
    Write-Host "     Workspace: http://localhost:8791/workspace"     -ForegroundColor DarkGray
} else {
    Write-Host "  [ATLAS PARCIAL] Algunos servicios no responden." -ForegroundColor Yellow
    if (-not $final.robot) {
        Write-Fail "Robot :8002 -- revisar logs/robot_backend.log"
    }
    if (-not $final.push)  {
        Write-Fail "PUSH :8791  -- arrancar con: python -m uvicorn atlas_adapter.atlas_http_api:app --port 8791"
    }
}

Write-Host ""
if ($allOk) { exit 0 } else { exit 1 }

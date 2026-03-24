<#
.SYNOPSIS
    ATLAS -- Actualizacion automatica de todas las herramientas y software.

.DESCRIPTION
    1. Escanea el registro maestro via atlas_tools_watchdog.py.
    2. Filtra herramientas con update_ready=true (o todas con -All).
    3. Actualiza cada herramienta via atlas_tool_update.ps1.
    4. Registra cada resultado en:
       - logs/ops_bus.log             (bitacora central de operaciones)
       - logs/ans_evolution_bitacora.json  (modulo bitacora del dashboard)
       - logs/snapshot_safe_diagnostic.log
       - logs/atlas_auto_update_report_<fecha>.json (informe detallado)
    5. Refresca el registro maestro con versiones post-actualizacion.

.PARAMETER All
    Actualizar todas las herramientas, no solo las que tienen upgrade disponible.

.PARAMETER DryRun
    Simula el proceso sin ejecutar actualizaciones reales.

.PARAMETER SkipSnapshot
    Omite snapshot de seguridad por herramienta (mas rapido).

.PARAMETER Category
    Filtra por categoria: trading, framework, software, network, runtime, dependency...

.EXAMPLE
    .\scripts\atlas_auto_update_all.ps1
    .\scripts\atlas_auto_update_all.ps1 -All
    .\scripts\atlas_auto_update_all.ps1 -Category trading
    .\scripts\atlas_auto_update_all.ps1 -DryRun
#>

param(
    [switch]$All,
    [switch]$DryRun,
    [switch]$SkipSnapshot,
    [string]$Category = "",
    [string]$RepoRoot = $(Resolve-Path (Join-Path $PSScriptRoot "..")).Path
)

$ErrorActionPreference = "Continue"
Set-StrictMode -Off

# ======================================================================
# CONFIGURACION
# ======================================================================

$SCRIPT_VERSION  = "1.0.0"
$LOG_DIR         = Join-Path $RepoRoot "logs"
$OPS_BUS_LOG     = Join-Path $LOG_DIR "ops_bus.log"
$DIAG_LOG        = Join-Path $LOG_DIR "snapshot_safe_diagnostic.log"
$BITACORA_FILE   = Join-Path $LOG_DIR "ans_evolution_bitacora.json"
$UPDATE_SCRIPT   = Join-Path $RepoRoot "scripts\atlas_tool_update.ps1"
$WATCHDOG_SCRIPT = Join-Path $RepoRoot "scripts\atlas_tools_watchdog.py"
$REPORT_DATE     = Get-Date -Format "yyyy-MM-dd_HH-mm-ss"
$REPORT_FILE     = Join-Path $LOG_DIR ("atlas_auto_update_report_" + $REPORT_DATE + ".json")
$LATEST_LINK     = Join-Path $LOG_DIR "atlas_auto_update_report_latest.json"

# Python del venv
$PYTHON = $null
foreach ($c in @(
    (Join-Path $RepoRoot "venv\Scripts\python.exe"),
    (Join-Path $RepoRoot ".venv\Scripts\python.exe")
)) {
    if (Test-Path $c) { $PYTHON = $c; break }
}
if (-not $PYTHON) { $PYTHON = "python" }

New-Item -ItemType Directory -Path $LOG_DIR -Force | Out-Null

# ======================================================================
# HELPERS DE LOGGING
# ======================================================================

function _Ts {
    (Get-Date).ToUniversalTime().ToString("yyyy-MM-ddTHH:mm:ssZ")
}

function _OpsBusLog {
    param([string]$subsystem, [string]$msg, [string]$level = "info")
    $entry = ("{0} [{1}] [{2}] {3}" -f (_Ts), $level.ToUpper().PadRight(8), $subsystem, $msg)
    try { $entry | Out-File -FilePath $OPS_BUS_LOG -Append -Encoding utf8 } catch {}
}

function _DiagLog {
    param([string]$msg)
    try { ("{0} AUTO_UPDATE {1}" -f (_Ts), $msg) | Out-File -FilePath $DIAG_LOG -Append -Encoding utf8 } catch {}
}

function _BitacoraLog {
    param([string]$msg, [bool]$ok = $true)
    try {
        $msgSafe = if ($msg.Length -gt 500) { $msg.Substring(0, 500) } else { $msg }
        $entry = @{
            timestamp = _Ts
            message   = $msgSafe
            ok        = $ok
            source    = "auto_update"
        }
        $entries = New-Object System.Collections.ArrayList
        if (Test-Path $BITACORA_FILE) {
            $raw = Get-Content $BITACORA_FILE -Raw -ErrorAction SilentlyContinue
            if ($raw -and $raw.Trim()) {
                $parsed = $raw | ConvertFrom-Json -ErrorAction SilentlyContinue
                if ($parsed) {
                    foreach ($e in @($parsed)) { [void]$entries.Add($e) }
                }
            }
        }
        [void]$entries.Add($entry)
        # Mantener ultimas 500 entradas
        while ($entries.Count -gt 500) { $entries.RemoveAt(0) }
        ($entries.ToArray() | ConvertTo-Json -Depth 5) | Set-Content -Path $BITACORA_FILE -Encoding UTF8
    } catch {}
}

function _Banner {
    param([string]$text, [string]$color = "Cyan")
    $line = "=" * 70
    Write-Host ""
    Write-Host ("  " + $line) -ForegroundColor $color
    Write-Host ("    " + $text) -ForegroundColor $color
    Write-Host ("  " + $line) -ForegroundColor $color
    Write-Host ""
}

function _Status {
    param([string]$icon, [string]$name, [string]$msg, [string]$color = "White")
    Write-Host ("  {0}  {1,-34} {2}" -f $icon, $name, $msg) -ForegroundColor $color
}

# ======================================================================
# PASO 1 -- Escanear registro actual
# ======================================================================

_Banner ("ATLAS AUTO-UPDATE  v" + $SCRIPT_VERSION + "  --  " + (_Ts)) "Magenta"
_OpsBusLog "AUTO_UPDATE" ("Inicio ciclo actualizacion -- DryRun=" + $DryRun + " All=" + $All + " Category=" + $Category) "info"
_BitacoraLog ("[AUTO-UPDATE] Inicio -- DryRun=" + $DryRun + " All=" + $All + " Category=" + $Category) $true
_DiagLog ("START DryRun=" + $DryRun + " All=" + $All + " Category=" + $Category)

Write-Host "  Escaneando registro de herramientas..." -ForegroundColor DarkCyan

$registryJson = $null
try {
    $raw = & $PYTHON $WATCHDOG_SCRIPT --force 2>$null
    $line = ($raw -split "`r?`n" | Where-Object { $_.Trim() -and $_.TrimStart().StartsWith("{") } | Select-Object -Last 1)
    if (-not $line) { throw "watchdog no devolvio JSON" }
    $registryJson = $line | ConvertFrom-Json
} catch {
    $errMsg = $_.Exception.Message
    Write-Host ("  ERROR: No se pudo ejecutar el watchdog: " + $errMsg) -ForegroundColor Red
    _OpsBusLog "AUTO_UPDATE" ("ERROR ejecutando watchdog: " + $errMsg) "high"
    _BitacoraLog ("[AUTO-UPDATE] ERROR critico ejecutando watchdog: " + $errMsg) $false
    exit 1
}

$allTools = @($registryJson.tools)
Write-Host ("  Registro cargado: {0} herramientas totales" -f $allTools.Count) -ForegroundColor DarkGray

# ======================================================================
# PASO 2 -- Filtrar herramientas a actualizar
# ======================================================================

$candidates = $allTools | Where-Object {
    $t = $_
    if (-not $t.update_script) { return $false }
    if ($Category -and $t.category -ne $Category) { return $false }
    if ($All) { return $true }
    return [bool]$t.update_ready
}

if (-not $candidates -or @($candidates).Count -eq 0) {
    _Banner "Todo al dia -- No hay actualizaciones disponibles" "Green"
    _OpsBusLog "AUTO_UPDATE" "Ciclo completado: no hay herramientas pendientes" "info"
    _BitacoraLog "[AUTO-UPDATE] Todo al dia -- sin actualizaciones pendientes" $true
    _DiagLog "NOOP -- no candidates"
    exit 0
}

$candidateList = @($candidates)
Write-Host ("  Herramientas a actualizar: {0}" -f $candidateList.Count) -ForegroundColor Yellow
Write-Host ""

# Orden: criticas primero, luego por categoria
$ordered = $candidateList | Sort-Object @{e={if($_.critical){0}else{1}}}, category, id

# ======================================================================
# PASO 3 -- Actualizar herramienta por herramienta
# ======================================================================

$results    = New-Object System.Collections.ArrayList
$ok_count   = 0
$fail_count = 0
$skip_count = 0

foreach ($tool in $ordered) {
    $tid     = $tool.id
    $tname   = $tool.name
    $vBefore = [string]($tool.version)
    $vLatest = [string]($tool.latest_version)
    $cat     = [string]($tool.category)

    if ($DryRun) {
        _Status "[DRY]" $tname ("[DRY-RUN] " + $vBefore + " -> " + $vLatest) "DarkYellow"
        [void]$results.Add(@{
            id = $tid; name = $tname; category = $cat
            version_before = $vBefore; version_latest = $vLatest; version_after = $vBefore
            status = "dry_run"; ok = $true; duration_sec = 0; updated_at = (_Ts)
        })
        $skip_count++
        continue
    }

    Write-Host ("  >> {0,-36} {1} -> {2}" -f $tname, $vBefore, $vLatest) -ForegroundColor Cyan
    _OpsBusLog "AUTO_UPDATE" ("Iniciando: " + $tname + " " + $vBefore + " -> " + $vLatest) "info"
    _DiagLog ("UPDATE_START tool=" + $tid + " from=" + $vBefore + " to=" + $vLatest)

    $t0       = Get-Date
    $updateOk = $false
    $errMsg   = ""
    $rawOut   = ""

    try {
        if ($SkipSnapshot) {
            $rawOut = & powershell -NoProfile -ExecutionPolicy Bypass -File $UPDATE_SCRIPT -Tool $tid -RepoRoot $RepoRoot -SkipSnapshot 2>&1 | Out-String
        } else {
            $rawOut = & powershell -NoProfile -ExecutionPolicy Bypass -File $UPDATE_SCRIPT -Tool $tid -RepoRoot $RepoRoot 2>&1 | Out-String
        }
        $lastLine = ($rawOut -split "`r?`n" | Where-Object { $_.Trim() -and $_.TrimStart().StartsWith("{") } | Select-Object -Last 1)
        if ($lastLine) {
            $parsed   = $lastLine | ConvertFrom-Json -ErrorAction SilentlyContinue
            $updateOk = [bool]($parsed -and $parsed.ok)
            if (-not $updateOk -and $parsed -and $parsed.steps) {
                $failStep = @($parsed.steps) | Where-Object { -not $_.ok } | Select-Object -First 1
                if ($failStep) { $errMsg = [string]$failStep.error }
            }
        } else {
            $updateOk = ($LASTEXITCODE -eq 0)
            if (-not $updateOk) {
                $errMsg = ($rawOut -split "`r?`n" | Where-Object { $_ } | Select-Object -Last 1)
            }
        }
    } catch {
        $updateOk = $false
        $errMsg   = $_.Exception.Message
    }

    $durSec = [Math]::Round(((Get-Date) - $t0).TotalSeconds, 1)

    # Obtener version post-instalacion
    $vAfter = $vBefore
    try {
        $postRaw = & $PYTHON -c ("import importlib.metadata as m; print(m.version('" + $tid + "'))") 2>$null
        if ($postRaw -and $postRaw.Trim()) {
            $vAfter = ($postRaw -split "`r?`n" | Where-Object { $_ } | Select-Object -Last 1).Trim()
        }
    } catch {}

    if ($updateOk) {
        $ok_count++
        _Status "[OK]" $tname ("OK  " + $vBefore + " -> " + $vAfter + "  (" + $durSec + "s)") "Green"
        _OpsBusLog "AUTO_UPDATE" ("OK: " + $tname + " " + $vBefore + " -> " + $vAfter + " (" + $durSec + "s)") "info"
        _BitacoraLog ("[AUTO-UPDATE] OK: " + $tname + " " + $vBefore + " -> " + $vAfter) $true
        _DiagLog ("UPDATE_OK tool=" + $tid + " before=" + $vBefore + " after=" + $vAfter + " dur=" + $durSec + "s")
    } else {
        $fail_count++
        $errShort = if ($errMsg) {
            if ($errMsg.Length -gt 120) { $errMsg.Substring(0, 120) } else { $errMsg }
        } else { "error_desconocido" }
        _Status "[!]" $tname ("FALLO  " + $errShort) "Red"
        _OpsBusLog "AUTO_UPDATE" ("FALLO: " + $tname + " " + $vBefore + " -> " + $vLatest + " -- " + $errShort) "high"
        _BitacoraLog ("[AUTO-UPDATE] FALLO: " + $tname + " -- " + $errShort) $false
        _DiagLog ("UPDATE_FAIL tool=" + $tid + " error=" + $errShort)
    }

    [void]$results.Add(@{
        id             = $tid
        name           = $tname
        category       = $cat
        critical       = [bool]$tool.critical
        version_before = $vBefore
        version_latest = $vLatest
        version_after  = $vAfter
        status         = if ($updateOk) { "updated" } else { "failed" }
        ok             = $updateOk
        error          = if ($updateOk) { $null } else { $errMsg }
        duration_sec   = $durSec
        updated_at     = (_Ts)
    })
}

# ======================================================================
# PASO 4 -- Refrescar registro con versiones actualizadas
# ======================================================================

if (-not $DryRun) {
    Write-Host ""
    Write-Host "  Refrescando registro maestro..." -ForegroundColor DarkCyan
    try {
        & $PYTHON $WATCHDOG_SCRIPT --force 2>$null | Out-Null
        _DiagLog "REGISTRY_REFRESHED"
    } catch {
        _DiagLog ("REGISTRY_REFRESH_FAILED: " + $_.Exception.Message)
    }
}

# ======================================================================
# PASO 5 -- Generar informe JSON
# ======================================================================

$report = @{
    script_version = $SCRIPT_VERSION
    started_at     = $REPORT_DATE
    finished_at    = (_Ts)
    dry_run        = [bool]$DryRun
    repo_root      = $RepoRoot
    python         = $PYTHON
    summary = @{
        total_candidates = ($ok_count + $fail_count + $skip_count)
        updated          = $ok_count
        failed           = $fail_count
        dry_run_skipped  = $skip_count
    }
    results = $results.ToArray()
}

try {
    $reportJson = $report | ConvertTo-Json -Depth 10
    $reportJson | Set-Content -Path $REPORT_FILE -Encoding UTF8
    Copy-Item -Path $REPORT_FILE -Destination $LATEST_LINK -Force -ErrorAction SilentlyContinue
    _DiagLog ("REPORT_WRITTEN path=" + $REPORT_FILE)
} catch {
    _DiagLog ("REPORT_WRITE_FAILED: " + $_.Exception.Message)
}

# ======================================================================
# PASO 6 -- Resumen final
# ======================================================================

Write-Host ""
$summaryColor = if ($fail_count -eq 0) { "Green" } elseif ($ok_count -gt 0) { "Yellow" } else { "Red" }
_Banner ("COMPLETADO  [OK] " + $ok_count + " actualizados   [!] " + $fail_count + " fallidos   [DRY] " + $skip_count + " dry-run") $summaryColor

if ($results.Count -gt 0) {
    Write-Host "  RESUMEN POR HERRAMIENTA:" -ForegroundColor DarkGray
    Write-Host ("  {0,-34} {1,-12} {2,-14} {3,-14} {4}" -f "Herramienta","Categoria","Antes","Despues","Estado") -ForegroundColor DarkGray
    Write-Host ("  " + ("-" * 90)) -ForegroundColor DarkGray
    foreach ($r in $results.ToArray()) {
        $stColor = if ($r.status -eq "updated") { "Green" } elseif ($r.status -eq "dry_run") { "DarkYellow" } else { "Red" }
        $stIcon  = if ($r.status -eq "updated") { "[OK]" } elseif ($r.status -eq "dry_run") { "[DRY]" } else { "[!]" }
        Write-Host ("  {0,-34} {1,-12} {2,-14} {3,-14} {4} {5}" -f `
            $r.name, $r.category, $r.version_before, $r.version_after, $stIcon, $r.status.ToUpper()
        ) -ForegroundColor $stColor
    }
    Write-Host ""
}

Write-Host ("  Informe completo : logs\atlas_auto_update_report_" + $REPORT_DATE + ".json") -ForegroundColor DarkGray
Write-Host ("  Ultimo informe   : logs\atlas_auto_update_report_latest.json") -ForegroundColor DarkGray
Write-Host ("  Bitacora ops     : logs\ops_bus.log") -ForegroundColor DarkGray
Write-Host ("  Bitacora dashboard: logs\ans_evolution_bitacora.json") -ForegroundColor DarkGray
Write-Host ("  Diagnostico      : logs\snapshot_safe_diagnostic.log") -ForegroundColor DarkGray
Write-Host ""

$summaryMsg = ("[AUTO-UPDATE] Completado -- OK=" + $ok_count + " FAIL=" + $fail_count + " DRY=" + $skip_count + " informe=" + $REPORT_FILE)
_OpsBusLog "AUTO_UPDATE" $summaryMsg "info"
_BitacoraLog $summaryMsg ($fail_count -eq 0)
_DiagLog ("FINISH ok=" + $ok_count + " fail=" + $fail_count + " skip=" + $skip_count)

exit $(if ($fail_count -gt 0) { 1 } else { 0 })

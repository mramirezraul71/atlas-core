<#
.SYNOPSIS
  ATLAS PUSH — Monitoreo y actualización automática del repositorio.

.DESCRIPTION
  Ejecuta el sistema de monitoreo/actualización del repo (tarea asignada a este script).
  Incluye filtros Git, comandos y acciones ante errores según config/repo_monitor.yaml.

  Modos:
    -Cycle       Un ciclo: fetch, status, opcional pull (por defecto).
    -AfterFix    Tras arreglos: commit + push de cambios filtrados.
    -StatusOnly  Solo estado (branch, head, remote).

  Si da error se aplican las acciones configuradas en on_error (reintentos, log, abort, reset).

.PARAMETER Cycle
  Ejecutar un ciclo de monitoreo (fetch + status [+ pull si config).

.PARAMETER AfterFix
  Commit y push de cambios no excluidos (tras arreglos).

.PARAMETER StatusOnly
  Solo mostrar estado del repo.

.PARAMETER Message
  Mensaje de commit (solo con -AfterFix).

.PARAMETER Config
  Ruta al YAML de configuración (por defecto config/repo_monitor.yaml).

.PARAMETER Verbose
  Mostrar salida del script Python en consola.

.EXAMPLE
  .\run_repo_monitor.ps1 -Cycle
  .\run_repo_monitor.ps1 -AfterFix -Message "fix: corrección ANS"
  .\run_repo_monitor.ps1 -StatusOnly
#>
param(
    [switch]$Cycle,
    [switch]$AfterFix,
    [switch]$StatusOnly,
    [string]$Message = "",
    [string]$Config = "",
    [switch]$Verbose
)

$ErrorActionPreference = "Stop"
$root = if ($PSScriptRoot) { Split-Path $PSScriptRoot -Parent } else { Get-Location }.Path
if (-not $root) { $root = (Get-Location).Path }

# Config por defecto
$configPath = if ($Config) { $Config } else { Join-Path $root "config\repo_monitor.yaml" }
$pythonExe = "python"
if (Get-Command python3 -ErrorAction SilentlyContinue) { $pythonExe = "python3" }

$env:ATLAS_PUSH_ROOT = $root
$env:ATLAS_REPO_PATH = $root
if ($Config) { $env:REPO_MONITOR_CONFIG = $Config }
else { $env:REPO_MONITOR_CONFIG = $configPath }
if ($Verbose) { $env:REPO_MONITOR_VERBOSE = "1" }

$argsList = @()
if ($AfterFix) {
    $argsList += "--after-fix"
    if ($Message) { $argsList += "-m"; $argsList += $Message }
} elseif ($StatusOnly) {
    $argsList += "--status-only"
} else {
    $argsList += "--cycle"
}

try {
    Push-Location $root
    & $pythonExe (Join-Path $root "scripts\repo_monitor.py") @argsList
    $exitCode = $LASTEXITCODE
} catch {
    Write-Error "repo_monitor: $_"
    $exitCode = 1
} finally {
    Pop-Location
}

# Acciones según código de salida (configurables; aquí solo salida coherente)
# 0 = OK, 1 = error operación (fetch/pull/push), 2 = no es repo / config
if ($exitCode -ne 0) {
    Write-Host "[repo_monitor] Finalizado con código $exitCode. Ver logs/repo_monitor.log y config/repo_monitor.yaml (on_error)." -ForegroundColor Yellow
}
exit $exitCode

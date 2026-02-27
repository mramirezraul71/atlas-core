# ATLAS_EVOLUTION daemon — Tríada de Crecimiento cada 6h (PyPI, GitHub, Hugging Face).
# Ejecutar en segundo plano en Windows (no abre ventana; logs opcionales).
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent (Split-Path -Parent $PSCommandPath)
$logDir = Join-Path $root "logs"
$logFile = Join-Path $logDir "evolution_daemon.log"
if (-not (Test-Path $logDir)) { New-Item -ItemType Directory -Path $logDir -Force | Out-Null }

$python = Get-Command python -ErrorAction SilentlyContinue
if (-not $python) { $python = Get-Command py -ErrorAction SilentlyContinue }
if (-not $python) { Write-Error "Python no encontrado en PATH" }

$daemonScript = Join-Path $root "evolution_daemon.py"
if (-not (Test-Path $daemonScript)) { Write-Error "No se encuentra evolution_daemon.py en $root" }

# Opción 1: con ventana minimizada y log
# Start-Process -FilePath $python.Source -ArgumentList $daemonScript -WorkingDirectory $root -WindowStyle Minimized -RedirectStandardOutput $logFile -RedirectStandardError (Join-Path $logDir "evolution_daemon.err.log")

# Opción 2: sin ventana (daemon en segundo plano), log a archivo
Start-Process -FilePath $python.Source -ArgumentList $daemonScript -WorkingDirectory $root -WindowStyle Hidden -RedirectStandardOutput $logFile -RedirectStandardError (Join-Path $logDir "evolution_daemon.err.log")
Write-Host "ATLAS_EVOLUTION daemon iniciado en segundo plano. Logs: $logDir"

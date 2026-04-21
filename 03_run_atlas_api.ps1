param(
  [Parameter(Mandatory=$true)][string]$RepoPath,
  [int]$AtlasPort = 8791,
  [string]$BindHost = "127.0.0.1",
  [string]$AppImport = ""  # opcional, ej: atlas_adapter.atlas_http_api:app
)

Write-Host "== Run ATLAS API ==" -ForegroundColor Cyan
Set-Location $RepoPath

$venvAct = Join-Path $RepoPath ".venv\Scripts\Activate.ps1"
if (!(Test-Path $venvAct)) { Write-Error "No existe venv. Ejecuta 01_setup_venv.ps1 primero."; exit 1 }
& $venvAct

# liberar puerto si ocupado
$cons = Get-NetTCPConnection -LocalPort $AtlasPort -ErrorAction SilentlyContinue
if ($cons) {
  $pid = $cons[0].OwningProcess
  Write-Host "Puerto $AtlasPort ocupado por PID $pid. Matando..." -ForegroundColor Yellow
  Stop-Process -Id $pid -Force
  Start-Sleep -Milliseconds 300
}

function Try-Run($importPath) {
  Write-Host "Intentando: $importPath" -ForegroundColor Yellow
  python -m uvicorn $importPath --host $BindHost --port $AtlasPort
  return $LASTEXITCODE
}

if ($AppImport -and $AppImport.Trim().Length -gt 0) {
  exit (Try-Run $AppImport)
}

# Punto de entrada vivo único (ver docs/atlas_push/ARCHITECTURE.md).
# Los antiguos candidatos (bridge.server:app, bridge.atlas_api_min:app,
# modules.atlas_remote_api:app) han sido retirados de la cascada en el
# PR A2; los archivados correspondientes viven en legacy/ y no forman
# parte del perímetro vivo. Si se necesita arrancar uno de ellos de
# forma puntual, usar -AppImport <modulo:app> como override manual.
$LiveApp = "atlas_adapter.atlas_http_api:app"

$code = Try-Run $LiveApp
if ($code -eq 0) { exit 0 }

Write-Error "No pude arrancar ATLAS con $LiveApp (code=$code). Revisa el entorno (venv, dependencias, puerto $AtlasPort) o usa -AppImport <modulo:app> para un override manual."
exit 1

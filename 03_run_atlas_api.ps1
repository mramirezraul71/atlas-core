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
$listeners = Get-NetTCPConnection -LocalPort $AtlasPort -State Listen -ErrorAction SilentlyContinue
if ($listeners) {
  $pids = @($listeners | Select-Object -ExpandProperty OwningProcess | Sort-Object -Unique)
  foreach ($p in $pids) {
    if ($p -and $p -gt 0) {
      Write-Host "Puerto $AtlasPort ocupado por PID $p. Matando..." -ForegroundColor Yellow
      Stop-Process -Id $p -Force -ErrorAction SilentlyContinue
    }
  }
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

# autodetección basada en el repo conocido
$preferred = @(
  "atlas_adapter.atlas_http_api:app",
  "bridge.server:app",
  "bridge.atlas_api_min:app",
  "modules.atlas_remote_api:app"
)

foreach ($cand in $preferred) {
  $code = Try-Run $cand
  if ($code -eq 0) { exit 0 }
  Write-Host "Falló $cand (code=$code). Probando siguiente..." -ForegroundColor DarkYellow
}

Write-Error "No pude arrancar ATLAS. Especifica -AppImport con el módulo correcto, ej: -AppImport atlas_adapter.atlas_http_api:app"
exit 1

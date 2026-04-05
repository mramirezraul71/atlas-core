param(
  [Parameter(Mandatory=$true)][string]$RepoPath,
  [int]$AtlasPort = 8791,
  [string]$BindHost = "127.0.0.1",
  [string]$AppImport = "",  # opcional, ej: atlas_adapter.atlas_http_api:app
  [switch]$DisableSafeStartup
)

Write-Host "== Run ATLAS API ==" -ForegroundColor Cyan
Set-Location $RepoPath

$runtimeHelpers = Join-Path $RepoPath "scripts\atlas_runtime.ps1"
if (!(Test-Path $runtimeHelpers)) { Write-Error "No existe atlas_runtime.ps1 en scripts."; exit 1 }
. $runtimeHelpers

$py = Resolve-AtlasPython -RepoRoot $RepoPath -RequirePreflight
if (!(Test-Path $py)) { Write-Error "No existe python.exe resuelto para ATLAS."; exit 1 }

if (-not $DisableSafeStartup) {
  $env:ATLAS_SAFE_STARTUP = "true"
  if (-not $env:ATLAS_MINIMAL_STARTUP) {
    $env:ATLAS_MINIMAL_STARTUP = "true"
  }
}

Stop-AtlasPythonProcesses -Pattern 'atlas_adapter\.atlas_http_api:app'
Start-Sleep -Milliseconds 300

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
  & $py -m uvicorn $importPath --host $BindHost --port $AtlasPort
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

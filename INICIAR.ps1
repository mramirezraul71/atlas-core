# ATLAS - Iniciar API (simple)
# Doble clic o: .\INICIAR.ps1

$RepoPath = "C:\ATLAS_PUSH"
$Port = 8791

Set-Location $RepoPath

# Activar venv
$venv = Join-Path $RepoPath ".venv\Scripts\Activate.ps1"
if (!(Test-Path $venv)) { Write-Host "Ejecuta 01_setup_venv.ps1 primero." -ForegroundColor Red; pause; exit 1 }
& $venv

# Liberar puerto (matar todos los procesos que lo usen)
$cons = Get-NetTCPConnection -LocalPort $Port -ErrorAction SilentlyContinue
if ($cons) {
  $pids = $cons | ForEach-Object { $_.OwningProcess } | Sort-Object -Unique
  foreach ($procId in $pids) {
    if ($procId -ne $PID -and $procId -gt 0) {
      Write-Host "Liberando puerto $Port (PID $procId)..." -ForegroundColor Yellow
      Stop-Process -Id $procId -Force -ErrorAction SilentlyContinue
    }
  }
  Start-Sleep -Seconds 2
}

Write-Host "ATLAS API en http://127.0.0.1:$Port/ui" -ForegroundColor Green
Start-Process "http://127.0.0.1:$Port/ui"

python -m uvicorn atlas_adapter.atlas_http_api:app --host 127.0.0.1 --port $Port

param(
  [string]$TaskName = "ATLAS_8795_Watchdog"
)

$ErrorActionPreference = "Stop"

$queryArgs = @("/Query", "/TN", $TaskName)
& schtasks.exe @queryArgs | Out-Null
if ($LASTEXITCODE -ne 0) {
  Write-Host "La tarea $TaskName no existe." -ForegroundColor Yellow
  exit 0
}

Write-Host "Eliminando tarea programada: $TaskName" -ForegroundColor Cyan
& schtasks.exe /Delete /TN $TaskName /F
if ($LASTEXITCODE -ne 0) {
  Write-Error "No se pudo eliminar la tarea ($TaskName). Codigo: $LASTEXITCODE"
  exit $LASTEXITCODE
}

Write-Host "Tarea eliminada correctamente." -ForegroundColor Green

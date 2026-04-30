param(
  [string]$TaskName = "ATLAS_8795_Watchdog"
)

$ErrorActionPreference = "Stop"

Write-Host "Consultando estado de tarea: $TaskName" -ForegroundColor Cyan
& schtasks.exe /Query /TN $TaskName /V /FO LIST
if ($LASTEXITCODE -ne 0) {
  Write-Host "Tarea no encontrada o sin permisos para leerla." -ForegroundColor Yellow
  exit $LASTEXITCODE
}

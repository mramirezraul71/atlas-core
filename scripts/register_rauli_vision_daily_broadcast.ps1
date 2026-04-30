param(
  [string]$TaskName = "ATLAS_RAULI_VISION_DAILY_BROADCAST",
  [string]$RunAt = "08:30",
  [string]$ScriptPath = "C:\\ATLAS_PUSH\\scripts\\rauli_vision_daily_broadcast.ps1"
)

$ErrorActionPreference = "Stop"

if (-not (Test-Path $ScriptPath)) {
  throw "No existe script de broadcast: $ScriptPath"
}

if ($RunAt -notmatch '^\d{2}:\d{2}$') {
  throw "Formato de hora invalido. Usa HH:mm, por ejemplo 08:30"
}

$taskCmd = "powershell -NoProfile -ExecutionPolicy Bypass -File `"$ScriptPath`""

Write-Host "[TASK] Registrando tarea $TaskName a las $RunAt..." -ForegroundColor Cyan
schtasks /Create /SC DAILY /TN $TaskName /TR $taskCmd /ST $RunAt /F | Out-Null

Write-Host "[TASK] Tarea registrada." -ForegroundColor Green
Write-Host "[TASK] Para ejecutar ahora:" -ForegroundColor DarkGray
Write-Host "schtasks /Run /TN $TaskName" -ForegroundColor DarkGray

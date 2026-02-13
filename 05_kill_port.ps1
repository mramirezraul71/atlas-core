param(
  [Parameter(Mandatory=$false)][int]$Port = 8791
)

Write-Host "== Kill port $Port ==" -ForegroundColor Cyan
$cons = Get-NetTCPConnection -LocalPort $Port -ErrorAction SilentlyContinue
if (!$cons) {
  Write-Host "Puerto $Port ya está libre." -ForegroundColor Green
  exit 0
}

$targetPid = $cons[0].OwningProcess
# Ignorar PID 0 (Idle) y no matar el proceso actual (PowerShell)
if ($targetPid -le 0) {
  Write-Host "Puerto $Port usado por proceso del sistema (PID $targetPid). Ignorando." -ForegroundColor Yellow
  exit 1
}
if ($targetPid -eq $PID) {
  Write-Host "AVISO: El puerto $Port lo usa esta misma sesión (PID $PID). No se matará para evitar cerrar la terminal." -ForegroundColor Yellow
  Write-Host "Cierra uvicorn con Ctrl+C en la terminal donde corre, o ejecuta este script desde otra terminal." -ForegroundColor Yellow
  exit 2
}
Write-Host "Killing process on port $Port..." -ForegroundColor Yellow
Stop-Process -Id $targetPid -Force
Start-Sleep -Milliseconds 500
Write-Host "Listo." -ForegroundColor Green

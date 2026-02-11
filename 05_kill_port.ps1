param(
  [Parameter(Mandatory=$true)][int]$Port
)

Write-Host "== Kill port $Port ==" -ForegroundColor Cyan
$cons = Get-NetTCPConnection -LocalPort $Port -ErrorAction SilentlyContinue
if (!$cons) { Write-Host "Puerto $Port ya está libre." -ForegroundColor Green; exit 0 }

$targetPid = $cons[0].OwningProcess
# No matar el proceso actual (esta sesión de PowerShell) para no cerrar la terminal
if ($targetPid -eq $PID) {
  Write-Host "AVISO: El puerto $Port lo usa esta misma sesión (PID $PID). No se matará para evitar cerrar la terminal." -ForegroundColor Yellow
  Write-Host "Cierra uvicorn con Ctrl+C en la terminal donde corre, o ejecuta este script desde otra terminal." -ForegroundColor Yellow
  exit 2
}
Write-Host "Matando PID $targetPid usando puerto $Port" -ForegroundColor Yellow
Stop-Process -Id $targetPid -Force
Start-Sleep -Milliseconds 500
Write-Host "Listo." -ForegroundColor Green

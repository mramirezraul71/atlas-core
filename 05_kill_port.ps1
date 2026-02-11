param(
  [Parameter(Mandatory=$true)][int]$Port
)

Write-Host "== Kill port $Port ==" -ForegroundColor Cyan
$cons = Get-NetTCPConnection -LocalPort $Port -ErrorAction SilentlyContinue
if (!$cons) { Write-Host "Puerto $Port ya está libre." -ForegroundColor Green; exit 0 }

$pid = $cons[0].OwningProcess
Write-Host "Matando PID $pid usando puerto $Port" -ForegroundColor Yellow
Stop-Process -Id $pid -Force
Start-Sleep -Milliseconds 300
Write-Host "Listo." -ForegroundColor Green

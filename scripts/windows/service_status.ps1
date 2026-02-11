# Show ATLAS Windows Service status.
param([string]$ServiceName = "ATLAS_PUSH")
$s = Get-Service -Name $ServiceName -ErrorAction SilentlyContinue
if (-not $s) { Write-Host "Servicio $ServiceName no instalado." -ForegroundColor Yellow; exit 0 }
Write-Host "Nombre: $($s.Name) | Estado: $($s.Status) | Inicio: $($s.StartType)" -ForegroundColor Cyan

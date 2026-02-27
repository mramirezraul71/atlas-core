# Stop ATLAS Windows Service.
param([string]$ServiceName = "ATLAS_PUSH")
Stop-Service -Name $ServiceName -Force -ErrorAction SilentlyContinue
Write-Host "Servicio $ServiceName detenido." -ForegroundColor Green

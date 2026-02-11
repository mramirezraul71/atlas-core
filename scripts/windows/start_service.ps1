# Start ATLAS Windows Service.
param([string]$ServiceName = "ATLAS_PUSH")
Start-Service -Name $ServiceName -ErrorAction Stop
Write-Host "Servicio $ServiceName iniciado." -ForegroundColor Green

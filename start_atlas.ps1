# ATLAS Startup Script
cd C:\ATLAS_PUSH

Write-Host "=== INICIANDO ATLAS ===" -ForegroundColor Cyan
Write-Host ""

# 1. WAHA
Write-Host "[1/4] WAHA (WhatsApp)..." -ForegroundColor Yellow
docker start waha 2>$null
Write-Host "  OK" -ForegroundColor Green

# 2. Evolution Daemon
Write-Host "[2/4] Evolution Daemon..." -ForegroundColor Yellow
Start-Process python -ArgumentList "evolution_daemon.py" -WindowStyle Hidden
Write-Host "  OK" -ForegroundColor Green

# 3. NEXUS Services
Write-Host "[3/4] NEXUS Services..." -ForegroundColor Yellow
Start-Process python -ArgumentList "scripts/start_nexus_services.py" -WindowStyle Hidden
Write-Host "  OK" -ForegroundColor Green

# 4. Dashboard
Write-Host "[4/4] Dashboard (8791)..." -ForegroundColor Yellow
$env:PYTHONPATH = "C:\ATLAS_PUSH"
$env:SERVICE_PORT = "8791"
Start-Process python -ArgumentList "tools/service_launcher.py" -WindowStyle Hidden
Write-Host "  OK" -ForegroundColor Green

Write-Host ""
Write-Host "=== ATLAS INICIADO ===" -ForegroundColor Green
Write-Host "Dashboard: http://localhost:8791/ui" -ForegroundColor Cyan
Write-Host "WAHA: http://localhost:3000/dashboard" -ForegroundColor Cyan

Start-Sleep -Seconds 3
Start-Process "http://localhost:8791/ui"

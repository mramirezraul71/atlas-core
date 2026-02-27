# Script PowerShell para iniciar servicios Docker en Atlas
# Home Assistant y Appsmith

Write-Host "=== Configuracion de servicios Docker para Atlas ===" -ForegroundColor Green

# Verificar Docker Desktop
Write-Host "`nVerificando Docker Desktop..." -ForegroundColor Yellow
try {
    $dockerVersion = docker --version
    Write-Host "Docker disponible: $dockerVersion" -ForegroundColor Green
} catch {
    Write-Host "Docker no disponible. Iniciando Docker Desktop..." -ForegroundColor Red
    Start-Process "C:\Program Files\Docker\Docker\Docker Desktop.exe"
    Write-Host "Esperando 30 segundos a que Docker inicie..." -ForegroundColor Yellow
    Start-Sleep -Seconds 30
}

# Crear directorios necesarios
Write-Host "`nCreando directorios de configuracion..." -ForegroundColor Yellow
New-Item -ItemType Directory -Force -Path "c:\ATLAS_PUSH\homeassistant_config" | Out-Null
New-Item -ItemType Directory -Force -Path "c:\ATLAS_PUSH\appsmith_data" | Out-Null

# Limpiar contenedores existentes
Write-Host "`nLimpiando contenedores existentes..." -ForegroundColor Yellow
docker stop atlas-homeassistant 2>$null
docker rm atlas-homeassistant 2>$null
docker stop atlas-appsmith 2>$null
docker rm atlas-appsmith 2>$null

# Descargar imagenes
Write-Host "`nDescargando imagenes Docker..." -ForegroundColor Yellow
docker pull homeassistant/home-assistant:stable
docker pull appsmith/appsmith-ce

# Iniciar Home Assistant
Write-Host "`nIniciando Home Assistant..." -ForegroundColor Yellow
docker run -d --name atlas-homeassistant -p 8123:8123 --restart=unless-stopped -v c:/ATLAS_PUSH/homeassistant_config:/config homeassistant/home-assistant:stable

# Iniciar Appsmith
Write-Host "`nIniciando Appsmith..." -ForegroundColor Yellow
docker run -d --name atlas-appsmith -p 80:80 -p 443:443 -v c:/ATLAS_PUSH/appsmith_data:/appsmith-stacks --restart=unless-stopped appsmith/appsmith-ce

# Verificar estado
Write-Host "`nVerificando estado de los servicios..." -ForegroundColor Yellow
Write-Host "Contenedores en ejecucion:" -ForegroundColor Cyan
docker ps --filter "name=atlas-"

Write-Host "`n=== Servicios configurados ===" -ForegroundColor Green
Write-Host "Home Assistant: http://localhost:8123" -ForegroundColor White
Write-Host "Appsmith: http://localhost" -ForegroundColor White
Write-Host "`nNota: Puede tomar varios minutos en iniciar completamente." -ForegroundColor Yellow

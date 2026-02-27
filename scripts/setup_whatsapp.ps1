# Setup WhatsApp para ATLAS usando WAHA (gratuito)
# Ejecutar como: .\scripts\setup_whatsapp.ps1

Write-Host "=== ATLAS WhatsApp Setup (WAHA) ===" -ForegroundColor Cyan
Write-Host ""

# 1. Verificar Docker
Write-Host "1. Verificando Docker..." -ForegroundColor Yellow
$dockerRunning = $false
try {
    $result = docker info 2>&1
    if ($LASTEXITCODE -eq 0) {
        $dockerRunning = $true
        Write-Host "   Docker OK" -ForegroundColor Green
    }
} catch {}

if (-not $dockerRunning) {
    Write-Host "   Docker no esta corriendo." -ForegroundColor Red
    Write-Host "   Por favor, inicia Docker Desktop y vuelve a ejecutar este script." -ForegroundColor Yellow
    Write-Host ""
    Write-Host "   Presiona cualquier tecla para abrir Docker Desktop..." -ForegroundColor Cyan
    $null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
    Start-Process "C:\Program Files\Docker\Docker\Docker Desktop.exe"
    Write-Host "   Esperando que Docker inicie (esto puede tomar 1-2 minutos)..." -ForegroundColor Yellow
    
    $timeout = 120
    $elapsed = 0
    while ($elapsed -lt $timeout) {
        Start-Sleep -Seconds 5
        $elapsed += 5
        try {
            $result = docker info 2>&1
            if ($LASTEXITCODE -eq 0) {
                $dockerRunning = $true
                Write-Host "   Docker iniciado!" -ForegroundColor Green
                break
            }
        } catch {}
        Write-Host "   Esperando... ($elapsed/$timeout segundos)" -ForegroundColor Gray
    }
    
    if (-not $dockerRunning) {
        Write-Host "   ERROR: Docker no inicio en tiempo." -ForegroundColor Red
        exit 1
    }
}
Write-Host ""

# 2. Verificar/Crear contenedor WAHA
Write-Host "2. Configurando WAHA..." -ForegroundColor Yellow
$wahaExists = docker ps -a --filter "name=waha" --format "{{.Names}}" 2>&1
$wahaRunning = docker ps --filter "name=waha" --format "{{.Names}}" 2>&1

if ($wahaRunning -eq "waha") {
    Write-Host "   WAHA ya esta corriendo" -ForegroundColor Green
} elseif ($wahaExists -eq "waha") {
    Write-Host "   Iniciando WAHA existente..." -ForegroundColor Yellow
    docker start waha
    Write-Host "   WAHA iniciado" -ForegroundColor Green
} else {
    Write-Host "   Descargando e iniciando WAHA (primera vez, puede tomar unos minutos)..." -ForegroundColor Yellow
    docker run -d -p 3000:3000 --name waha --restart unless-stopped devlikeapro/waha
    if ($LASTEXITCODE -eq 0) {
        Write-Host "   WAHA instalado y corriendo" -ForegroundColor Green
    } else {
        Write-Host "   ERROR instalando WAHA" -ForegroundColor Red
        exit 1
    }
}
Write-Host ""

# 3. Esperar a que WAHA este listo
Write-Host "3. Esperando que WAHA este listo..." -ForegroundColor Yellow
Start-Sleep -Seconds 3
$wahaReady = $false
for ($i = 0; $i -lt 12; $i++) {
    try {
        $response = Invoke-WebRequest -Uri "http://localhost:3000/api/sessions" -TimeoutSec 3 -ErrorAction SilentlyContinue
        if ($response.StatusCode -eq 200) {
            $wahaReady = $true
            Write-Host "   WAHA listo!" -ForegroundColor Green
            break
        }
    } catch {}
    Write-Host "   Esperando... ($($i * 5) segundos)" -ForegroundColor Gray
    Start-Sleep -Seconds 5
}

if (-not $wahaReady) {
    Write-Host "   WAHA tardo en iniciar. Verifica en http://localhost:3000" -ForegroundColor Yellow
}
Write-Host ""

# 4. Solicitar numero de telefono
Write-Host "4. Configuracion de tu numero personal" -ForegroundColor Yellow
$phoneNumber = Read-Host "   Ingresa tu numero de WhatsApp (con codigo de pais, ej: +34612345678)"
$phoneNumber = $phoneNumber.Trim()

if (-not $phoneNumber) {
    Write-Host "   Numero no ingresado. Puedes configurarlo despues en credenciales.txt" -ForegroundColor Yellow
    $phoneNumber = "+TUNUMERO"
}
Write-Host ""

# 5. Actualizar credenciales
Write-Host "5. Actualizando configuracion..." -ForegroundColor Yellow

# Buscar archivo de credenciales
$credPaths = @(
    "$env:ATLAS_VAULT_PATH",
    "C:\Users\Raul\OneDrive\RAUL - Personal\Escritorio\credenciales.txt",
    "C:\dev\credenciales.txt"
)

$credFile = $null
foreach ($path in $credPaths) {
    if ($path -and (Test-Path $path)) {
        $credFile = $path
        break
    }
}

$whatsappConfig = @"

# === WhatsApp (WAHA) ===
WHATSAPP_ENABLED=true
WHATSAPP_PROVIDER=waha
WHATSAPP_TO=$phoneNumber
WAHA_API_URL=http://localhost:3000
WAHA_SESSION=default
OPS_WHATSAPP_ENABLED=true
"@

if ($credFile) {
    # Verificar si ya tiene configuracion de WhatsApp
    $content = Get-Content $credFile -Raw -ErrorAction SilentlyContinue
    if ($content -notmatch "WHATSAPP_ENABLED") {
        Add-Content -Path $credFile -Value $whatsappConfig
        Write-Host "   Configuracion agregada a: $credFile" -ForegroundColor Green
    } else {
        Write-Host "   WhatsApp ya configurado en: $credFile" -ForegroundColor Yellow
        Write-Host "   Verifica que WHATSAPP_TO=$phoneNumber" -ForegroundColor Yellow
    }
} else {
    # Crear archivo local
    $localEnv = ".\.env.whatsapp"
    $whatsappConfig | Out-File -FilePath $localEnv -Encoding UTF8
    Write-Host "   Configuracion guardada en: $localEnv" -ForegroundColor Green
    Write-Host "   IMPORTANTE: Copia estas variables a tu credenciales.txt" -ForegroundColor Yellow
}
Write-Host ""

# 6. Instrucciones finales
Write-Host "=== SIGUIENTE PASO ===" -ForegroundColor Cyan
Write-Host ""
Write-Host "Abre tu navegador en:" -ForegroundColor White
Write-Host "   http://localhost:3000" -ForegroundColor Green
Write-Host ""
Write-Host "1. Haz clic en 'default' (o crea una sesion)" -ForegroundColor White
Write-Host "2. Escanea el codigo QR con tu WhatsApp" -ForegroundColor White
Write-Host "   (WhatsApp -> Menu -> Dispositivos vinculados -> Vincular dispositivo)" -ForegroundColor Gray
Write-Host "3. Una vez vinculado, prueba con:" -ForegroundColor White
Write-Host "   Invoke-WebRequest -Uri 'http://localhost:8791/api/comms/whatsapp/test' -Method POST" -ForegroundColor Green
Write-Host ""
Write-Host "Presiona cualquier tecla para abrir el navegador..." -ForegroundColor Cyan
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
Start-Process "http://localhost:3000"

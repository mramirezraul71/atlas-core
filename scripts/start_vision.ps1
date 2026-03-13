#!/usr/bin/env pwsh
# start_vision.ps1 — Arranca RAULI-VISION (espejo + proxy) si no están corriendo
# Uso: .\scripts\start_vision.ps1
# Se puede llamar desde Task Scheduler o desde start_atlas.ps1

$VISION_ROOT = "C:\ATLAS_PUSH\_external\RAULI-VISION"
$ESPEJO_PORT = 8080
$PROXY_PORT  = 3000
$ESPEJO_BIN  = "$VISION_ROOT\espejo\espejo.exe"
$PROXY_BIN   = "$VISION_ROOT\cliente-local\rauli-proxy.exe"
$LOG_DIR     = "C:\ATLAS_PUSH\logs"

function Test-PortListening($port) {
    try {
        $result = Test-NetConnection -ComputerName 127.0.0.1 -Port $port -InformationLevel Quiet -WarningAction SilentlyContinue 2>$null
        return $result
    } catch {
        return $false
    }
}

function Write-Log($msg) {
    $ts = Get-Date -Format "yyyy-MM-ddTHH:mm:ss"
    Write-Host "[$ts] $msg"
    Add-Content -Path "$LOG_DIR\vision_startup.log" -Value "[$ts] $msg" -Encoding UTF8 -ErrorAction SilentlyContinue
}

Write-Log "=== RAULI-VISION startup check ==="

# --- Espejo (puerto 8080) ---
if (-not (Test-PortListening $ESPEJO_PORT)) {
    Write-Log "Espejo no detectado en :$ESPEJO_PORT — arrancando..."
    if (-not (Test-Path $ESPEJO_BIN)) {
        Write-Log "Compilando espejo..."
        Push-Location "$VISION_ROOT\espejo"
        go build -o espejo.exe ./cmd/server/ 2>&1 | Out-Null
        Pop-Location
    }
    # Usar batch wrapper para que ADMIN_TOKEN sea heredado correctamente por espejo.exe
    Start-Process -FilePath "$VISION_ROOT\espejo\run_espejo.bat" -WorkingDirectory "$VISION_ROOT\espejo" `
        -RedirectStandardOutput "$LOG_DIR\vision_espejo.log" `
        -RedirectStandardError  "$LOG_DIR\vision_espejo.err" `
        -WindowStyle Hidden -NoNewWindow:$false
    Start-Sleep -Seconds 3
    if (Test-PortListening $ESPEJO_PORT) {
        Write-Log "Espejo OK en :$ESPEJO_PORT"
    } else {
        Write-Log "ERROR: espejo no responde tras arranque"
    }
} else {
    Write-Log "Espejo ya activo en :$ESPEJO_PORT"
}

# --- Proxy / cliente-local (puerto 3000) ---
if (-not (Test-PortListening $PROXY_PORT)) {
    Write-Log "Proxy no detectado en :$PROXY_PORT — arrancando..."
    if (-not (Test-Path $PROXY_BIN)) {
        Write-Log "Compilando proxy..."
        Push-Location "$VISION_ROOT\cliente-local"
        go build -o rauli-proxy.exe ./cmd/proxy/ 2>&1 | Out-Null
        Pop-Location
    }
    $env:ESPEJO_URL    = "http://127.0.0.1:$ESPEJO_PORT"
    $env:CLIENT_ID     = "rauli-local"
    $env:CLIENT_SECRET = "rauli-local-secret"
    $env:PORT          = $PROXY_PORT
    Start-Process -FilePath $PROXY_BIN -WorkingDirectory "$VISION_ROOT\cliente-local" `
        -RedirectStandardOutput "$LOG_DIR\vision_proxy.log" `
        -RedirectStandardError  "$LOG_DIR\vision_proxy.err" `
        -WindowStyle Hidden -NoNewWindow:$false
    Start-Sleep -Seconds 3
    if (Test-PortListening $PROXY_PORT) {
        Write-Log "Proxy RAULI-VISION OK en :$PROXY_PORT"
    } else {
        Write-Log "ERROR: proxy no responde tras arranque"
    }
} else {
    Write-Log "Proxy ya activo en :$PROXY_PORT"
}

Write-Log "=== Vision check completo ==="

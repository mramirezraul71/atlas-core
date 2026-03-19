#!/usr/bin/env pwsh
# pre_operation_safety.ps1 — Backup y verificación de integridad antes de operar
# Llamar al iniciar el turno o antes de cualquier operación masiva.

$DB_PATH     = "C:\ATLAS_PUSH\_external\rauli-panaderia\backend\database\genesis.db"
$BACKUP_DIR  = "C:\ATLAS_PUSH\_external\rauli-panaderia\backend\database\backups"
$LOG         = "C:\ATLAS_PUSH\logs\safety.log"
$MAX_BACKUPS = 7

function Write-Log($msg) {
    $ts = Get-Date -Format "yyyy-MM-ddTHH:mm:ss"
    Write-Host "[$ts] $msg"
    Add-Content -Path $LOG -Value "[$ts] $msg" -Encoding UTF8 -ErrorAction SilentlyContinue
}

New-Item -ItemType Directory -Path $BACKUP_DIR -Force -ErrorAction SilentlyContinue | Out-Null
Write-Log "=== Pre-operation safety check ==="

# --- 1. Backup de genesis.db ---
if (Test-Path $DB_PATH) {
    $stamp  = Get-Date -Format "yyyyMMdd_HHmmss"
    $dest   = "$BACKUP_DIR\genesis.db.bak_$stamp"
    Copy-Item $DB_PATH $dest -Force
    $size   = (Get-Item $dest).Length
    Write-Log "Backup OK: $dest ($([math]::Round($size/1024))KB)"

    # Limpiar backups viejos (mantener MAX_BACKUPS)
    $backups = Get-ChildItem $BACKUP_DIR -Filter "genesis.db.bak_*" | Sort-Object LastWriteTime
    if ($backups.Count -gt $MAX_BACKUPS) {
        $toDelete = $backups | Select-Object -First ($backups.Count - $MAX_BACKUPS)
        $toDelete | Remove-Item -Force
        Write-Log "Backups antiguos eliminados: $($toDelete.Count)"
    }
} else {
    Write-Log "ADVERTENCIA: genesis.db no encontrada en $DB_PATH"
}

# --- 2. Verificar que panadería responde ---
try {
    $r = Invoke-WebRequest -Uri "http://127.0.0.1:3001/api/health" -TimeoutSec 5 -UseBasicParsing -ErrorAction Stop
    Write-Log "Panaderia backend: OK (3001)"
} catch {
    Write-Log "FALLO: Panaderia backend no responde en :3001"
}

# --- 3. Verificar que Vision responde ---
try {
    $r = Invoke-WebRequest -Uri "http://127.0.0.1:3000/api/health" -TimeoutSec 5 -UseBasicParsing -ErrorAction Stop
    Write-Log "RAULI-VISION proxy: OK (3000)"
} catch {
    Write-Log "Vision no disponible en :3000 — ejecutar start_vision.ps1"
}

# --- 4. Verificar que Atlas responde ---
try {
    $r = Invoke-WebRequest -Uri "http://127.0.0.1:8791/health" -TimeoutSec 5 -UseBasicParsing -ErrorAction Stop
    Write-Log "Atlas API: OK (8791)"
} catch {
    Write-Log "FALLO: Atlas no responde en :8791"
}

# --- 5. Verificar tunnels ---
try {
    $r = Invoke-WebRequest -Uri "https://panaderia.rauliatlasapp.com/api/health" -TimeoutSec 10 -UseBasicParsing -ErrorAction Stop
    Write-Log "Tunnel panaderia: OK"
} catch {
    Write-Log "Tunnel panaderia no disponible (modo local activo)"
}

Write-Log "=== Safety check completo ==="

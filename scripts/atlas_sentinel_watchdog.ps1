# ATLAS Sentinel Watchdog v1.1
# Scanner permanente de todos los servicios criticos con autocorreccion.
# Cubre: panaderia (3001/5173), espejo (3000/8080), atlas (8791), tunel cloudflare.
# Autoarranque via HKCU Run (sin necesidad de Admin).

param(
    [switch]$Register,    # Instalar autoarranque en HKCU Run (sin Admin)
    [switch]$Unregister   # Eliminar autoarranque
)

# ── Autoarranque via registro HKCU (sin permisos de Admin) ───────────────
$RUN_KEY  = "HKCU:\Software\Microsoft\Windows\CurrentVersion\Run"
$RUN_NAME = "ATLAS_Sentinel_Watchdog"
$RUN_VAL  = "powershell.exe -NoProfile -WindowStyle Hidden -ExecutionPolicy Bypass -File `"$PSCommandPath`""

if ($Unregister) {
    Remove-ItemProperty -Path $RUN_KEY -Name $RUN_NAME -ErrorAction SilentlyContinue
    Write-Host "[OK] Autoarranque eliminado."
    exit 0
}

if ($Register) {
    if (-not (Test-Path $RUN_KEY)) { New-Item -Path $RUN_KEY | Out-Null }
    New-ItemProperty -Path $RUN_KEY -Name $RUN_NAME -PropertyType String -Value $RUN_VAL -Force | Out-Null
    Write-Host "[OK] ATLAS_Sentinel_Watchdog registrado en HKCU Run. Se iniciara automaticamente al iniciar sesion."
    exit 0
}

# Evitar instancias duplicadas del loop
$self = $MyInvocation.MyCommand.Path
$duplicates = @(Get-CimInstance Win32_Process -ErrorAction SilentlyContinue | Where-Object {
    $_.Name -match "powershell" -and $_.CommandLine -match [regex]::Escape($self) -and $_.ProcessId -ne $PID
})
if ($duplicates.Count -gt 0) {
    Write-Host "[SKIP] Watchdog ya en ejecucion (PID $($duplicates[0].ProcessId)). Saliendo."
    exit 0
}

# ── Configuracion ─────────────────────────────────────────────────────────
$LOG       = "C:/ATLAS_PUSH/logs/atlas_sentinel_watchdog.log"
$CF_EXE    = "C:/Program Files (x86)/cloudflared/cloudflared.exe"
$CF_CONFIG = "C:/ATLAS_PUSH/config/cloudflared/cloudflare_atlas_config.yaml"
$ATLAS_API = "http://127.0.0.1:8791"
$CHECK_SEC = 20     # comprobar cada 20 segundos
$FAIL_THR  = 2      # fallos consecutivos antes de actuar
$COOL_SEC  = 90     # cooldown entre intentos de reparacion del mismo servicio

# Estado por servicio
$state = @{}

# ── Definicion de servicios a vigilar ─────────────────────────────────────
$SERVICES = @(
    @{
        id      = "panaderia_api"
        label   = "Panaderia API"
        check   = { (http_ok "http://127.0.0.1:3001/api/health") }
        heal    = {
            wlog "HEAL" "Reiniciando panaderia via Atlas arms..."
            $r = atlas_post "/arms/panaderia/start" "{}"
            wlog "HEAL" "arms/panaderia/start -> ok=$($r.ok)"
        }
    },
    @{
        id      = "panaderia_front"
        label   = "Panaderia Frontend"
        check   = { (http_ok "http://127.0.0.1:5173") }
        heal    = {
            wlog "HEAL" "Frontend caido - relanzando via arms..."
            atlas_post "/arms/panaderia/start" "{}" | Out-Null
        }
    },
    @{
        id      = "espejo_proxy"
        label   = "Espejo Proxy"
        check   = { (http_ok "http://127.0.0.1:3000/api/health") }
        heal    = {
            wlog "HEAL" "Espejo proxy caido - intentando restart via arms..."
            atlas_post "/arms/vision/start" "{}" | Out-Null
        }
    },
    @{
        id      = "espejo_go"
        label   = "Espejo Go"
        check   = { (http_ok "http://127.0.0.1:8080/api/health") }
        heal    = {
            wlog "HEAL" "Espejo Go (8080) caido - reiniciando proceso..."
            $exe = (Get-ChildItem "C:/ATLAS_PUSH/_external/RAULI-VISION/espejo/*.exe" -ErrorAction SilentlyContinue | Select-Object -First 1)
            if ($exe) {
                $env:PORT         = "8080"
                $env:JWT_SECRET   = "rauli-vision-espejo-secret"
                $env:ADMIN_TOKEN  = "rauli-admin-local"
                $env:ACCESS_STORE = "data\access-store.json"
                Start-Process -FilePath $exe.FullName -WorkingDirectory $exe.DirectoryName -WindowStyle Hidden -ErrorAction SilentlyContinue
            }
        }
    },
    @{
        id      = "atlas_api"
        label   = "Atlas API"
        check   = {
            try {
                $r = Invoke-WebRequest -Uri "$ATLAS_API/health" -TimeoutSec 5 -UseBasicParsing -ErrorAction Stop
                return ($r.StatusCode -lt 500)
            } catch { return $false }
        }
        heal    = {
            wlog "HEAL" "Atlas API (8791) no responde - alerta critica, revisar manualmente"
        }
    },
    @{
        id      = "cloudflare_tunnel"
        label   = "Cloudflare Tunnel"
        check   = {
            try {
                $m = Invoke-WebRequest -Uri "http://127.0.0.1:20241/metrics" -TimeoutSec 3 -UseBasicParsing -ErrorAction Stop
                if ($m.Content -match 'cloudflared_tunnel_ha_connections\s+(\d+)') {
                    return ([int]$Matches[1] -gt 0)
                }
            } catch {}
            # Si metrics no responde, verificar proceso
            return (@(Get-Process -Name "cloudflared" -ErrorAction SilentlyContinue).Count -gt 0)
        }
        heal    = {
            wlog "HEAL" "Reiniciando cloudflared tunel..."
            Get-Process -Name "cloudflared" -ErrorAction SilentlyContinue | ForEach-Object {
                Stop-Process -Id $_.Id -Force -ErrorAction SilentlyContinue
            }
            Start-Sleep -Seconds 3
            Start-Process -FilePath $CF_EXE -ArgumentList "tunnel","--config",$CF_CONFIG,"run" -WindowStyle Hidden -ErrorAction SilentlyContinue
            wlog "HEAL" "cloudflared relanzado, esperando reconexion..."
        }
    }
)

# ── Helpers ──────────────────────────────────────────────────────────────
function wlog {
    param([string]$lvl, [string]$msg)
    $ts   = Get-Date -Format "yyyy-MM-ddTHH:mm:ss"
    $line = "[$ts][$lvl] $msg"
    Write-Host $line
    try { Add-Content -Path $LOG -Value $line -Encoding UTF8 -ErrorAction Stop } catch {}
}

function http_ok {
    param([string]$url)
    try {
        $r = Invoke-WebRequest -Uri $url -TimeoutSec 5 -UseBasicParsing -ErrorAction Stop
        return ($r.StatusCode -lt 500)
    } catch { return $false }
}

function atlas_post {
    param([string]$path, [string]$body = "{}")
    try {
        $r = Invoke-WebRequest -Uri "$ATLAS_API$path" -Method POST `
            -Body $body -ContentType "application/json" `
            -TimeoutSec 15 -UseBasicParsing -ErrorAction Stop
        return ($r.Content | ConvertFrom-Json)
    } catch { return @{ok=$false; error=$_.Exception.Message} }
}

# ── Inicializar estado por servicio ───────────────────────────────────────
foreach ($svc in $SERVICES) {
    $state[$svc.id] = @{
        fails      = 0
        last_heal  = [datetime]::MinValue
        healed     = 0
    }
}

# ── Loop principal ────────────────────────────────────────────────────────
wlog "INFO" "=== Atlas Sentinel Watchdog iniciado | servicios=$($SERVICES.Count) interval=${CHECK_SEC}s ==="

while ($true) {
    $all_ok   = $true
    $summary  = @()

    foreach ($svc in $SERVICES) {
        $id  = $svc.id
        $ok  = $false
        try { $ok = & $svc.check } catch {}

        if ($ok) {
            if ($state[$id].fails -gt 0) {
                wlog "INFO" "$($svc.label) recuperado (habia $($state[$id].fails) fallos)"
            }
            $state[$id].fails = 0
            $summary += "$id=OK"
        } else {
            $state[$id].fails++
            $all_ok = $false
            $summary += "$id=FAIL($($state[$id].fails))"

            if ($state[$id].fails -ge $FAIL_THR) {
                $secs = ([datetime]::Now - $state[$id].last_heal).TotalSeconds
                if ($secs -ge $COOL_SEC) {
                    $state[$id].last_heal = [datetime]::Now
                    $state[$id].healed++
                    wlog "WARN" "$($svc.label) FALLO #$($state[$id].fails) | intento heal #$($state[$id].healed)"
                    try { & $svc.heal } catch { wlog "ERROR" "Heal $id fallo: $($_.Exception.Message)" }
                    $state[$id].fails = 0
                } else {
                    $remaining = [int]($COOL_SEC - $secs)
                    wlog "WARN" "$($svc.label) FALLO #$($state[$id].fails) | cooldown ${remaining}s"
                }
            }
        }
    }

    if ($all_ok) {
        wlog "INFO" "ALL_OK | $($summary -join ' ')"
    } else {
        wlog "WARN" "PARTIAL | $($summary -join ' ')"
    }

    Start-Sleep -Seconds $CHECK_SEC
}

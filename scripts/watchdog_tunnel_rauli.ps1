# ATLAS — Watchdog autonomo para tunel Cloudflare RAULI-VISION
# Monitorea servicios locales y el estado del tunel.
# Si detecta caida, reinicia cloudflared automaticamente.
#
# Uso rapido:
#   powershell.exe -NoProfile -WindowStyle Hidden -File "C:\ATLAS_PUSH\scripts\watchdog_tunnel_rauli.ps1"

param()

$CLOUDFLARED_EXE  = "C:/Program Files (x86)/cloudflared/cloudflared.exe"
$TUNNEL_CONFIG    = "C:/ATLAS_PUSH/config/cloudflared/cloudflare_atlas_config.yaml"
$LOG_FILE         = "C:/ATLAS_PUSH/logs/watchdog_tunnel.log"
$METRICS_URL      = "http://127.0.0.1:20241/metrics"
$LOCAL_HEALTH_URL = "http://127.0.0.1:3000/api/health"
$ESPEJO_URL       = "http://127.0.0.1:8080/api/health"

$CHECK_SEC        = 30
$FAIL_THRESHOLD   = 2
$COOLDOWN_SEC     = 120

function wlog {
    param([string]$lvl, [string]$msg)
    $ts   = Get-Date -Format "yyyy-MM-ddTHH:mm:ss"
    $line = "[$ts][$lvl] $msg"
    Write-Host $line
    try { Add-Content -Path $LOG_FILE -Value $line -Encoding UTF8 -ErrorAction Stop } catch {}
}

function http_ok {
    param([string]$url)
    try {
        $r = Invoke-WebRequest -Uri $url -TimeoutSec 5 -UseBasicParsing -ErrorAction Stop
        return ($r.StatusCode -lt 500)
    } catch {
        return $false
    }
}

function ha_connections {
    try {
        $m = Invoke-WebRequest -Uri $METRICS_URL -TimeoutSec 3 -UseBasicParsing -ErrorAction Stop
        if ($m.Content -match 'cloudflared_tunnel_ha_connections\s+(\d+)') {
            return [int]$Matches[1]
        }
    } catch {}
    return -1
}

function cf_running {
    $p = @(Get-Process -Name "cloudflared" -ErrorAction SilentlyContinue)
    return ($p.Count -gt 0)
}

function stop_cf {
    wlog "WARN" "Deteniendo cloudflared..."
    Get-Process -Name "cloudflared" -ErrorAction SilentlyContinue | ForEach-Object {
        Stop-Process -Id $_.Id -Force -ErrorAction SilentlyContinue
    }
    Start-Sleep -Seconds 3
}

function start_cf {
    wlog "INFO" "Iniciando tunel con $TUNNEL_CONFIG"
    Start-Process `
        -FilePath $CLOUDFLARED_EXE `
        -ArgumentList "tunnel","--config",$TUNNEL_CONFIG,"--loglevel","info","run" `
        -WindowStyle Hidden `
        -ErrorAction SilentlyContinue
    Start-Sleep -Seconds 10
    $ha = ha_connections
    wlog "INFO" "Post-restart HA=$ha"
    return ($ha -gt 0)
}

# --- Main ---
$fail_count  = 0
$last_restart = [datetime]::MinValue
$restarts     = 0

wlog "INFO" "=== Watchdog iniciado === intervalo:${CHECK_SEC}s umbral:$FAIL_THRESHOLD cooldown:${COOLDOWN_SEC}s"

while ($true) {
    $ok_local  = http_ok $LOCAL_HEALTH_URL
    $ok_espejo = http_ok $ESPEJO_URL
    $ha        = ha_connections
    $cf_up     = cf_running

    $problems = @()
    if (-not $ok_local)  { $problems += "3000-down" }
    if (-not $ok_espejo) { $problems += "8080-down" }
    if (-not $cf_up)     { $problems += "cf-not-running" }
    if ($ha -eq 0)       { $problems += "ha-0" }

    if ($problems.Count -eq 0) {
        if ($fail_count -gt 0) {
            wlog "INFO" "Servicio recuperado. Reset contador."
        }
        $fail_count = 0
        wlog "INFO" "OK | 3000=$ok_local 8080=$ok_espejo HA=$ha cf=$cf_up"
    } else {
        $fail_count++
        wlog "WARN" "Fallo #$fail_count | $($problems -join ',') HA=$ha"

        if ($fail_count -ge $FAIL_THRESHOLD) {
            $secs_since = ([datetime]::Now - $last_restart).TotalSeconds
            if ($secs_since -lt $COOLDOWN_SEC) {
                $wait = [int]($COOLDOWN_SEC - $secs_since)
                wlog "WARN" "Cooldown activo, espera ${wait}s"
            } else {
                $restarts++
                $last_restart = [datetime]::Now
                wlog "ERROR" "Reiniciando tunel #$restarts"
                if ((-not $cf_up) -or ($ha -eq 0)) {
                    stop_cf
                    start_cf | Out-Null
                }
                if ((-not $ok_local) -or (-not $ok_espejo)) {
                    wlog "ERROR" "Servicio local caido (3000=$ok_local 8080=$ok_espejo) - revisar manualmente"
                }
                $fail_count = 0
            }
        }
    }

    Start-Sleep -Seconds $CHECK_SEC
}

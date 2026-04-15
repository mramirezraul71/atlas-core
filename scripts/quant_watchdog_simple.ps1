<#
.SYNOPSIS
    Watchdog simple para Atlas Code-Quant (:8795).

.DESCRIPTION
    Verifica /health cada N segundos.
    Si falla X veces consecutivas, relanza el servicio usando atlas_quant_start.ps1.
#>

param(
    [int]$Port = 8795,
    [int]$IntervalSec = 15,
    [int]$FailureThreshold = 2,
    [int]$MaxWaitSec = 60,
    [int]$RestartCooldownSec = 45,
    [switch]$RunOnce
)

$ErrorActionPreference = "Stop"

$RepoRoot = "C:\ATLAS_PUSH"
$StartScript = Join-Path $RepoRoot "scripts\atlas_quant_start.ps1"
$HealthUrl = "http://127.0.0.1:$Port/health"
$LogDir = Join-Path $RepoRoot "logs"
$LogFile = Join-Path $LogDir "quant_watchdog.log"

if (-not (Test-Path $LogDir)) {
    New-Item -Path $LogDir -ItemType Directory -Force | Out-Null
}

function Write-Log([string]$level, [string]$message) {
    $ts = Get-Date -Format "yyyy-MM-dd HH:mm:ss"
    $line = "[$ts] [$level] $message"
    Add-Content -Path $LogFile -Value $line
    Write-Host $line
}

function Test-QuantHealth {
    try {
        $resp = Invoke-WebRequest -UseBasicParsing -Uri $HealthUrl -TimeoutSec 5 -ErrorAction Stop
        return ($resp.StatusCode -eq 200)
    } catch {
        return $false
    }
}

if (-not (Test-Path $StartScript)) {
    throw "No existe script de arranque: $StartScript"
}

$fails = 0
$lastRestart = [datetime]::MinValue

Write-Log "INFO" "Watchdog iniciado (port=$Port interval=${IntervalSec}s threshold=$FailureThreshold)"

do {
    $ok = Test-QuantHealth
    if ($ok) {
        if ($fails -gt 0) {
            Write-Log "INFO" "Health recuperado tras $fails fallos consecutivos."
        }
        $fails = 0
    } else {
        $fails += 1
        Write-Log "WARN" "Health check falló ($fails/$FailureThreshold) en $HealthUrl"

        if ($fails -ge $FailureThreshold) {
            $sinceRestart = (Get-Date) - $lastRestart
            if ($sinceRestart.TotalSeconds -lt $RestartCooldownSec) {
                Write-Log "WARN" "Cooldown activo (${RestartCooldownSec}s). No se reinicia todavía."
            } else {
                Write-Log "ERROR" "Umbral alcanzado. Reiniciando servicio Quant..."
                try {
                    & $StartScript -Port $Port -MaxWaitSec $MaxWaitSec | Out-Null
                    $lastRestart = Get-Date
                    $fails = 0
                    Write-Log "INFO" "Restart solicitado correctamente."
                } catch {
                    Write-Log "ERROR" "Falló restart: $($_.Exception.Message)"
                }
            }
        }
    }

    if (-not $RunOnce) {
        Start-Sleep -Seconds $IntervalSec
    }
} while (-not $RunOnce)

Write-Log "INFO" "Watchdog finalizado."

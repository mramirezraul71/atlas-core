param(
    [string]$RepoPath = "C:\ATLAS_PUSH",
    [string]$PushBaseUrl = "http://127.0.0.1:8791",
    [string]$PanaderiaUrl = "https://panaderia.rauliatlasapp.com",
    [string]$PanaderiaApiUrl = "https://panaderia-api.rauliatlasapp.com/api/health",
    [string]$VisionHealthUrl = "https://vision.rauliatlasapp.com/api/health",
    [string]$VisionTikTokProbeUrl = "https://vision.rauliatlasapp.com/api/search?q=tiktok.com&max=1",
    [int]$RefreshSeconds = 10,
    [int]$TimeoutSec = 12,
    [int]$WarnLatencyMs = 2500,
    [int]$CriticalLatencyMs = 7000,
    [int]$HealCooldownSec = 180,
    [switch]$AutoHealOnCritical,
    [switch]$ClearScreen,
    [switch]$Once
)

$ErrorActionPreference = "Stop"

$logDir = Join-Path $RepoPath "logs"
$logFile = Join-Path $logDir "atlas_unified_monitor.jsonl"
$healScript = Join-Path $RepoPath "scripts\panaderia_tunnel_autoheal.ps1"
$lastHealUtc = [DateTime]::MinValue

New-Item -ItemType Directory -Path $logDir -Force | Out-Null

function Safe-JsonCall {
    param(
        [Parameter(Mandatory = $true)][string]$Url,
        [int]$Timeout = 10
    )
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    try {
        $data = Invoke-RestMethod -Method Get -Uri $Url -TimeoutSec $Timeout -ErrorAction Stop
        $sw.Stop()
        return [ordered]@{
            ok = $true
            ms = [int]$sw.ElapsedMilliseconds
            data = $data
            error = $null
            url = $Url
        }
    } catch {
        $sw.Stop()
        return [ordered]@{
            ok = $false
            ms = [int]$sw.ElapsedMilliseconds
            data = $null
            error = $_.Exception.Message
            url = $Url
        }
    }
}

function Safe-HttpCall {
    param(
        [Parameter(Mandatory = $true)][string]$Url,
        [int]$Timeout = 10
    )
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $statusCode = $null
    try {
        $resp = Invoke-WebRequest -Method Get -Uri $Url -UseBasicParsing -TimeoutSec $Timeout -ErrorAction Stop
        $sw.Stop()
        $statusCode = [int]$resp.StatusCode
        return [ordered]@{
            ok = ($statusCode -ge 200 -and $statusCode -lt 300)
            code = $statusCode
            ms = [int]$sw.ElapsedMilliseconds
            error = $null
            url = $Url
        }
    } catch {
        $sw.Stop()
        try {
            if ($_.Exception.Response -and $_.Exception.Response.StatusCode) {
                $statusCode = [int]$_.Exception.Response.StatusCode
            }
        } catch {}
        return [ordered]@{
            ok = $false
            code = $statusCode
            ms = [int]$sw.ElapsedMilliseconds
            error = $_.Exception.Message
            url = $Url
        }
    }
}

function Semaforo {
    param(
        [bool]$Ok,
        [int]$LatencyMs,
        [int]$WarnMs,
        [int]$CriticalMs
    )
    if (-not $Ok) { return "RED" }
    if ($LatencyMs -ge $CriticalMs) { return "RED" }
    if ($LatencyMs -ge $WarnMs) { return "YELLOW" }
    return "GREEN"
}

function Color-ByState {
    param([string]$State)
    $s = ""
    if ($null -ne $State) {
        $s = [string]$State
    }
    switch ($s.ToUpperInvariant()) {
        "GREEN" { return "Green" }
        "YELLOW" { return "Yellow" }
        default { return "Red" }
    }
}

function Get-PendingSupervisorCount {
    param([string]$BasePush)
    $res = Safe-JsonCall -Url "$($BasePush.TrimEnd('/'))/supervisor/directives?status=pending&limit=200" -Timeout $TimeoutSec
    if (-not $res.ok) { return [ordered]@{ ok = $false; count = -1; ms = $res.ms; error = $res.error } }
    $items = @()
    if ($res.data -and $res.data.data) {
        $items = @($res.data.data)
    }
    return [ordered]@{
        ok = $true
        count = $items.Count
        ms = $res.ms
        error = $null
    }
}

function Maybe-AutoHeal {
    param(
        [bool]$Critical,
        [string]$HealScriptPath
    )
    if (-not $AutoHealOnCritical.IsPresent) {
        return [ordered]@{ attempted = $false; ok = $true; reason = "disabled" }
    }
    if (-not $Critical) {
        return [ordered]@{ attempted = $false; ok = $true; reason = "not_critical" }
    }
    if (-not (Test-Path $HealScriptPath)) {
        return [ordered]@{ attempted = $false; ok = $false; reason = "heal_script_missing" }
    }
    $now = (Get-Date).ToUniversalTime()
    $elapsed = ($now - $lastHealUtc).TotalSeconds
    if ($lastHealUtc -ne [DateTime]::MinValue -and $elapsed -lt $HealCooldownSec) {
        return [ordered]@{
            attempted = $false
            ok = $true
            reason = "cooldown"
            cooldown_remaining_s = [int]([Math]::Ceiling($HealCooldownSec - $elapsed))
        }
    }
    try {
        & powershell -NoProfile -ExecutionPolicy Bypass -File $HealScriptPath -ForceHeal -PushFailThreshold 1 -PushForceRestartThreshold 1 | Out-Null
        $script:lastHealUtc = $now
        return [ordered]@{ attempted = $true; ok = $true; reason = "heal_invoked" }
    } catch {
        return [ordered]@{
            attempted = $true
            ok = $false
            reason = "heal_failed"
            error = $_.Exception.Message
        }
    }
}

function Log-JsonLine {
    param([hashtable]$Payload)
    ($Payload | ConvertTo-Json -Depth 8 -Compress) + "`n" | Out-File -FilePath $logFile -Append -Encoding UTF8
}

$pushBase = $PushBaseUrl.TrimEnd("/")

Write-Host "ATLAS Unified Monitor" -ForegroundColor Cyan
Write-Host "Push: $pushBase" -ForegroundColor DarkCyan
Write-Host "Panaderia: $PanaderiaUrl" -ForegroundColor DarkCyan
Write-Host "Vision: $VisionHealthUrl" -ForegroundColor DarkCyan
Write-Host "AutoHeal: $($AutoHealOnCritical.IsPresent)" -ForegroundColor DarkCyan
Write-Host "Log: $logFile" -ForegroundColor DarkGray
Write-Host ""

while ($true) {
    if ($ClearScreen) {
        Clear-Host
        Write-Host "ATLAS Unified Monitor" -ForegroundColor Cyan
        Write-Host "Push: $pushBase | AutoHeal: $($AutoHealOnCritical.IsPresent)" -ForegroundColor DarkCyan
        Write-Host ""
    }

    $nowLocal = Get-Date
    $tsLocal = $nowLocal.ToString("yyyy-MM-dd HH:mm:ss")
    $tsUtc = $nowLocal.ToUniversalTime().ToString("o")

    $push = Safe-JsonCall -Url "$pushBase/health" -Timeout $TimeoutSec
    $comms = Safe-JsonCall -Url "$pushBase/api/comms/atlas/status" -Timeout $TimeoutSec
    $wa = Safe-JsonCall -Url "$pushBase/api/comms/whatsapp/status" -Timeout $TimeoutSec
    $sup = Get-PendingSupervisorCount -BasePush $pushBase
    $panFront = Safe-HttpCall -Url $PanaderiaUrl -Timeout $TimeoutSec
    $panApi = Safe-HttpCall -Url $PanaderiaApiUrl -Timeout $TimeoutSec
    $vision = Safe-HttpCall -Url $VisionHealthUrl -Timeout $TimeoutSec
    $visionProbe = Safe-JsonCall -Url $VisionTikTokProbeUrl -Timeout $TimeoutSec

    $pushState = Semaforo -Ok ([bool]$push.ok) -LatencyMs ([int]$push.ms) -WarnMs $WarnLatencyMs -CriticalMs $CriticalLatencyMs
    $panState = Semaforo -Ok ([bool]($panFront.ok -and $panApi.ok)) -LatencyMs ([int]([Math]::Max($panFront.ms, $panApi.ms))) -WarnMs $WarnLatencyMs -CriticalMs $CriticalLatencyMs
    $visionState = Semaforo -Ok ([bool]$vision.ok) -LatencyMs ([int]$vision.ms) -WarnMs $WarnLatencyMs -CriticalMs $CriticalLatencyMs

    $waReady = $false
    $waAuth = $false
    if ($wa.ok -and $wa.data) {
        $waReady = [bool]$wa.data.ready
        $waAuth = [bool]$wa.data.provider_status.authenticated
    }
    $waState = if ($wa.ok -and $waReady -and $waAuth) { "GREEN" } elseif ($wa.ok) { "YELLOW" } else { "RED" }

    $probeOk = $false
    if ($visionProbe.ok -and $visionProbe.data -and $visionProbe.data.results) {
        $results = @($visionProbe.data.results)
        if ($results.Count -gt 0) {
            $u = [string]($results[0].url)
            $probeOk = $u -like "https://r.jina.ai/http://tiktok.com*"
        }
    }
    $probeState = if ($probeOk) { "GREEN" } else { "YELLOW" }

    $queuePending = -1
    if ($comms.ok -and $comms.data -and $null -ne $comms.data.queue_pending) {
        $queuePending = [int]$comms.data.queue_pending
    }
    $critical = @($pushState, $panState, $visionState, $waState) -contains "RED"
    $heal = Maybe-AutoHeal -Critical $critical -HealScriptPath $healScript

    $line = [ordered]@{
        ts_local = $tsLocal
        ts_utc = $tsUtc
        semaforo = @{
            push = $pushState
            panaderia = $panState
            vision = $visionState
            whatsapp = $waState
            tiktok_probe = $probeState
        }
        latency_ms = @{
            push = [int]$push.ms
            pan_front = [int]$panFront.ms
            pan_api = [int]$panApi.ms
            vision = [int]$vision.ms
            whatsapp = [int]$wa.ms
            comms = [int]$comms.ms
        }
        indicators = @{
            queue_pending = $queuePending
            supervisor_pending = [int]$sup.count
            wa_ready = $waReady
            wa_auth = $waAuth
            tiktok_probe_ok = $probeOk
        }
        heal = $heal
        errors = @{
            push = $push.error
            panaderia = if ($panFront.error) { $panFront.error } else { $panApi.error }
            vision = $vision.error
            whatsapp = $wa.error
            comms = $comms.error
            supervisor = $sup.error
            tiktok_probe = $visionProbe.error
        }
    }
    Log-JsonLine -Payload $line

    Write-Host "[$tsLocal] ESTADO GLOBAL" -ForegroundColor White
    Write-Host ("  PUSH      : {0} ({1} ms)" -f $pushState, [int]$push.ms) -ForegroundColor (Color-ByState $pushState)
    Write-Host ("  PANADERIA : {0} (front {1} ms / api {2} ms)" -f $panState, [int]$panFront.ms, [int]$panApi.ms) -ForegroundColor (Color-ByState $panState)
    Write-Host ("  VISION    : {0} ({1} ms)" -f $visionState, [int]$vision.ms) -ForegroundColor (Color-ByState $visionState)
    Write-Host ("  WHATSAPP  : {0} (ready={1} auth={2})" -f $waState, $waReady, $waAuth) -ForegroundColor (Color-ByState $waState)
    Write-Host ("  TIKTOK/CU : {0}" -f $probeState) -ForegroundColor (Color-ByState $probeState)
    Write-Host ("  Cola/ Sup : queue={0} supervisor_pending={1}" -f $queuePending, [int]$sup.count) -ForegroundColor DarkGray
    if ($heal.attempted) {
        if ($heal.ok) {
            Write-Host "  AutoHeal  : ejecutado" -ForegroundColor Yellow
        } else {
            Write-Host ("  AutoHeal  : fallo ({0})" -f ([string]$heal.reason)) -ForegroundColor Red
        }
    } elseif ($heal.reason -eq "cooldown") {
        Write-Host ("  AutoHeal  : cooldown {0}s" -f [int]$heal.cooldown_remaining_s) -ForegroundColor DarkYellow
    }
    Write-Host ""

    if ($Once) { break }
    Start-Sleep -Seconds $RefreshSeconds
}

param(
    [string]$RepoPath = "C:\ATLAS_PUSH",
    [string]$PushBaseUrl = "http://127.0.0.1:8791",
    [string]$TunnelHealthUrl = "",
    [int]$TimeoutSec = 8,
    [int]$AlertLookbackLines = 500,
    [int]$MissingAlertThreshold = 2,
    [switch]$CheckOnly,
    [switch]$ForceRestartComms
)

$ErrorActionPreference = "Stop"

$logDir = Join-Path $RepoPath "logs"
$jsonLog = Join-Path $logDir "cloudflare_comms_autocorrect.jsonl"
$opsBusLog = Join-Path $logDir "ops_bus.log"
$lockPath = Join-Path $logDir "cloudflare_comms_autocorrect.lock"
$script:LockHandle = $null

New-Item -ItemType Directory -Path $logDir -Force | Out-Null

function Acquire-Lock {
    try {
        $script:LockHandle = [System.IO.File]::Open(
            $lockPath,
            [System.IO.FileMode]::OpenOrCreate,
            [System.IO.FileAccess]::ReadWrite,
            [System.IO.FileShare]::None
        )
        return $true
    } catch {
        return $false
    }
}

function Release-Lock {
    try {
        if ($script:LockHandle) {
            $script:LockHandle.Close()
            $script:LockHandle.Dispose()
            $script:LockHandle = $null
        }
    } catch {}
}

function Invoke-HttpGet {
    param(
        [string]$Url,
        [int]$Timeout = 8
    )
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $ok = $false
    $statusCode = $null
    $errorText = $null
    $body = ""
    try {
        $resp = Invoke-WebRequest -Uri $Url -UseBasicParsing -TimeoutSec $Timeout -Method Get -ErrorAction Stop
        $statusCode = [int]$resp.StatusCode
        $body = "" + $resp.Content
        $ok = ($statusCode -ge 200 -and $statusCode -lt 300)
    } catch {
        $errorText = $_.Exception.Message
        try {
            if ($_.Exception.Response -and $_.Exception.Response.StatusCode) {
                $statusCode = [int]$_.Exception.Response.StatusCode
            }
        } catch {}
    }
    $sw.Stop()
    return [ordered]@{
        url = $Url
        ok = [bool]$ok
        code = $statusCode
        ms = [int]$sw.ElapsedMilliseconds
        error = $errorText
        body = $body
    }
}

function Invoke-JsonGet {
    param(
        [string]$Url,
        [int]$Timeout = 8
    )
    $probe = Invoke-HttpGet -Url $Url -Timeout $Timeout
    $data = $null
    if ($probe.ok -and $probe.body) {
        try {
            $data = $probe.body | ConvertFrom-Json -ErrorAction Stop
        } catch {}
    }
    $jsonOk = $false
    if ($data -and $null -ne $data.ok) {
        $jsonOk = [bool]$data.ok
    } else {
        $jsonOk = [bool]$probe.ok
    }
    return [ordered]@{
        url = $probe.url
        transport_ok = [bool]$probe.ok
        ok = [bool]$jsonOk
        code = $probe.code
        ms = $probe.ms
        error = $probe.error
        data = $data
    }
}

function Invoke-JsonGetFallback {
    param(
        [string[]]$Urls,
        [int]$Timeout = 8
    )
    $attempts = New-Object System.Collections.Generic.List[object]
    foreach ($url in $Urls) {
        if ([string]::IsNullOrWhiteSpace($url)) {
            continue
        }
        $r = Invoke-JsonGet -Url $url -Timeout $Timeout
        $attempts.Add($r) | Out-Null
        if ($r.transport_ok) {
            return [ordered]@{
                ok = [bool]$r.ok
                available = $true
                selected_url = $url
                probe = $r
                attempts = @($attempts.ToArray())
            }
        }
        if ($null -ne $r.code -and [int]$r.code -ne 404) {
            return [ordered]@{
                ok = [bool]$r.ok
                available = $true
                selected_url = $url
                probe = $r
                attempts = @($attempts.ToArray())
            }
        }
    }
    return [ordered]@{
        ok = $true
        available = $false
        selected_url = $null
        probe = $null
        attempts = @($attempts.ToArray())
        skipped = "endpoint_not_available"
    }
}

function Resolve-TunnelHealthUrl {
    param([string]$InputUrl)
    $candidate = ("" + $InputUrl).Trim()
    if (-not $candidate) {
        $candidate = ("" + $env:ATLAS_CLOUDFLARE_TUNNEL_HEALTH_URL).Trim()
    }
    if (-not $candidate) {
        $candidate = "http://127.0.0.1:20241/ready"
    }
    return $candidate
}

function Get-CloudflaredProcesses {
    try {
        $procs = @(Get-CimInstance Win32_Process | Where-Object {
            $_.Name -match "^cloudflared(\.exe)?$"
        })
        return [ordered]@{
            running = [bool]($procs.Count -gt 0)
            count = [int]$procs.Count
            pids = @($procs | ForEach-Object { $_.ProcessId })
        }
    } catch {
        return [ordered]@{
            running = $false
            count = 0
            pids = @()
            error = $_.Exception.Message
        }
    }
}

function Test-TunnelHealth {
    param(
        [string]$HealthUrl,
        [int]$Timeout = 8
    )
    $main = Invoke-HttpGet -Url $HealthUrl -Timeout $Timeout
    if ($main.ok) {
        return [ordered]@{
            ok = $true
            via = "health_url"
            probe = $main
        }
    }

    $metricsUrl = "http://127.0.0.1:20241/metrics"
    $metrics = Invoke-HttpGet -Url $metricsUrl -Timeout ([Math]::Min($Timeout, 5))
    $haConnections = -1
    if ($metrics.ok -and $metrics.body -match 'cloudflared_tunnel_ha_connections\s+(\d+)') {
        $haConnections = [int]$Matches[1]
    }
    $okMetrics = [bool]($metrics.ok -and $haConnections -gt 0)
    return [ordered]@{
        ok = $okMetrics
        via = if ($okMetrics) { "metrics_ha_connections" } else { "unhealthy" }
        probe = $main
        metrics = [ordered]@{
            url = $metricsUrl
            ok = $metrics.ok
            code = $metrics.code
            error = $metrics.error
            ha_connections = $haConnections
        }
    }
}

function Get-AlertNoiseStats {
    param(
        [string]$OpsLogPath,
        [int]$Lookback = 500
    )
    if (-not (Test-Path $OpsLogPath)) {
        return [ordered]@{
            found = $false
            lines_scanned = 0
            missing_url_count = 0
            cloudflare_failed_count = 0
        }
    }
    $lines = @(Get-Content -Path $OpsLogPath -Tail $Lookback -ErrorAction SilentlyContinue)
    $missingCount = 0
    $failedCount = 0
    foreach ($line in $lines) {
        if ($line -match "no ATLAS_CLOUDFLARE_ALERT_URL configured") {
            $missingCount++
        }
        if ($line -match "cloudflare alert failed") {
            $failedCount++
        }
    }
    return [ordered]@{
        found = $true
        lines_scanned = [int]$lines.Count
        missing_url_count = [int]$missingCount
        cloudflare_failed_count = [int]$failedCount
    }
}

function Restart-NamedTunnel {
    param([string]$Repo)
    $scriptPath = Join-Path $Repo "scripts\start_cloudflare_named_tunnel.ps1"
    if (-not (Test-Path $scriptPath)) {
        return [ordered]@{
            attempted = $false
            ok = $false
            error = "start_cloudflare_named_tunnel.ps1_not_found"
        }
    }
    try {
        & powershell -NoProfile -ExecutionPolicy Bypass -WindowStyle Hidden -File $scriptPath -StopQuickTunnels | Out-Null
        return [ordered]@{
            attempted = $true
            ok = $true
            error = $null
        }
    } catch {
        return [ordered]@{
            attempted = $true
            ok = $false
            error = $_.Exception.Message
        }
    }
}

function Restart-CommsService {
    param(
        [string]$BaseUrl,
        [string]$Service,
        [int]$Timeout = 10
    )
    $uri = ($BaseUrl.TrimEnd("/") + "/api/comms/bootstrap/restart-service")
    try {
        $payload = @{ service = $Service } | ConvertTo-Json -Compress
        $resp = Invoke-RestMethod -Method Post -Uri $uri -ContentType "application/json" -Body $payload -TimeoutSec $Timeout
        return [ordered]@{
            attempted = $true
            ok = [bool]$resp.ok
            service = $Service
            response = $resp
        }
    } catch {
        return [ordered]@{
            attempted = $true
            ok = $false
            service = $Service
            error = $_.Exception.Message
        }
    }
}

function Reset-TelegramChannel {
    param(
        [string]$BaseUrl,
        [int]$Timeout = 8
    )
    $uri = ($BaseUrl.TrimEnd("/") + "/api/comms/hub/reset-channel")
    try {
        $payload = @{ channel = "telegram" } | ConvertTo-Json -Compress
        $resp = Invoke-RestMethod -Method Post -Uri $uri -ContentType "application/json" -Body $payload -TimeoutSec $Timeout
        return [ordered]@{
            attempted = $true
            ok = [bool]$resp.ok
            response = $resp
        }
    } catch {
        return [ordered]@{
            attempted = $true
            ok = $false
            error = $_.Exception.Message
        }
    }
}

if (-not (Acquire-Lock)) {
    Write-Output "{`"ok`":true,`"skipped`":`"lock_busy`"}"
    exit 0
}

try {
    $tsUtc = (Get-Date).ToUniversalTime().ToString("o")
    $actions = New-Object System.Collections.Generic.List[object]
    $recommendations = New-Object System.Collections.Generic.List[string]

    $resolvedTunnelHealth = Resolve-TunnelHealthUrl -InputUrl $TunnelHealthUrl

    $checks = [ordered]@{}
    $checks.push_health = Invoke-JsonGetFallback -Urls @(
        ($PushBaseUrl.TrimEnd("/") + "/health"),
        ($PushBaseUrl.TrimEnd("/") + "/api/health"),
        ($PushBaseUrl.TrimEnd("/") + "/status")
    ) -Timeout $TimeoutSec
    $checks.comms_bootstrap_health = Invoke-JsonGetFallback -Urls @(
        ($PushBaseUrl.TrimEnd("/") + "/api/comms/bootstrap/health"),
        ($PushBaseUrl.TrimEnd("/") + "/comms/bootstrap/health")
    ) -Timeout $TimeoutSec
    $checks.cloudflared = Get-CloudflaredProcesses
    $checks.tunnel = Test-TunnelHealth -HealthUrl $resolvedTunnelHealth -Timeout $TimeoutSec
    $checks.alert_noise = Get-AlertNoiseStats -OpsLogPath $opsBusLog -Lookback $AlertLookbackLines

    $alertUrlConfigured = [bool]((("" + $env:ATLAS_CLOUDFLARE_ALERT_URL).Trim()).Length -gt 0)
    if (-not $alertUrlConfigured -and [int]$checks.alert_noise.missing_url_count -ge $MissingAlertThreshold) {
        $recommendations.Add(
            "Configurar ATLAS_CLOUDFLARE_ALERT_URL para eliminar fallback critico por URL faltante."
        ) | Out-Null
    }

    if (-not $CheckOnly.IsPresent) {
        $tunnelNeedsHeal = [bool]((-not $checks.cloudflared.running) -or (-not $checks.tunnel.ok))
        if ($tunnelNeedsHeal) {
            $healTunnel = Restart-NamedTunnel -Repo $RepoPath
            $actions.Add([ordered]@{
                kind = "restart_named_tunnel"
                result = $healTunnel
            }) | Out-Null
            Start-Sleep -Seconds 2
            $checks.cloudflared_post = Get-CloudflaredProcesses
            $checks.tunnel_post = Test-TunnelHealth -HealthUrl $resolvedTunnelHealth -Timeout $TimeoutSec
        }

        $needCommsHeal = [bool](
            $checks.comms_bootstrap_health.available -and
            (
                $ForceRestartComms.IsPresent -or
                (-not $checks.comms_bootstrap_health.ok)
            )
        )
        if ($needCommsHeal) {
            foreach ($service in @("hub", "telegram_poller")) {
                $result = Restart-CommsService -BaseUrl $PushBaseUrl -Service $service -Timeout $TimeoutSec
                $actions.Add([ordered]@{
                    kind = "restart_comms_service"
                    service = $service
                    result = $result
                }) | Out-Null
            }
            $resetTelegram = Reset-TelegramChannel -BaseUrl $PushBaseUrl -Timeout $TimeoutSec
            $actions.Add([ordered]@{
                kind = "reset_telegram_channel"
                result = $resetTelegram
            }) | Out-Null

            Start-Sleep -Seconds 1
            $checks.comms_bootstrap_health_post = Invoke-JsonGetFallback -Urls @(
                ($PushBaseUrl.TrimEnd("/") + "/api/comms/bootstrap/health"),
                ($PushBaseUrl.TrimEnd("/") + "/comms/bootstrap/health")
            ) -Timeout $TimeoutSec
        } elseif (-not $checks.comms_bootstrap_health.available) {
            $recommendations.Add(
                "El endpoint de Comms bootstrap no esta disponible en :8791; se omite autorestart de hub/telegram_poller."
            ) | Out-Null
        }
    }

    $finalTunnelOk = if ($checks.tunnel_post) { [bool]$checks.tunnel_post.ok } else { [bool]$checks.tunnel.ok }
    $finalCommsOk = $true
    if ($checks.comms_bootstrap_health.available) {
        $finalCommsOk = if ($checks.comms_bootstrap_health_post) {
            [bool]$checks.comms_bootstrap_health_post.ok
        } else {
            [bool]$checks.comms_bootstrap_health.ok
        }
    }
    $overallOk = [bool]($finalTunnelOk -and $finalCommsOk)

    $actionItems = @()
    if ($actions.Count -gt 0) {
        $actionItems = @($actions.ToArray())
    }
    $recommendationItems = @()
    if ($recommendations.Count -gt 0) {
        $recommendationItems = @($recommendations.ToArray())
    }

    $runResult = [ordered]@{
        ok = $overallOk
        timestamp_utc = $tsUtc
        check_only = [bool]$CheckOnly.IsPresent
        applied_healing = [bool](-not $CheckOnly.IsPresent)
        checks = $checks
        actions = $actionItems
        recommendations = $recommendationItems
    }

    ($runResult | ConvertTo-Json -Depth 8 -Compress) | Add-Content -Path $jsonLog -Encoding UTF8
    $runResult | ConvertTo-Json -Depth 8
} finally {
    Release-Lock
}

param(
    [string]$RepoPath = "C:\ATLAS_PUSH",
    [string]$PanaderiaUrl = "https://panaderia.rauliatlasapp.com",
    [string]$PanaderiaApiUrl = "https://panaderia-api.rauliatlasapp.com/api/health",
    [string]$LocalFrontendUrl = "http://127.0.0.1:5173",
    [string]$LocalApiUrl = "http://127.0.0.1:3001/api/health",
    [string]$PushHealthUrl = "http://127.0.0.1:8791/health",
    [string]$ArmsStartUrl = "http://127.0.0.1:8791/arms/panaderia/start",
    [int]$TimeoutSec = 10,
    [int]$FailThreshold = 2,
    [int]$HealCooldownSec = 120,
    [switch]$ForceHeal
)

$ErrorActionPreference = "Stop"

$logDir = Join-Path $RepoPath "logs"
$logFile = Join-Path $logDir "panaderia_tunnel_autoheal.jsonl"
$stateFile = Join-Path $logDir "panaderia_tunnel_autoheal.state.json"
$lockFile = Join-Path $logDir "panaderia_tunnel_autoheal.lock"
$restartPushScript = Join-Path $RepoPath "scripts\restart_push_from_api.ps1"
$startTunnelScript = Join-Path $RepoPath "scripts\start_cloudflare_named_tunnel.ps1"
$script:LockHandle = $null

New-Item -ItemType Directory -Path $logDir -Force | Out-Null

function Acquire-Lock {
    try {
        $script:LockHandle = [System.IO.File]::Open(
            $lockFile,
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

function Read-State {
    if (-not (Test-Path $stateFile)) {
        return @{
            public_fail_streak = 0
            last_heal_utc = ""
        }
    }
    try {
        $raw = Get-Content -Path $stateFile -Raw -Encoding UTF8
        $obj = $raw | ConvertFrom-Json
        return @{
            public_fail_streak = [int]($obj.public_fail_streak)
            last_heal_utc = [string]($obj.last_heal_utc)
        }
    } catch {
        return @{
            public_fail_streak = 0
            last_heal_utc = ""
        }
    }
}

function Write-State {
    param([hashtable]$State)
    $payload = @{
        public_fail_streak = [int]($State.public_fail_streak)
        last_heal_utc = [string]($State.last_heal_utc)
        updated_utc = (Get-Date).ToUniversalTime().ToString("o")
    }
    ($payload | ConvertTo-Json -Depth 4) | Set-Content -Path $stateFile -Encoding UTF8
}

function Test-Http {
    param(
        [string]$Url,
        [int]$Timeout = 10
    )
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $code = $null
    $ok = $false
    $err = $null
    try {
        $resp = Invoke-WebRequest -Uri $Url -UseBasicParsing -TimeoutSec $Timeout -Method Get -ErrorAction Stop
        $code = [int]$resp.StatusCode
        $ok = ($code -ge 200 -and $code -lt 300)
    } catch {
        $err = $_.Exception.Message
        try {
            if ($_.Exception.Response -and $_.Exception.Response.StatusCode) {
                $code = [int]$_.Exception.Response.StatusCode
            }
        } catch {}
    }
    $sw.Stop()
    return [ordered]@{
        url = $Url
        ok = $ok
        code = $code
        ms = [int]$sw.ElapsedMilliseconds
        error = $err
    }
}

function Invoke-ArmsStart {
    param([string]$Url)
    try {
        $resp = Invoke-RestMethod -Method Post -Uri $Url -TimeoutSec 20
        return @{
            ok = $true
            data = $resp
            error = $null
        }
    } catch {
        return @{
            ok = $false
            data = $null
            error = $_.Exception.Message
        }
    }
}

function Restart-PushIfNeeded {
    param($PushCheck)
    if ($PushCheck.ok) {
        return @{
            attempted = $false
            ok = $true
            error = $null
        }
    }
    if (-not (Test-Path $restartPushScript)) {
        return @{
            attempted = $false
            ok = $false
            error = "restart_push_script_missing"
        }
    }
    try {
        & powershell -NoProfile -ExecutionPolicy Bypass -File $restartPushScript | Out-Null
        $recheck = Test-Http -Url $PushHealthUrl -Timeout 20
        return @{
            attempted = $true
            ok = [bool]$recheck.ok
            error = if ($recheck.ok) { $null } else { "push_health_still_down" }
        }
    } catch {
        return @{
            attempted = $true
            ok = $false
            error = $_.Exception.Message
        }
    }
}

function Restart-Tunnel {
    if (-not (Test-Path $startTunnelScript)) {
        return @{
            ok = $false
            error = "start_tunnel_script_missing"
        }
    }
    try {
        $procs = Get-CimInstance Win32_Process | Where-Object { $_.Name -match "cloudflared" }
        foreach ($p in $procs) {
            try { Stop-Process -Id $p.ProcessId -Force -ErrorAction Stop } catch {}
        }
        Start-Sleep -Seconds 1
        & powershell -NoProfile -ExecutionPolicy Bypass -File $startTunnelScript -StopQuickTunnels -ConfigureAutoStart | Out-Null
        Start-Sleep -Seconds 2
        return @{
            ok = $true
            error = $null
        }
    } catch {
        return @{
            ok = $false
            error = $_.Exception.Message
        }
    }
}

function Seconds-SinceUtc {
    param([string]$UtcIso)
    if (-not $UtcIso) { return 999999 }
    try {
        $last = [DateTime]::Parse($UtcIso).ToUniversalTime()
        $now = (Get-Date).ToUniversalTime()
        return [int]($now - $last).TotalSeconds
    } catch {
        return 999999
    }
}

if (-not (Acquire-Lock)) {
    exit 0
}

try {
    $state = Read-State
    $tsUtc = (Get-Date).ToUniversalTime().ToString("o")
    $actions = New-Object System.Collections.Generic.List[object]

    $push = Test-Http -Url $PushHealthUrl -Timeout $TimeoutSec
    $localFront = Test-Http -Url $LocalFrontendUrl -Timeout $TimeoutSec
    $localApi = Test-Http -Url $LocalApiUrl -Timeout $TimeoutSec
    $publicFront = Test-Http -Url $PanaderiaUrl -Timeout $TimeoutSec
    $publicApi = Test-Http -Url $PanaderiaApiUrl -Timeout $TimeoutSec

    $localReady = [bool]($localFront.ok -and $localApi.ok)
    $publicReady = [bool]($publicFront.ok -and $publicApi.ok)

    if ($publicReady) {
        $state.public_fail_streak = 0
    } else {
        $state.public_fail_streak = [int]$state.public_fail_streak + 1
    }

    if (-not $push.ok) {
        $pushFix = Restart-PushIfNeeded -PushCheck $push
        $actions.Add(@{
            step = "restart_push_if_needed"
            attempted = $pushFix.attempted
            ok = $pushFix.ok
            error = $pushFix.error
        })
        $push = Test-Http -Url $PushHealthUrl -Timeout $TimeoutSec
    }

    if (-not $localReady -and $push.ok) {
        $armStart = Invoke-ArmsStart -Url $ArmsStartUrl
        $actions.Add(@{
            step = "start_panaderia_arm"
            attempted = $true
            ok = $armStart.ok
            error = $armStart.error
        })
        Start-Sleep -Seconds 2
        $localFront = Test-Http -Url $LocalFrontendUrl -Timeout $TimeoutSec
        $localApi = Test-Http -Url $LocalApiUrl -Timeout $TimeoutSec
        $localReady = [bool]($localFront.ok -and $localApi.ok)
    }

    $needsHeal = [bool](-not $publicReady)
    if ($needsHeal) {
        $streakReached = [bool]($state.public_fail_streak -ge $FailThreshold)
        $cooldownSec = Seconds-SinceUtc -UtcIso $state.last_heal_utc
        $cooldownReady = [bool]($cooldownSec -ge $HealCooldownSec)
        $canHeal = [bool]($ForceHeal.IsPresent -or ($streakReached -and $cooldownReady))

        if ($canHeal) {
            $tunnelRestart = Restart-Tunnel
            $state.last_heal_utc = (Get-Date).ToUniversalTime().ToString("o")
            $actions.Add(@{
                step = "restart_cloudflared_tunnel"
                attempted = $true
                ok = $tunnelRestart.ok
                error = $tunnelRestart.error
                reason = if ($ForceHeal.IsPresent) { "force_heal" } else { "public_fail_streak=$($state.public_fail_streak)" }
            })

            Start-Sleep -Seconds 3
            $publicFront = Test-Http -Url $PanaderiaUrl -Timeout $TimeoutSec
            $publicApi = Test-Http -Url $PanaderiaApiUrl -Timeout $TimeoutSec
            $publicReady = [bool]($publicFront.ok -and $publicApi.ok)
            if ($publicReady) {
                $state.public_fail_streak = 0
            }
        } else {
            $actions.Add(@{
                step = "heal_skipped"
                attempted = $false
                ok = $true
                reason = "cooldown_or_threshold"
                fail_streak = $state.public_fail_streak
            })
        }
    }

    Write-State -State $state

    $payload = @{
        ts_utc = $tsUtc
        push = $push
        local = @{
            frontend = $localFront
            api = $localApi
            ready = $localReady
        }
        public = @{
            frontend = $publicFront
            api = $publicApi
            ready = $publicReady
        }
        state = @{
            public_fail_streak = [int]$state.public_fail_streak
            last_heal_utc = [string]$state.last_heal_utc
        }
        actions = @($actions.ToArray())
    }

    ($payload | ConvertTo-Json -Depth 12 -Compress) | Add-Content -Path $logFile -Encoding UTF8

    if ($publicReady) {
        Write-Output "AUTOHEAL status=ok public_ready=true local_ready=$localReady streak=$($state.public_fail_streak)"
    } else {
        Write-Output "AUTOHEAL status=degraded public_ready=false local_ready=$localReady streak=$($state.public_fail_streak)"
    }
} catch {
    $lineNo = $null
    $lineTxt = $null
    try {
        $lineNo = $_.InvocationInfo.ScriptLineNumber
        $lineTxt = $_.InvocationInfo.Line
    } catch {}
    $errPayload = [ordered]@{
        ts_utc = (Get-Date).ToUniversalTime().ToString("o")
        fatal_error = $_.Exception.Message
        line = $lineNo
        script_line = $lineTxt
        stack = $_.ScriptStackTrace
    }
    ($errPayload | ConvertTo-Json -Compress) | Add-Content -Path $logFile -Encoding UTF8
    Write-Output "AUTOHEAL status=error error=$($_.Exception.Message) line=$lineNo"
} finally {
    Release-Lock
}

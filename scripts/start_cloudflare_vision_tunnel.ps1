param(
    [string]$RepoPath = "C:\ATLAS_PUSH",
    [string]$CredFile = "C:\dev\credenciales.txt",
    [string]$VisionBaseUrl = "http://127.0.0.1:3000",
    [string]$VisionHealthPath = "/api/health",
    [switch]$StopQuickVisionTunnels,
    [switch]$ConfigureAutoStart,
    [switch]$DisableQuickFallback
)

$ErrorActionPreference = "Stop"

function Get-CredValue {
    param([string]$Key)
    if (-not (Test-Path $CredFile)) { return $null }
    $line = Get-Content $CredFile | Where-Object { $_ -match "^$([regex]::Escape($Key))=" } | Select-Object -First 1
    if (-not $line) { return $null }
    return ($line -replace "^$([regex]::Escape($Key))=", "").Trim()
}

function Upsert-CredValue {
    param(
        [string]$Key,
        [string]$Value
    )
    if (-not (Test-Path $CredFile)) { return }
    $lines = Get-Content -Path $CredFile
    $prefix = "$Key="
    $updated = $false
    for ($i = 0; $i -lt $lines.Count; $i++) {
        if ($lines[$i].StartsWith($prefix)) {
            $lines[$i] = "$Key=$Value"
            $updated = $true
            break
        }
    }
    if (-not $updated) {
        $lines += "$Key=$Value"
    }
    Set-Content -Path $CredFile -Value $lines -Encoding UTF8
}

function Get-CloudflaredPath {
    $cmd = Get-Command cloudflared -ErrorAction SilentlyContinue
    if ($cmd -and $cmd.Path) { return $cmd.Path }
    $fallback = "C:\Program Files (x86)\cloudflared\cloudflared.exe"
    if (Test-Path $fallback) { return $fallback }
    throw "cloudflared not found. Install with: winget install Cloudflare.cloudflared"
}

function Build-VisionHealthUrl {
    param(
        [string]$BaseUrl,
        [string]$HealthPath
    )
    $base = ("" + $BaseUrl).TrimEnd("/")
    if (-not $base) { return "" }
    $path = ("" + $HealthPath).Trim()
    if (-not $path) { $path = "/api/health" }
    if (-not $path.StartsWith("/")) { $path = "/$path" }
    return "$base$path"
}

function Test-VisionPublicHealth {
    param(
        [string]$BaseUrl,
        [string]$HealthPath,
        [int]$TimeoutSec = 10
    )
    if (-not $BaseUrl) { return $false }
    $url = Build-VisionHealthUrl -BaseUrl $BaseUrl -HealthPath $HealthPath
    if (-not $url) { return $false }
    try {
        $resp = Invoke-WebRequest -Uri $url -UseBasicParsing -TimeoutSec $TimeoutSec -MaximumRedirection 5
        if ([int]$resp.StatusCode -ne 200) { return $false }
        $content = "" + $resp.Content
        if ($content -match '"status"\s*:\s*"ok"') { return $true }
        if ($content -match '"ok"\s*:\s*true') { return $true }
        return $false
    } catch {
        return $false
    }
}

function Get-QuickTunnelUrlFromLog {
    param([string]$LogFile)
    if (-not (Test-Path $LogFile)) { return $null }
    try {
        $tail = (Get-Content -Path $LogFile -Tail 500 -ErrorAction Stop) -join "`n"
        $m = [regex]::Matches($tail, "https://[a-z0-9-]+\.trycloudflare\.com", [System.Text.RegularExpressions.RegexOptions]::IgnoreCase)
        if ($m.Count -gt 0) {
            return $m[$m.Count - 1].Value.ToLower()
        }
    } catch {}
    return $null
}

function Stop-QuickVisionTunnels {
    param([string]$BaseUrl)
    $escaped = [regex]::Escape($BaseUrl)
    $quick = Get-CimInstance Win32_Process | Where-Object {
        $_.Name -match "cloudflared" -and $_.CommandLine -match "\s--url\s+$escaped"
    }
    foreach ($p in $quick) {
        try {
            Stop-Process -Id $p.ProcessId -Force -ErrorAction Stop
            Write-Host "[OK] vision quick tunnel stopped PID=$($p.ProcessId)" -ForegroundColor Green
        } catch {
            Write-Warning "Could not stop PID=$($p.ProcessId): $($_.Exception.Message)"
        }
    }
}

function Ensure-QuickVisionFallback {
    param(
        [string]$CloudflaredPath,
        [string]$RepoPath,
        [string]$BaseUrl
    )
    $quickLog = Join-Path $RepoPath "logs\cloudflared_vision_quick.log"
    $startedNow = $false
    $escaped = [regex]::Escape($BaseUrl)
    $quickRunning = Get-CimInstance Win32_Process | Where-Object {
        $_.Name -match "cloudflared" -and $_.CommandLine -match "\s--url\s+$escaped"
    }
    if (-not $quickRunning) {
        if (Test-Path $quickLog) {
            Remove-Item -Path $quickLog -Force -ErrorAction SilentlyContinue
        }
        Start-Process -FilePath $CloudflaredPath -ArgumentList @(
            "tunnel",
            "--no-autoupdate",
            "--protocol", "quic",
            "--loglevel", "info",
            "--logfile", $quickLog,
            "--url", $BaseUrl
        ) -WindowStyle Hidden | Out-Null
        $startedNow = $true
        Start-Sleep -Seconds 2
    }

    $url = $null
    $maxTries = 10
    if ($startedNow) { $maxTries = 20 }
    for ($i = 0; $i -lt $maxTries; $i++) {
        $url = Get-QuickTunnelUrlFromLog -LogFile $quickLog
        if ($url) { break }
        Start-Sleep -Seconds 1
    }
    return $url
}

function Resolve-NamedVisionBase {
    $namedHost = Get-CredValue -Key "CLOUDFLARE_VISION_TUNNEL_NAME"
    if (-not $namedHost) {
        $dashHost = Get-CredValue -Key "CLOUDFLARE_TUNNEL_NAME"
        if ($dashHost -and $dashHost.Contains(".")) {
            $parts = $dashHost.Split(".", 2)
            if ($parts.Count -eq 2) {
                $namedHost = "vision.$($parts[1])"
            }
        }
    }
    if ($namedHost -and $namedHost.Contains(".")) {
        return "https://$namedHost"
    }
    return ""
}

function Upsert-RunKey {
    param(
        [string]$Name,
        [string]$Value
    )
    $runKeyPath = "HKCU:\Software\Microsoft\Windows\CurrentVersion\Run"
    if (-not (Test-Path $runKeyPath)) {
        New-Item -Path $runKeyPath | Out-Null
    }
    New-ItemProperty -Path $runKeyPath -Name $Name -PropertyType String -Value $Value -Force | Out-Null
}

$visionBase = $VisionBaseUrl.TrimEnd("/")
if (-not $visionBase) {
    throw "VisionBaseUrl is required."
}
New-Item -ItemType Directory -Path (Join-Path $RepoPath "logs") -Force | Out-Null

if ($StopQuickVisionTunnels.IsPresent) {
    Stop-QuickVisionTunnels -BaseUrl $visionBase
}

$cloudflared = Get-CloudflaredPath
$namedBase = Resolve-NamedVisionBase
$activeVision = Get-CredValue -Key "ATLAS_VISION_PUBLIC_URL"
if (-not $activeVision) {
    $activeVision = Get-CredValue -Key "ATLAS_VISION_APP_URL"
}

if ($namedBase -and (Test-VisionPublicHealth -BaseUrl $namedBase -HealthPath $VisionHealthPath -TimeoutSec 10)) {
    Upsert-CredValue -Key "ATLAS_VISION_APP_URL" -Value $namedBase
    Upsert-CredValue -Key "ATLAS_VISION_PUBLIC_URL" -Value $namedBase
    Upsert-CredValue -Key "ATLAS_VISION_PUBLIC_API_URL" -Value $namedBase
    Upsert-CredValue -Key "CLOUDFLARE_VISION_TUNNEL_URL" -Value $namedBase
    if ($activeVision -and ($activeVision -ne $namedBase)) {
        Write-Host "[OK] vision named host recovered, active URL restored: $namedBase" -ForegroundColor Green
        Stop-QuickVisionTunnels -BaseUrl $visionBase
    } else {
        Write-Host "[OK] vision named host healthy: $namedBase" -ForegroundColor Green
    }
} elseif (-not $DisableQuickFallback.IsPresent) {
    $localOk = Test-VisionPublicHealth -BaseUrl $visionBase -HealthPath $VisionHealthPath -TimeoutSec 8
    if (-not $localOk) {
        Write-Warning "Local Vision health is not OK at $visionBase. Quick fallback skipped."
    } elseif ($activeVision -and (Test-VisionPublicHealth -BaseUrl $activeVision -HealthPath $VisionHealthPath -TimeoutSec 10)) {
        Write-Host "[OK] active Vision URL already healthy: $activeVision" -ForegroundColor Green
    } else {
        Write-Warning "Vision named host not healthy. Enabling quick tunnel fallback."
        $quickUrl = Ensure-QuickVisionFallback -CloudflaredPath $cloudflared -RepoPath $RepoPath -BaseUrl $visionBase
        if ($quickUrl) {
            Upsert-CredValue -Key "ATLAS_VISION_APP_URL" -Value $quickUrl
            Upsert-CredValue -Key "ATLAS_VISION_PUBLIC_URL" -Value $quickUrl
            Upsert-CredValue -Key "ATLAS_VISION_PUBLIC_API_URL" -Value $quickUrl
            Upsert-CredValue -Key "CLOUDFLARE_VISION_TUNNEL_URL" -Value $quickUrl
            Write-Host "[OK] vision quick fallback active: $quickUrl" -ForegroundColor Green
        } else {
            Write-Warning "Could not read trycloudflare URL from Vision quick tunnel log."
        }
    }
}

if ($ConfigureAutoStart.IsPresent) {
    $runner = "powershell -NoProfile -ExecutionPolicy Bypass -WindowStyle Hidden -File `"$PSScriptRoot\start_cloudflare_vision_tunnel.ps1`" -StopQuickVisionTunnels"
    Upsert-RunKey -Name "ATLAS_CLOUDFLARED_VISION" -Value $runner
    Write-Host "[OK] autostart configured in HKCU Run: ATLAS_CLOUDFLARED_VISION" -ForegroundColor Green
}

Get-CimInstance Win32_Process | Where-Object {
    $_.Name -match "cloudflared"
} | Select-Object ProcessId, Name, CommandLine | Format-Table -AutoSize

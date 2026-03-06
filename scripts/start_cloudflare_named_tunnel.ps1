param(
    [string]$RepoPath = "C:\ATLAS_PUSH",
    [string]$CredFile = "C:\dev\credenciales.txt",
    [string]$HealthUrl = "http://127.0.0.1:8791/health",
    [switch]$StopQuickTunnels,
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
    throw "cloudflared no encontrado. Instálalo con: winget install Cloudflare.cloudflared"
}

function Get-ServiceBaseUrl {
    param([string]$ProbeUrl)
    try {
        $u = [Uri]$ProbeUrl
        return ("{0}://{1}:{2}" -f $u.Scheme, $u.Host, $u.Port)
    } catch {
        return "http://127.0.0.1:8791"
    }
}

function Test-AtlasPublicHealth {
    param(
        [string]$BaseUrl,
        [int]$TimeoutSec = 10
    )
    if (-not $BaseUrl) { return $false }
    $health = ($BaseUrl.TrimEnd("/") + "/health")
    try {
        $resp = Invoke-WebRequest -Uri $health -UseBasicParsing -TimeoutSec $TimeoutSec -MaximumRedirection 5
        if ([int]$resp.StatusCode -ne 200) { return $false }
        $content = "" + $resp.Content
        return ($content -match '"service"\s*:\s*"atlas_push"')
    } catch {
        return $false
    }
}

function Get-QuickTunnelUrlFromLog {
    param([string]$LogFile)
    if (-not (Test-Path $LogFile)) { return $null }
    try {
        $tail = (Get-Content -Path $LogFile -Tail 400 -ErrorAction Stop) -join "`n"
        $m = [regex]::Matches($tail, "https://[a-z0-9-]+\.trycloudflare\.com", [System.Text.RegularExpressions.RegexOptions]::IgnoreCase)
        if ($m.Count -gt 0) {
            return $m[$m.Count - 1].Value.ToLower()
        }
    } catch {}
    return $null
}

function Ensure-QuickTunnelFallback {
    param(
        [string]$CloudflaredPath,
        [string]$RepoPath,
        [string]$ServiceBaseUrl
    )
    $quickLog = Join-Path $RepoPath "logs\cloudflared_quick.log"
    $startedNow = $false
    $quickRunning = Get-CimInstance Win32_Process | Where-Object {
        $_.Name -match "cloudflared" -and $_.CommandLine -match "\s--url\s+$([regex]::Escape($ServiceBaseUrl))"
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
            "--url", $ServiceBaseUrl
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

function Stop-QuickTunnels {
    $quick = Get-CimInstance Win32_Process | Where-Object {
        $_.Name -match "cloudflared" -and $_.CommandLine -match "\s--url\s+http://127\.0\.0\.1:"
    }
    foreach ($p in $quick) {
        try {
            Stop-Process -Id $p.ProcessId -Force -ErrorAction Stop
            Write-Host "[OK] quick tunnel detenido PID=$($p.ProcessId)" -ForegroundColor Green
        } catch {
            Write-Warning "No se pudo detener PID=$($p.ProcessId): $($_.Exception.Message)"
        }
    }
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

New-Item -ItemType Directory -Path (Join-Path $RepoPath "logs") -Force | Out-Null

$token = ("" + $env:CLOUDFLARE_TOKEN).Trim()
if (-not $token) {
    $token = Get-CredValue -Key "CLOUDFLARE_TOKEN"
}
if (-not $token) {
    throw "Falta CLOUDFLARE_TOKEN en entorno o en $CredFile. Ejecuta cloudflare_finalize_named_tunnel.ps1 primero."
}

$cloudflared = Get-CloudflaredPath
$logFile = Join-Path $RepoPath "logs\cloudflared_named.log"

if ($StopQuickTunnels.IsPresent) {
    Stop-QuickTunnels
}

$named = Get-CimInstance Win32_Process | Where-Object {
    $_.Name -match "cloudflared" -and $_.CommandLine -match "tunnel.*run.*--token"
}
if ($named) {
    Write-Host "[OK] túnel nombrado ya en ejecución." -ForegroundColor Green
} else {
    Start-Process -FilePath $cloudflared -ArgumentList @(
        "tunnel",
        "--no-autoupdate",
        "--protocol", "quic",
        "--loglevel", "info",
        "--logfile", $logFile,
        "run",
        "--token", $token
    ) -WindowStyle Hidden | Out-Null
    Start-Sleep -Seconds 2
    $namedNow = Get-CimInstance Win32_Process | Where-Object {
        $_.Name -match "cloudflared" -and $_.CommandLine -match "tunnel.*run.*--token"
    }
    if (-not $namedNow) {
        throw "No se logró iniciar cloudflared túnel nombrado."
    }
    Write-Host "[OK] túnel nombrado iniciado." -ForegroundColor Green
}

if ($ConfigureAutoStart.IsPresent) {
    $runner = "powershell -NoProfile -ExecutionPolicy Bypass -WindowStyle Hidden -File `"$PSScriptRoot\start_cloudflare_named_tunnel.ps1`" -StopQuickTunnels"
    Upsert-RunKey -Name "ATLAS_CLOUDFLARED_NAMED" -Value $runner
    Write-Host "[OK] autoarranque configurado en HKCU Run: ATLAS_CLOUDFLARED_NAMED" -ForegroundColor Green
}

$enableQuickFallback = -not $DisableQuickFallback.IsPresent
if ($enableQuickFallback) {
    $namedHost = Get-CredValue -Key "CLOUDFLARE_TUNNEL_NAME"
    $namedBase = $null
    if ($namedHost -and $namedHost.Contains(".")) {
        $namedBase = "https://$namedHost"
    }
    $publicBase = Get-CredValue -Key "CLOUDFLARE_TUNNEL_URL"
    if (-not $publicBase) {
        $publicBase = Get-CredValue -Key "ATLAS_DASHBOARD_PUBLIC_URL"
    }
    $serviceBase = Get-ServiceBaseUrl -ProbeUrl $HealthUrl

    if ($namedBase -and (Test-AtlasPublicHealth -BaseUrl $namedBase -TimeoutSec 10)) {
        Upsert-CredValue -Key "CLOUDFLARE_TUNNEL_URL" -Value $namedBase
        Upsert-CredValue -Key "ATLAS_DASHBOARD_PUBLIC_URL" -Value $namedBase
        if ($publicBase -and ($publicBase -ne $namedBase)) {
            Write-Host "[OK] host nombrado recuperado, URL activa restaurada: $namedBase" -ForegroundColor Green
            Stop-QuickTunnels
        }
    } elseif ($publicBase -and (-not (Test-AtlasPublicHealth -BaseUrl $publicBase -TimeoutSec 10))) {
        Write-Warning "Host público nombrado no responde health de ATLAS. Activando fallback quick tunnel."
        $quickUrl = Ensure-QuickTunnelFallback -CloudflaredPath $cloudflared -RepoPath $RepoPath -ServiceBaseUrl $serviceBase
        if ($quickUrl) {
            Upsert-CredValue -Key "CLOUDFLARE_TUNNEL_URL" -Value $quickUrl
            Upsert-CredValue -Key "ATLAS_DASHBOARD_PUBLIC_URL" -Value $quickUrl
            Write-Host "[OK] fallback quick tunnel activo: $quickUrl" -ForegroundColor Green
        } else {
            Write-Warning "No se pudo obtener URL trycloudflare desde logs del quick tunnel."
        }
    }
}

Get-CimInstance Win32_Process | Where-Object {
    $_.Name -match "cloudflared"
} | Select-Object ProcessId, Name, CommandLine | Format-Table -AutoSize

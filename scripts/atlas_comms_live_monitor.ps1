param(
    [string]$BaseUrl = "http://127.0.0.1:8791",
    [int]$RefreshSeconds = 2,
    [int]$HistoryLimit = 12,
    [switch]$IncludeNonWhatsapp,
    [switch]$ClearScreen,
    [switch]$Once
)

$ErrorActionPreference = "Stop"

function Get-JsonOrNull {
    param(
        [Parameter(Mandatory = $true)][string]$Url
    )
    try {
        return Invoke-RestMethod -Method GET -Uri $Url -TimeoutSec 6
    } catch {
        return $null
    }
}

function SafeText {
    param(
        [object]$Value,
        [int]$Max = 120
    )
    if ($null -eq $Value) {
        $text = ""
    } else {
        $text = [string]$Value
    }
    $text = $text -replace "[\r\n\t]+", " "
    $text = $text.Trim()
    if ($text.Length -le $Max) { return $text }
    return $text.Substring(0, [Math]::Max(0, $Max - 3)) + "..."
}

$base = $BaseUrl.TrimEnd("/")
$seen = @{}

Write-Host "ATLAS Comms Live Monitor" -ForegroundColor Cyan
Write-Host "Base URL: $base" -ForegroundColor DarkCyan
Write-Host "Ctrl+C para salir." -ForegroundColor DarkGray
Write-Host ""

while ($true) {
    if ($ClearScreen) {
        Clear-Host
        Write-Host "ATLAS Comms Live Monitor" -ForegroundColor Cyan
        Write-Host "Base URL: $base" -ForegroundColor DarkCyan
        Write-Host "Ctrl+C para salir." -ForegroundColor DarkGray
        Write-Host ""
    }

    $now = Get-Date -Format "yyyy-MM-dd HH:mm:ss"
    $wa = Get-JsonOrNull -Url "$base/api/comms/whatsapp/status"
    $hub = Get-JsonOrNull -Url "$base/api/comms/atlas/status"
    $hist = Get-JsonOrNull -Url "$base/api/comms/atlas/history?limit=$HistoryLimit"

    if (-not $wa -or -not $hub -or -not $hist) {
        Write-Host "[$now] ERROR consultando endpoints de comunicaciones." -ForegroundColor Red
        if ($Once) { break }
        Start-Sleep -Seconds $RefreshSeconds
        continue
    }

    if ($null -eq $wa.provider_status -or [string]::IsNullOrWhiteSpace([string]$wa.provider_status.status)) {
        $waState = "unknown"
    } else {
        $waState = [string]$wa.provider_status.status
    }
    $waAuth = [bool]($wa.provider_status.authenticated)
    $waReady = [bool]($wa.ready)
    if ($null -eq $hub.queue_pending) {
        $queuePending = 0
    } else {
        $queuePending = [int]$hub.queue_pending
    }
    $offlineMode = [bool]($hub.offline_mode)
    if ($null -eq $hub.external_users -or $null -eq $hub.external_users.linked) {
        $linkedUsers = 0
    } else {
        $linkedUsers = [int]$hub.external_users.linked
    }

    $statusColor = "Green"
    if (-not $waReady -or -not $waAuth) { $statusColor = "Yellow" }
    if ($waState -eq "FAILED") { $statusColor = "Red" }

    Write-Host ("[{0}] WA={1} auth={2} ready={3} queue={4} offline={5} linked={6}" -f `
            $now, $waState, $waAuth, $waReady, $queuePending, $offlineMode, $linkedUsers) -ForegroundColor $statusColor

    $items = @()
    if ($hist.items) {
        $items = @($hist.items)
    }
    if (-not $IncludeNonWhatsapp) {
        $items = @($items | Where-Object { $_.channel -eq "whatsapp" })
    }

    # Mostrar nuevos eventos desde el último ciclo.
    $ordered = @($items | Sort-Object created_at)
    $newEvents = @()
    foreach ($it in $ordered) {
        if ($null -eq $it.message_id) {
            $id = ""
        } else {
            $id = [string]$it.message_id
        }
        if (-not $id) { continue }
        if (-not $seen.ContainsKey($id)) {
            $seen[$id] = $true
            $newEvents += $it
        }
    }

    if ($newEvents.Count -gt 0) {
        foreach ($ev in $newEvents) {
            $at = SafeText $ev.created_at 25
            $user = SafeText $ev.user_id 40
            $req = SafeText $ev.request 110
            $rsp = SafeText $ev.response 130
            Write-Host ("  IN  [{0}] {1}: {2}" -f $at, $user, $req) -ForegroundColor White
            Write-Host ("  OUT [{0}] {1}" -f $at, $rsp) -ForegroundColor Gray
        }
    } else {
        Write-Host "  (sin mensajes nuevos)" -ForegroundColor DarkGray
    }

    Write-Host ""

    if ($Once) { break }
    Start-Sleep -Seconds $RefreshSeconds
}

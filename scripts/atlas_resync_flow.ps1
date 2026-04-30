param(
    [string]$Workspace = "C:\ATLAS_PUSH",
    [int]$Limit = 200
)

$ErrorActionPreference = "Stop"

Set-Location $Workspace

$logFile = Join-Path $Workspace "logs\snapshot_safe_diagnostic.log"
if (!(Test-Path (Split-Path $logFile -Parent))) {
    New-Item -ItemType Directory -Path (Split-Path $logFile -Parent) -Force | Out-Null
}

function Write-Snapshot([string]$line) {
    $ts = Get-Date -Format "yyyy-MM-ddTHH:mm:ssK"
    "$ts $line" | Out-File -FilePath $logFile -Append -Encoding utf8
}

Write-Host "== ATLAS Resync Flow =="
Write-Host "Workspace: $Workspace"

$online = $false
$reason = "unknown"

try {
    $resp = Invoke-WebRequest -Uri "http://1.1.1.1/cdn-cgi/trace" -TimeoutSec 6 -UseBasicParsing
    if ($resp.StatusCode -ge 200 -and $resp.StatusCode -lt 400) {
        $online = $true
        $reason = "1.1.1.1"
    }
} catch {
    $reason = $_.Exception.Message
}

if (-not $online) {
    try {
        $resp2 = Invoke-WebRequest -Uri "https://www.cloudflare.com/cdn-cgi/trace" -TimeoutSec 6 -UseBasicParsing
        if ($resp2.StatusCode -ge 200 -and $resp2.StatusCode -lt 400) {
            $online = $true
            $reason = "cloudflare.com"
        }
    } catch {
        $reason = $_.Exception.Message
    }
}

if (-not $online) {
    Write-Snapshot "NETWORK_OFFLINE_MODE source=atlas_resync_flow reason=$reason"
    Write-Host "No internet yet. Queue remains pending."
    exit 0
}

Write-Host "Internet restored ($reason). Processing pending queue..."
python -m modules.humanoid.comms.atlas_comms_hub --resync --limit $Limit
$rc = $LASTEXITCODE

if ($rc -ne 0) {
    Write-Snapshot "ATLAS_COMMS_RESYNC_FAIL rc=$rc"
    exit $rc
}

Write-Snapshot "TASK_OK name=ATLAS_Comms_Resync source=atlas_resync_flow limit=$Limit"
Write-Host "Resync completed."

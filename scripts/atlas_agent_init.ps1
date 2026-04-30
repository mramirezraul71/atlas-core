param(
    [string]$CoreToken = "",
    [switch]$PersistUserToken,
    [bool]$RestartPush = $true
)

$ErrorActionPreference = "Stop"

$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$SnapshotLog = Join-Path $RepoRoot "logs\snapshot_safe_diagnostic.log"
$SnapshotScript = Join-Path $RepoRoot "scripts\atlas_snapshot_safe.ps1"
$RuntimeHelpers = Join-Path $RepoRoot "scripts\atlas_runtime.ps1"
$RestartPushScript = Join-Path $RepoRoot "scripts\restart_push_from_api.ps1"

if (-not (Test-Path $RuntimeHelpers)) {
    throw "Runtime helpers not found: $RuntimeHelpers"
}
. $RuntimeHelpers

$PyExe = Resolve-AtlasPython -RepoRoot $RepoRoot -RequirePreflight

if ($CoreToken) {
    $env:ATLAS_CENTRAL_CORE = $CoreToken
    if ($PersistUserToken.IsPresent) {
        setx ATLAS_CENTRAL_CORE $CoreToken | Out-Null
    }
}

New-Item -ItemType Directory -Path (Join-Path $RepoRoot "logs") -Force | Out-Null
if (-not (Test-Path $SnapshotLog)) {
    New-Item -ItemType File -Path $SnapshotLog -Force | Out-Null
}

function Write-Snapshot([string]$line) {
    $ts = Get-Date -Format "yyyy-MM-ddTHH:mm:ssK"
    "$ts ATLAS_AGENT_INIT $line" | Out-File -FilePath $SnapshotLog -Append -Encoding utf8
}

Write-Snapshot "START repo=$RepoRoot python=$PyExe"
if ($env:ATLAS_CENTRAL_CORE) {
    Write-Snapshot "TOKEN_SOURCE=env:ATLAS_CENTRAL_CORE"
} else {
    Write-Snapshot "TOKEN_SOURCE=missing_env_uses_fallback_if_configured"
}

if (-not (Test-Path $SnapshotScript)) {
    throw "Missing snapshot script: $SnapshotScript"
}

Write-Snapshot "RUN_SNAPSHOT_SAFE begin"
& powershell -NoProfile -ExecutionPolicy Bypass -File $SnapshotScript
if ($LASTEXITCODE -ne 0) {
    Write-Snapshot "RUN_SNAPSHOT_SAFE failed code=$LASTEXITCODE"
    throw "atlas_snapshot_safe.ps1 failed with code $LASTEXITCODE"
}
Write-Snapshot "RUN_SNAPSHOT_SAFE ok"

if ($RestartPush -and (Test-Path $RestartPushScript)) {
    Write-Snapshot "RESTART_PUSH begin"
    & powershell -NoProfile -ExecutionPolicy Bypass -File $RestartPushScript -DelaySeconds 1 -HealthTimeoutSec 90
    if ($LASTEXITCODE -ne 0) {
        Write-Snapshot "RESTART_PUSH failed code=$LASTEXITCODE"
        throw "restart_push_from_api.ps1 failed with code $LASTEXITCODE"
    }
    Write-Snapshot "RESTART_PUSH ok"
}

$initJson = & $PyExe -m tools.atlas_clawd_bridge.bridge init --json
if ($LASTEXITCODE -ne 0) {
    Write-Snapshot "BRIDGE_INIT failed code=$LASTEXITCODE"
    throw "atlas_clawd_bridge init command failed with code $LASTEXITCODE"
}

try {
    $initObj = $initJson | ConvertFrom-Json -ErrorAction Stop
    $stable = $false
    if ($initObj -and $initObj.stability -and $initObj.stability.status) {
        $stable = [bool]$initObj.stability.status.stable
    }
    Write-Snapshot ("BRIDGE_INIT stable=" + $stable)
    if (-not $stable) {
        throw "Bridge initialized but snapshot status is not stable."
    }
} catch {
    Write-Snapshot ("BRIDGE_INIT parse_error=" + $_.Exception.Message)
    throw
}

try {
    $health = Invoke-WebRequest -Uri "http://127.0.0.1:8791/api/clawd/bridge/status" -UseBasicParsing -TimeoutSec 15 -ErrorAction Stop
    Write-Snapshot ("HTTP_STATUS endpoint=/api/clawd/bridge/status code=" + [string]$health.StatusCode)
} catch {
    Write-Snapshot ("HTTP_STATUS endpoint=/api/clawd/bridge/status error=" + $_.Exception.Message)
    throw
}

Write-Snapshot "DONE"
Write-Output "ATLAS_AGENT_INIT_OK bridge=atlas_clawd_bridge stable=true"

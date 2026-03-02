param(
    [string]$CoreToken = "",
    [switch]$SkipBrowserInstall
)

$ErrorActionPreference = "Stop"

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$actuatorDir = Join-Path $repoRoot "tools\atlas_actuators"
$snapshotLog = Join-Path $repoRoot "logs\snapshot_safe_diagnostic.log"

if (-not (Test-Path $actuatorDir)) {
    throw "No existe directorio de actuadores: $actuatorDir"
}

New-Item -ItemType Directory -Path (Join-Path $repoRoot "logs") -Force | Out-Null

function Write-Snapshot([string]$line) {
    $ts = Get-Date -Format "yyyy-MM-ddTHH:mm:ssK"
    "$ts ATLAS_ACTUATOR_INSTALL $line" | Out-File -FilePath $snapshotLog -Append -Encoding utf8
}

if ($CoreToken) {
    $env:ATLAS_CENTRAL_CORE = $CoreToken
}

if ($env:ATLAS_CENTRAL_CORE) {
    Write-Snapshot "TOKEN_SOURCE=env:ATLAS_CENTRAL_CORE"
} else {
    Write-Snapshot "TOKEN_SOURCE=governance_config_fallback"
}

Write-Snapshot "START dir=$actuatorDir"

Push-Location $actuatorDir
try {
    npm install
    Write-Snapshot "NPM_INSTALL_OK"

    if (-not $SkipBrowserInstall) {
        npx playwright install chromium
        Write-Snapshot "PLAYWRIGHT_BROWSER_INSTALL_OK chromium"
    } else {
        Write-Snapshot "PLAYWRIGHT_BROWSER_INSTALL_SKIPPED"
    }

    node .\atlas_actuator_healthcheck.js
    if ($LASTEXITCODE -eq 0) {
        Write-Snapshot "HEALTHCHECK_OK"
    } else {
        Write-Snapshot "HEALTHCHECK_WARN code=$LASTEXITCODE"
    }
}
finally {
    Pop-Location
}

Write-Output "ACTUATOR_INSTALL_OK dir=$actuatorDir skip_browser=$SkipBrowserInstall"

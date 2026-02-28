param(
    [string]$RepoRoot = $(Resolve-Path (Join-Path $PSScriptRoot "..")).Path
)

$ErrorActionPreference = "SilentlyContinue"

$logDir = Join-Path $RepoRoot "logs"
New-Item -ItemType Directory -Path $logDir -Force | Out-Null
$report = Join-Path $logDir "snapshot_safe_diagnostic.log"

function Write-Line([string]$text) {
    $ts = Get-Date -Format "yyyy-MM-ddTHH:mm:ssK"
    "$ts $text" | Out-File -FilePath $report -Append -Encoding utf8
}

function HttpCode([string]$url) {
    try {
        $r = Invoke-WebRequest -Uri $url -Method GET -TimeoutSec 8 -UseBasicParsing
        return [string]$r.StatusCode
    } catch {
        return "ERR"
    }
}

Write-Line "=== SNAPSHOT_SAFE_START ==="
Write-Line ("PUSH /health => " + (HttpCode "http://127.0.0.1:8791/health"))
Write-Line ("NEXUS /health => " + (HttpCode "http://127.0.0.1:8000/health"))
Write-Line ("ROBOT /status => " + (HttpCode "http://127.0.0.1:8002/status"))

try {
    $smoke = & python (Join-Path $RepoRoot "scripts/smoke_test_endpoints.py") 2>&1
    foreach ($line in $smoke) { Write-Line ("SMOKE " + $line) }
} catch {
    Write-Line ("SMOKE ERR " + $_.Exception.Message)
}

try {
    $hooks = & python (Join-Path $RepoRoot "scripts/install_git_hooks.py") 2>&1
    foreach ($line in $hooks) { Write-Line ("HOOKS " + $line) }
} catch {
    Write-Line ("HOOKS ERR " + $_.Exception.Message)
}

try {
    $listeners = Get-NetTCPConnection -State Listen |
        Where-Object { $_.LocalPort -in 8791, 8000, 8002 } |
        Select-Object LocalPort, OwningProcess
    foreach ($x in $listeners) {
        Write-Line ("LISTEN port=" + $x.LocalPort + " pid=" + $x.OwningProcess)
    }
} catch {
    Write-Line ("LISTEN ERR " + $_.Exception.Message)
}

Write-Line "=== SNAPSHOT_SAFE_END ==="

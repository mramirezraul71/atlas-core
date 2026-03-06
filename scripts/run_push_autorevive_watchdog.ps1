param(
    [string]$HealthUrl = "http://127.0.0.1:8791/status",
    [int]$IntervalSec = 15,
    [int]$FailThreshold = 3,
    [int]$RestartCooldownSec = 180
)

$ErrorActionPreference = "Continue"
$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$LogDir = Join-Path $RepoRoot "logs"
$TaskLog = Join-Path $LogDir "push_autorevive_watchdog.task.log"
$WatchdogScript = Join-Path $PSScriptRoot "push_autorevive_watchdog.ps1"

New-Item -ItemType Directory -Path $LogDir -Force | Out-Null

try {
    & powershell -NoProfile -ExecutionPolicy Bypass -File $WatchdogScript `
      -HealthUrl $HealthUrl `
      -IntervalSec $IntervalSec `
      -FailThreshold $FailThreshold `
      -RestartCooldownSec $RestartCooldownSec 2>&1 | Out-File -FilePath $TaskLog -Append -Encoding utf8
} catch {
    $ts = (Get-Date).ToString("o")
    "$ts TASK_WRAPPER_ERROR $($_.Exception.Message)" | Out-File -FilePath $TaskLog -Append -Encoding utf8
}

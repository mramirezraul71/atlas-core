param(
    [string]$TaskName = "ATLAS_PUSH_Autorevive_Watchdog",
    [string]$HealthUrl = "http://127.0.0.1:8791/status",
    [int]$IntervalSec = 15,
    [int]$FailThreshold = 3,
    [int]$RestartCooldownSec = 180,
    [switch]$RunNow
)

$ErrorActionPreference = "Stop"
$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$RunnerScript = (Resolve-Path (Join-Path $PSScriptRoot "run_push_autorevive_watchdog.ps1")).Path

if ($IntervalSec -lt 5) {
    throw "IntervalSec must be >= 5."
}
if ($FailThreshold -lt 1) {
    throw "FailThreshold must be >= 1."
}
if ($RestartCooldownSec -lt 30) {
    throw "RestartCooldownSec must be >= 30."
}

$taskCmd = "powershell -NoProfile -ExecutionPolicy Bypass -WindowStyle Hidden -File `"$RunnerScript`" -HealthUrl `"$HealthUrl`" -IntervalSec $IntervalSec -FailThreshold $FailThreshold -RestartCooldownSec $RestartCooldownSec"

$create = @(
    "/Create",
    "/TN", $TaskName,
    "/SC", "ONLOGON",
    "/TR", $taskCmd,
    "/RL", "HIGHEST",
    "/F"
)

Write-Host "Creating/updating task: $TaskName (ONLOGON)"
schtasks $create | Out-Host

if ($RunNow.IsPresent) {
    Write-Host "Running task immediately: $TaskName"
    schtasks /Run /TN $TaskName | Out-Host
}

Write-Host "Task configured."
schtasks /Query /TN $TaskName /V /FO LIST | Out-Host

# Register a periodic Windows Scheduled Task for ATLAS agent E2E audits.
#
# Example:
#   powershell -NoProfile -ExecutionPolicy Bypass -File scripts/register_atlas_agent_e2e_task.ps1 -EveryMinutes 30 -RunNow
#
param(
    [string]$TaskName = "ATLAS_Agent_E2E_Audit",
    [int]$EveryMinutes = 30,
    [switch]$WithRestart,
    [switch]$RunQualityGates,
    [switch]$RunNow
)

$ErrorActionPreference = "Stop"
$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path

function Resolve-Python {
    param([string]$Root)
    $candidates = @(
        (Join-Path $Root ".venv\Scripts\python.exe"),
        (Join-Path $Root "venv\Scripts\python.exe")
    )
    foreach ($p in $candidates) {
        if (Test-Path $p) {
            return (Resolve-Path $p).Path
        }
    }
    $cmd = Get-Command python -ErrorAction SilentlyContinue
    if ($cmd -and $cmd.Source) {
        return $cmd.Source
    }
    throw "Python interpreter not found (.venv/venv/python)."
}

if ($EveryMinutes -lt 5) {
    throw "EveryMinutes must be >= 5."
}

$PyExe = Resolve-Python -Root $RepoRoot
$AuditScript = (Resolve-Path (Join-Path $RepoRoot "scripts\atlas_agent_e2e_audit.py")).Path
$TaskRunner = (Resolve-Path (Join-Path $RepoRoot "scripts\run_atlas_agent_e2e_task.ps1")).Path
$TaskLog = Join-Path $RepoRoot "logs\atlas_agent_e2e_task.log"

$qualityFlag = if ($RunQualityGates.IsPresent) { "true" } else { "false" }
$taskParts = @(
    "powershell.exe",
    "-NoProfile",
    "-ExecutionPolicy", "Bypass",
    "-WindowStyle", "Hidden",
    "-File", "`"$TaskRunner`"",
    "-RepoRoot", "`"$RepoRoot`"",
    "-PythonExe", "`"$PyExe`"",
    "-RunQualityGates", $qualityFlag
)
if ($WithRestart.IsPresent) {
    $taskParts += "-WithRestart"
}
$taskCmd = $taskParts -join " "

$create = @(
    "/Create",
    "/TN", $TaskName,
    "/SC", "MINUTE",
    "/MO", "$EveryMinutes",
    "/TR", $taskCmd,
    "/RL", "LIMITED",
    "/F"
)

Write-Host "Creating/updating task: $TaskName (every $EveryMinutes min)"
schtasks $create | Out-Host

if ($RunNow.IsPresent) {
    Write-Host "Running task immediately: $TaskName"
    schtasks /Run /TN $TaskName | Out-Host
}

Write-Host "Task configured."
schtasks /Query /TN $TaskName /V /FO LIST | Out-Host

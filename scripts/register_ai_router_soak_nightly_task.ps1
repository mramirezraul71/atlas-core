# Register a nightly Windows Scheduled Task for ATLAS Multi-AI soak validation.
#
# Example:
#   powershell -NoProfile -ExecutionPolicy Bypass -File scripts/register_ai_router_soak_nightly_task.ps1 -StartTime "02:10" -RunAsSystem -RunNow
#
param(
    [string]$TaskName = "ATLAS_MultiAI_Soak_Nightly",
    [string]$StartTime = "02:10",
    [int]$Rounds = 3,
    [int]$Timeout = 120,
    [double]$SleepSeconds = 0.25,
    [switch]$NoWarmup,
    [switch]$RunAsSystem,
    [switch]$RunNow
)

$ErrorActionPreference = "Stop"
$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path

function Test-IsAdministrator {
    $identity = [Security.Principal.WindowsIdentity]::GetCurrent()
    $principal = New-Object Security.Principal.WindowsPrincipal($identity)
    return $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
}

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

if ($Rounds -lt 1) {
    throw "Rounds must be >= 1."
}
if ($Timeout -lt 10) {
    throw "Timeout must be >= 10."
}
if ($SleepSeconds -lt 0) {
    throw "SleepSeconds must be >= 0."
}
if ($StartTime -notmatch "^\d{2}:\d{2}$") {
    throw "StartTime must use HH:mm format (e.g., 02:10)."
}
if ($RunAsSystem.IsPresent -and -not (Test-IsAdministrator)) {
    throw "RunAsSystem requires elevated PowerShell (Run as Administrator)."
}

$PyExe = Resolve-Python -Root $RepoRoot
$TaskRunner = (Resolve-Path (Join-Path $RepoRoot "scripts\run_ai_router_soak_task.ps1")).Path

$warmupArg = if ($NoWarmup.IsPresent) { "-NoWarmup" } else { "" }
$taskParts = @(
    "powershell.exe",
    "-NoProfile",
    "-ExecutionPolicy", "Bypass",
    "-WindowStyle", "Hidden",
    "-File", "`"$TaskRunner`"",
    "-RepoRoot", "`"$RepoRoot`"",
    "-PythonExe", "`"$PyExe`"",
    "-Rounds", "$Rounds",
    "-Timeout", "$Timeout",
    "-SleepSeconds", "$SleepSeconds"
)
if ($warmupArg) {
    $taskParts += $warmupArg
}
$taskCmd = $taskParts -join " "

$create = @(
    "/Create",
    "/TN", $TaskName,
    "/SC", "DAILY",
    "/ST", $StartTime,
    "/TR", $taskCmd,
    "/F"
)

if ($RunAsSystem.IsPresent) {
    $create += @("/RU", "SYSTEM", "/RL", "HIGHEST")
} else {
    $create += @("/RL", "LIMITED")
}

Write-Host "Creating/updating task: $TaskName (daily $StartTime)"
schtasks $create | Out-Host
if ($LASTEXITCODE -ne 0) {
    throw "Failed to create/update task '$TaskName' (exit code $LASTEXITCODE)."
}

if ($RunNow.IsPresent) {
    Write-Host "Running task immediately: $TaskName"
    schtasks /Run /TN $TaskName | Out-Host
    if ($LASTEXITCODE -ne 0) {
        throw "Failed to run task '$TaskName' (exit code $LASTEXITCODE)."
    }
}

Write-Host "Task configured."
schtasks /Query /TN $TaskName /V /FO LIST | Out-Host
if ($LASTEXITCODE -ne 0) {
    throw "Failed to query task '$TaskName' (exit code $LASTEXITCODE)."
}

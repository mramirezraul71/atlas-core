param(
    [string]$TaskName = "ATLAS_PUSH_Autorevive_Watchdog",
    [string]$HealthUrl = "http://127.0.0.1:8791/health",
    [int]$IntervalSec = 30,
    [int]$FailThreshold = 8,
    [int]$RestartCooldownSec = 300,
    [ValidateSet("LIMITED","HIGHEST")][string]$RunLevel = "LIMITED",
    [switch]$SkipRunKeyFallback,
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
    "/RL", $RunLevel,
    "/F"
)

Write-Host "Creating/updating task: $TaskName (ONLOGON)"
$taskCreated = $false
try {
    schtasks $create | Out-Host
    if ($LASTEXITCODE -eq 0) {
        $taskCreated = $true
    } else {
        Write-Warning "schtasks /Create falló con código $LASTEXITCODE"
    }
} catch {
    Write-Warning "No se pudo crear tarea programada: $($_.Exception.Message)"
}

if ($taskCreated -and $RunNow.IsPresent) {
    Write-Host "Running task immediately: $TaskName"
    schtasks /Run /TN $TaskName | Out-Host
}

if ($taskCreated) {
    Write-Host "Task configured."
    schtasks /Query /TN $TaskName /V /FO LIST | Out-Host
    exit 0
}

if (-not $SkipRunKeyFallback.IsPresent) {
    $runKeyPath = "HKCU:\Software\Microsoft\Windows\CurrentVersion\Run"
    $runValueName = $TaskName
    $runValueCmd = "powershell -NoProfile -ExecutionPolicy Bypass -WindowStyle Hidden -File `"$RunnerScript`" -HealthUrl `"$HealthUrl`" -IntervalSec $IntervalSec -FailThreshold $FailThreshold -RestartCooldownSec $RestartCooldownSec"
    if (-not (Test-Path $runKeyPath)) {
        New-Item -Path $runKeyPath | Out-Null
    }
    New-ItemProperty -Path $runKeyPath -Name $runValueName -PropertyType String -Value $runValueCmd -Force | Out-Null
    Write-Warning "No se pudo crear tarea programada. Fallback aplicado en HKCU Run: $runValueName"
    if ($RunNow.IsPresent) {
        Start-Process -FilePath "powershell.exe" -ArgumentList @(
            "-NoProfile",
            "-ExecutionPolicy", "Bypass",
            "-WindowStyle", "Hidden",
            "-File", $RunnerScript,
            "-HealthUrl", $HealthUrl,
            "-IntervalSec", $IntervalSec,
            "-FailThreshold", $FailThreshold,
            "-RestartCooldownSec", $RestartCooldownSec
        ) | Out-Null
    }
    exit 0
}

throw "No se pudo registrar la tarea programada y se omitió fallback por -SkipRunKeyFallback."

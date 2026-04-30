param(
    [string]$RepoPath = "C:\ATLAS_PUSH",
    [int]$RefreshSeconds = 10,
    [switch]$ConfigureAutoStart
)

$ErrorActionPreference = "Stop"

$monitorScript = Join-Path $RepoPath "scripts\atlas_unified_monitor.ps1"
if (-not (Test-Path $monitorScript)) {
    throw "Monitor script not found: $monitorScript"
}

function Upsert-RunKey {
    param(
        [Parameter(Mandatory = $true)][string]$Name,
        [Parameter(Mandatory = $true)][string]$Value
    )
    $runPath = "HKCU:\Software\Microsoft\Windows\CurrentVersion\Run"
    if (-not (Test-Path $runPath)) {
        New-Item -Path $runPath -Force | Out-Null
    }
    New-ItemProperty -Path $runPath -Name $Name -Value $Value -PropertyType String -Force | Out-Null
}

function Get-MonitorProcesses {
    Get-CimInstance Win32_Process -ErrorAction SilentlyContinue |
        Where-Object {
            $_.Name -match "powershell" -and
            $_.CommandLine -and
            $_.CommandLine -match "atlas_unified_monitor\.ps1" -and
            $_.CommandLine -notmatch "start_atlas_unified_monitor\.ps1"
        }
}

$existing = @(Get-MonitorProcesses)
if ($existing.Count -gt 0) {
    Write-Host "ATLAS Unified Monitor already running:" -ForegroundColor Yellow
    $existing | Select-Object ProcessId, CommandLine | Format-Table -AutoSize | Out-Host
} else {
    $args = @(
        "-NoProfile",
        "-ExecutionPolicy",
        "Bypass",
        "-WindowStyle",
        "Hidden",
        "-File",
        $monitorScript,
        "-RefreshSeconds",
        [string]$RefreshSeconds,
        "-AutoHealOnCritical"
    )
    Start-Process -FilePath "powershell" -ArgumentList $args -WorkingDirectory $RepoPath -WindowStyle Hidden | Out-Null
    Start-Sleep -Seconds 1
    $started = @(Get-MonitorProcesses)
    if ($started.Count -gt 0) {
        Write-Host "ATLAS Unified Monitor started." -ForegroundColor Green
    } else {
        Write-Warning "Monitor process not detected after start."
    }
}

if ($ConfigureAutoStart.IsPresent) {
    $launcher = "powershell -NoProfile -ExecutionPolicy Bypass -WindowStyle Hidden -File `"$monitorScript`" -RefreshSeconds $RefreshSeconds -AutoHealOnCritical"
    Upsert-RunKey -Name "ATLAS_UNIFIED_MONITOR" -Value $launcher
    Write-Host "AutoStart configured in HKCU Run: ATLAS_UNIFIED_MONITOR" -ForegroundColor Green
}

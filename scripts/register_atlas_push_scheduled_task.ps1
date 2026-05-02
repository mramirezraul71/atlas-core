# Registra una tarea programada para arrancar el supervisor PUSH al iniciar sesion (Windows).
# Requiere ejecutarse una vez con privilegios suficientes para Register-ScheduledTask.
# Uso:
#   powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\register_atlas_push_scheduled_task.ps1 -RepoPath C:\dev\atlas\atlas-consolidated

param(
    [Parameter(Mandatory = $true)][string]$RepoPath,
    [string]$TaskName = "ATLAS_PUSH_Supervisor",
    [string]$BindHost = "127.0.0.1",
    [int]$Port = 8791
)

$ErrorActionPreference = "Stop"
$RepoPath = (Resolve-Path -LiteralPath $RepoPath).Path
$supervisor = Join-Path (Split-Path -Parent $PSScriptRoot) "scripts\run_push_supervisor.ps1"
if (-not (Test-Path -LiteralPath $supervisor)) {
    $supervisor = Join-Path $PSScriptRoot "run_push_supervisor.ps1"
}
if (-not (Test-Path -LiteralPath $supervisor)) {
    Write-Error "No se encontro run_push_supervisor.ps1 junto a este script."
    exit 1
}

$argLine = "-NoProfile -ExecutionPolicy Bypass -WindowStyle Hidden -File `"$supervisor`" -RepoPath `"$RepoPath`" -BindHost `"$BindHost`" -Port $Port -SkipIfHealthy"

$action = New-ScheduledTaskAction -Execute "powershell.exe" -Argument $argLine
$trigger = New-ScheduledTaskTrigger -AtLogOn
$settings = New-ScheduledTaskSettingsSet `
    -AllowStartIfOnBatteries `
    -DontStopIfGoingOnBatteries `
    -StartWhenAvailable `
    -RestartCount 3 `
    -RestartInterval (New-TimeSpan -Minutes 1)

try {
    Unregister-ScheduledTask -TaskName $TaskName -Confirm:$false -ErrorAction SilentlyContinue
}
catch { }

Register-ScheduledTask `
    -TaskName $TaskName `
    -Action $action `
    -Trigger $trigger `
    -Settings $settings `
    -Description "ATLAS PUSH: supervisor con reinicio (service_launcher + uvicorn)." `
    -RunLevel Highest | Out-Null

Write-Host "Tarea registrada: $TaskName (al iniciar sesion). RepoPath=$RepoPath" -ForegroundColor Green
Write-Host "Probar manual: powershell -File `"$supervisor`" -RepoPath `"$RepoPath`"" -ForegroundColor Cyan

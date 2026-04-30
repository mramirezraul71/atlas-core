param(
  [string]$TaskName = "ATLAS_8795_Watchdog",
  [int]$RepeatMinutes = 1,
  [Parameter(Mandatory=$false)][string]$RepoPath = "",
  [switch]$UseCurrentUser
)

$ErrorActionPreference = "Stop"

if ($RepeatMinutes -lt 1) {
  Write-Error "RepeatMinutes debe ser >= 1."
  exit 1
}

if ([string]::IsNullOrWhiteSpace($RepoPath)) {
  $scriptRoot = if ($PSScriptRoot) { $PSScriptRoot } else { Split-Path -Path $MyInvocation.MyCommand.Path -Parent }
  $RepoPath = Split-Path (Split-Path $scriptRoot -Parent) -Parent
}

$watchdogScript = Join-Path $RepoPath "scripts\windows\atlas_quant_8795_watchdog.ps1"
if (-not (Test-Path $watchdogScript)) {
  Write-Error "No existe el watchdog script: $watchdogScript"
  exit 1
}
$hiddenLauncher = Join-Path $RepoPath "scripts\windows\run_hidden_powershell.vbs"
if (-not (Test-Path $hiddenLauncher)) {
  Write-Error "No existe launcher oculto: $hiddenLauncher"
  exit 1
}

$taskCommand = "wscript.exe //B //NoLogo `"$hiddenLauncher`" `"$watchdogScript`" -Silent"
$args = @(
  "/Create",
  "/TN", $TaskName,
  "/SC", "MINUTE",
  "/MO", "$RepeatMinutes",
  "/TR", $taskCommand,
  "/F"
)

if (-not $UseCurrentUser) {
  $args += @("/RU", "SYSTEM", "/RL", "HIGHEST")
}

Write-Host "Instalando tarea programada: $TaskName" -ForegroundColor Cyan
Write-Host "  Script: $watchdogScript" -ForegroundColor Gray
Write-Host "  Frecuencia: cada $RepeatMinutes minuto(s)" -ForegroundColor Gray
if (-not $UseCurrentUser) {
  Write-Host "  Contexto: SYSTEM (requiere permisos de administrador)" -ForegroundColor Gray
} else {
  Write-Host "  Contexto: usuario actual" -ForegroundColor Gray
}

& schtasks.exe @args
if ($LASTEXITCODE -ne 0) {
  Write-Error "No se pudo crear la tarea ($TaskName). Codigo: $LASTEXITCODE"
  exit $LASTEXITCODE
}

Write-Host "Tarea creada correctamente." -ForegroundColor Green
Write-Host "Verifica con: schtasks /Query /TN $TaskName /V /FO LIST" -ForegroundColor Green

<#
.SYNOPSIS
    Registra el Vault Daily Recharge como tarea programada de Windows.
    Corre diariamente a las 3:30 AM (configurable en vault_recharge_config.json).

.USAGE
    # Instalar la tarea (una sola vez):
    .\scripts\setup_vault_scheduler.ps1

    # Desinstalar:
    .\scripts\setup_vault_scheduler.ps1 -Uninstall

    # Ejecutar ahora manualmente:
    .\scripts\setup_vault_scheduler.ps1 -RunNow

    # Ver estado de la tarea:
    .\scripts\setup_vault_scheduler.ps1 -Status
#>

param(
    [switch]$Uninstall,
    [switch]$RunNow,
    [switch]$Status
)

$TaskName    = "ATLAS-VaultDailyRecharge"
$ScriptDir   = Split-Path -Parent $MyInvocation.MyCommand.Path
$RootDir     = Split-Path -Parent $ScriptDir
$PythonExe   = Join-Path $RootDir "venv\Scripts\python.exe"
$ScriptPath  = Join-Path $ScriptDir "vault_daily_recharge.py"
$LogPath     = Join-Path $RootDir "logs\vault_recharge_scheduler.log"
$ConfigPath  = Join-Path $ScriptDir "vault_recharge_config.json"
$TaskTime    = "03:30"

# Leer hora de la config si existe
if (Test-Path $ConfigPath) {
    try {
        $cfg = Get-Content $ConfigPath -Raw | ConvertFrom-Json
        $h   = [string]$cfg.schedule_hour
        $m   = [string]$cfg.schedule_minute
        $TaskTime = "{0:D2}:{1:D2}" -f [int]$h, [int]$m
    } catch {}
}

Write-Host ""
Write-Host "==============================================" -ForegroundColor Cyan
Write-Host "  ATLAS - Vault Daily Recharge Scheduler"     -ForegroundColor Cyan
Write-Host "==============================================" -ForegroundColor Cyan
Write-Host ""

# ── STATUS ────────────────────────────────────────────────────────────────────
if ($Status) {
    $t = Get-ScheduledTask -TaskName $TaskName -ErrorAction SilentlyContinue
    if ($t) {
        $info = Get-ScheduledTaskInfo -TaskName $TaskName
        Write-Host "OK   Tarea registrada: $TaskName" -ForegroundColor Green
        Write-Host "     Estado:           $($t.State)"
        Write-Host "     Ultima ejecucion: $($info.LastRunTime)"
        Write-Host "     Resultado:        $($info.LastTaskResult)"
        Write-Host "     Proxima ejecucion: $($info.NextRunTime)"
    } else {
        Write-Host "--   Tarea NO registrada: $TaskName" -ForegroundColor Yellow
    }
    exit 0
}

# ── UNINSTALL ─────────────────────────────────────────────────────────────────
if ($Uninstall) {
    $t = Get-ScheduledTask -TaskName $TaskName -ErrorAction SilentlyContinue
    if ($t) {
        Unregister-ScheduledTask -TaskName $TaskName -Confirm:$false
        Write-Host "OK   Tarea eliminada: $TaskName" -ForegroundColor Green
    } else {
        Write-Host "--   Tarea no encontrada: $TaskName" -ForegroundColor Yellow
    }
    exit 0
}

# ── RUN NOW ───────────────────────────────────────────────────────────────────
if ($RunNow) {
    Write-Host "Ejecutando recharge ahora..." -ForegroundColor Yellow
    $t = Get-ScheduledTask -TaskName $TaskName -ErrorAction SilentlyContinue
    if ($t) {
        Start-ScheduledTask -TaskName $TaskName
        Write-Host "OK   Tarea lanzada en background." -ForegroundColor Green
        Write-Host "     Ver log: $LogPath"
    } else {
        Write-Host "ERR  Tarea no registrada. Ejecuta el script sin parametros primero." -ForegroundColor Red
    }
    exit 0
}

# ── INSTALL ───────────────────────────────────────────────────────────────────
Write-Host "Python:  $PythonExe"
Write-Host "Script:  $ScriptPath"
Write-Host "Log:     $LogPath"
Write-Host "Hora:    $TaskTime diario"
Write-Host ""

if (-not (Test-Path $PythonExe)) {
    Write-Host "ERR  Python no encontrado en: $PythonExe" -ForegroundColor Red
    Write-Host "     Ejecuta 01_setup_venv.ps1 primero." -ForegroundColor Red
    exit 1
}

if (-not (Test-Path $ScriptPath)) {
    Write-Host "ERR  Script no encontrado: $ScriptPath" -ForegroundColor Red
    exit 1
}

# Verificar/instalar yt-dlp
Write-Host "Verificando yt-dlp..." -NoNewline
$ytdlp = & $PythonExe -m yt_dlp --version 2>&1
if ($LASTEXITCODE -eq 0) {
    Write-Host " OK ($ytdlp)" -ForegroundColor Green
} else {
    Write-Host " No encontrado. Instalando..." -ForegroundColor Yellow
    & $PythonExe -m pip install yt-dlp --quiet
    if ($LASTEXITCODE -eq 0) {
        Write-Host "  OK  yt-dlp instalado" -ForegroundColor Green
    } else {
        Write-Host "  ERR No se pudo instalar yt-dlp" -ForegroundColor Red
        exit 1
    }
}

# Verificar/instalar requests
$reqCheck = & $PythonExe -c "import requests; print('ok')" 2>&1
if ($reqCheck -ne "ok") {
    Write-Host "Instalando requests..." -NoNewline
    & $PythonExe -m pip install requests --quiet
    Write-Host " OK" -ForegroundColor Green
}

# Construir la accion
$pyArgs = '"' + $ScriptPath + '"'
$action = New-ScheduledTaskAction `
    -Execute $PythonExe `
    -Argument $pyArgs `
    -WorkingDirectory $RootDir

# Triggers: diario + al iniciar sesion (para cargar si esta vacio)
$triggerDaily = New-ScheduledTaskTrigger -Daily -At $TaskTime
$triggerLogon = New-ScheduledTaskTrigger -AtLogOn -User ([System.Security.Principal.WindowsIdentity]::GetCurrent().Name)
$triggers = @($triggerDaily, $triggerLogon)

# Configuracion
$settings = New-ScheduledTaskSettingsSet `
    -ExecutionTimeLimit (New-TimeSpan -Hours 2) `
    -RestartCount 2 `
    -RestartInterval (New-TimeSpan -Minutes 30) `
    -StartWhenAvailable `
    -WakeToRun:$false `
    -MultipleInstances IgnoreNew

# Registrar como usuario actual
$principal = New-ScheduledTaskPrincipal `
    -UserId ([System.Security.Principal.WindowsIdentity]::GetCurrent().Name) `
    -LogonType Interactive `
    -RunLevel Limited

# Eliminar tarea anterior si existe
$existing = Get-ScheduledTask -TaskName $TaskName -ErrorAction SilentlyContinue
if ($existing) {
    Unregister-ScheduledTask -TaskName $TaskName -Confirm:$false
    Write-Host "Tarea anterior eliminada." -ForegroundColor Gray
}

Register-ScheduledTask `
    -TaskName    $TaskName `
    -Action      $action `
    -Trigger     $triggers `
    -Settings    $settings `
    -Principal   $principal `
    -Description "ATLAS Vault Daily Recharge" `
    | Out-Null

if ($?) {
    Write-Host ""
    Write-Host "OK   Tarea registrada correctamente:" -ForegroundColor Green
    Write-Host "     Nombre:   $TaskName"
    Write-Host "     Diario:   $TaskTime"
    Write-Host "     Arranque: tambien corre al iniciar sesion si vault vacio"
    Write-Host ""

    Write-Host "Lanzando primera carga ahora..." -ForegroundColor Yellow
    Start-ScheduledTask -TaskName $TaskName
    Write-Host "OK   Carga inicial en curso en background." -ForegroundColor Green
    Write-Host ""
    Write-Host "Para ver el progreso:" -ForegroundColor Cyan
    Write-Host "  Get-Content '$LogPath' -Wait -Tail 20"
    Write-Host ""
    Write-Host "Otros comandos:" -ForegroundColor Cyan
    Write-Host "  .\scripts\setup_vault_scheduler.ps1 -RunNow"
    Write-Host "  .\scripts\setup_vault_scheduler.ps1 -Status"
    Write-Host "  .\scripts\setup_vault_scheduler.ps1 -Uninstall"
    Write-Host ""
} else {
    Write-Host "ERR  Error al registrar la tarea." -ForegroundColor Red
    exit 1
}

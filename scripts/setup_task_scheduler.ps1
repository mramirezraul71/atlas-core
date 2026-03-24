<#
.SYNOPSIS
    Registra ATLAS-Quant autonomous_loop en Windows Task Scheduler.
    Arranca automaticamente a las 9:15 AM ET cada dia de semana.
    No requiere que nadie ejecute ningun script manualmente.

.EXAMPLE
    # Ejecutar UNA VEZ con privilegios de administrador:
    powershell -ExecutionPolicy Bypass -File scripts\setup_task_scheduler.ps1
#>

$TaskName   = "ATLAS-Quant-AutonomousLoop"
$RepoRoot   = "C:\ATLAS_PUSH"
$Python     = "$RepoRoot\venv\Scripts\python.exe"
$Script     = "$RepoRoot\scripts\autonomous_loop.py"
$LogOut     = "$RepoRoot\logs\autonomous_loop_stdout.log"
$LogErr     = "$RepoRoot\logs\autonomous_loop_stderr.log"

# Crear carpeta logs si no existe
if (-not (Test-Path "$RepoRoot\logs")) {
    New-Item -ItemType Directory -Path "$RepoRoot\logs" -Force | Out-Null
}

# Eliminar tarea existente si hay
Unregister-ScheduledTask -TaskName $TaskName -Confirm:$false -ErrorAction SilentlyContinue

# Accion: ejecutar python con el script
$Action = New-ScheduledTaskAction `
    -Execute $Python `
    -Argument "--  `"$Script`" --interval 120 --max-per-cycle 1" `
    -WorkingDirectory $RepoRoot

# Trigger: lunes a viernes a las 9:15 AM
$Trigger = New-ScheduledTaskTrigger `
    -Weekly `
    -DaysOfWeek Monday,Tuesday,Wednesday,Thursday,Friday `
    -At "09:15AM"

# Configuracion: correr aunque no este logueado, reiniciar si falla
$Settings = New-ScheduledTaskSettingsSet `
    -ExecutionTimeLimit (New-TimeSpan -Hours 8) `
    -RestartCount 3 `
    -RestartInterval (New-TimeSpan -Minutes 2) `
    -MultipleInstances IgnoreNew `
    -StartWhenAvailable

# Registrar con SYSTEM para que corra sin sesion abierta
Register-ScheduledTask `
    -TaskName $TaskName `
    -Action $Action `
    -Trigger $Trigger `
    -Settings $Settings `
    -RunLevel Highest `
    -Force | Out-Null

Write-Host ""
Write-Host "  [OK] Tarea registrada: $TaskName" -ForegroundColor Green
Write-Host "  [OK] Trigger: Lun-Vie 9:15 AM (hora sistema)" -ForegroundColor Green
Write-Host "  [OK] Reintentos: 3x si falla" -ForegroundColor Green
Write-Host ""
Write-Host "  Para verificar:" -ForegroundColor Yellow
Write-Host "    Get-ScheduledTask -TaskName '$TaskName'" -ForegroundColor Yellow
Write-Host "  Para ejecutar manualmente ahora:" -ForegroundColor Yellow
Write-Host "    Start-ScheduledTask -TaskName '$TaskName'" -ForegroundColor Yellow
Write-Host "  Para eliminar:" -ForegroundColor Yellow
Write-Host "    Unregister-ScheduledTask -TaskName '$TaskName' -Confirm:`$false" -ForegroundColor Yellow
Write-Host ""
Write-Host "  NOTA: La hora del trigger es la hora del sistema Windows." -ForegroundColor Cyan
Write-Host "  Mercado ET = UTC-4 (EDT). Verifica que Windows este en ET." -ForegroundColor Cyan

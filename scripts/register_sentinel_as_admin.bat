@echo off
:: Ejecutar este .bat como Administrador para registrar el Sentinel Watchdog como tarea de Windows
echo Registrando ATLAS_Sentinel_Watchdog como tarea programada...
powershell.exe -NoProfile -ExecutionPolicy Bypass -Command "& 'C:\ATLAS_PUSH\scripts\atlas_sentinel_watchdog.ps1' -Register"
echo.
echo Iniciando tarea ahora...
schtasks /Run /TN "ATLAS_Sentinel_Watchdog"
echo.
echo Listo. El watchdog se iniciara automaticamente en cada arranque.
pause

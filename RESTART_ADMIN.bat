@echo off
echo ============================================
echo ATLAS RESTART - Ejecutar como Administrador
echo ============================================

echo Matando proceso ATLAS en puerto 8795...
for /f "tokens=5" %%a in ('netstat -ano ^| findstr :8795 ^| findstr LISTENING') do (
    echo Terminando PID %%a
    taskkill /F /T /PID %%a
)

timeout /t 4 /nobreak

echo Cargando credenciales...
for /f "usebackq tokens=1,2 delims==" %%a in ("C:\dev\credenciales.txt") do (
    set "%%a=%%b"
)

set ATLAS_SANDBOX_RESTRICTED_SYMBOLS=OS
set QUANT_EXIT_GOVERNANCE_ENABLED=false

echo Iniciando ATLAS...
cd /d C:\ATLAS_PUSH
start /B C:\ATLAS_PUSH\venv\Scripts\python.exe C:\ATLAS_PUSH\immediate_start.py

echo ATLAS iniciando en background. Verificar en 30 segundos.
echo Puerto: http://localhost:8795/health
pause

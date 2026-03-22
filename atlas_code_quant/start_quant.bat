@echo off
title Atlas Code-Quant API :8792
cd /d "%~dp0"

:: Venv de ATLAS — tiene numpy, pandas, gymnasium, stable-baselines3, etc.
set VENV_PYTHON=C:\ATLAS_PUSH\venv\Scripts\python.exe
set PYTHONPATH=%~dp0
set QUANT_API_KEY=atlas-quant-local

echo [Atlas Code-Quant] Verificando entorno...
%VENV_PYTHON% -c "import numpy, pandas, gymnasium; print('  OK')" 2>nul
if errorlevel 1 (
    echo [ERROR] Faltan dependencias en el venv.
    echo Ejecuta: cd C:\ATLAS_PUSH ^&^& venv\Scripts\pip install -r atlas_code_quant\requirements.txt
    pause
    exit /b 1
)

echo [Atlas Code-Quant] Iniciando API en puerto 8792...
%VENV_PYTHON% -m uvicorn api.main:app --host 0.0.0.0 --port 8792 --log-level info

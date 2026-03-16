@echo off
title Atlas Code-Quant API :8792
cd /d "%~dp0"
set PYTHONPATH=%~dp0
set QUANT_API_KEY=atlas-quant-local

echo [Atlas Code-Quant] Iniciando API en puerto 8792...
C:\Python314\python.exe -m uvicorn api.main:app --host 0.0.0.0 --port 8792 --log-level info

@echo off
setlocal

cd /d C:\ATLAS_PUSH

REM Market Open Supervisor (paper-safe)
REM Single-instance protegido por mutex dentro del script Python.

C:\ATLAS_PUSH\venv\Scripts\python.exe -m atlas_code_quant.scripts.market_open_supervisor --loop

endlocal

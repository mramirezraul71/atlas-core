@echo off
title ATLAS Telegram Bot Interactivo
color 0A
echo.
echo  ============================================
echo   ATLAS Telegram Bot v2.0
echo   Bot interactivo para control via Telegram
echo  ============================================
echo.

REM Verificar que Python este disponible
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERROR] Python no encontrado en PATH
    echo Instala Python 3.10+ desde https://python.org
    pause
    exit /b 1
)

REM Instalar dependencias si no estan instaladas
echo [INFO] Verificando dependencias...
python -c "import requests" >nul 2>&1
if %errorlevel% neq 0 (
    echo [INFO] Instalando 'requests'...
    pip install requests
)

REM Verificar que MCP server este corriendo
echo [INFO] Verificando MCP server en localhost:8799...
python -c "import requests; r=requests.get('http://localhost:8799',timeout=3); print('[OK] MCP activo')" 2>nul
if %errorlevel% neq 0 (
    echo [WARN] MCP server no detectado en :8799
    echo Asegurate de que atlas_mcp_server.py este corriendo primero.
    echo Puedes iniciarlo con: start_mcp.bat
    echo.
    echo Iniciando bot de todas formas en 5 segundos...
    timeout /t 5 /nobreak >nul
)

echo.
echo [INFO] Iniciando ATLAS Telegram Bot...
echo [INFO] Usa Ctrl+C para detener el bot
echo.

python C:\ATLAS_PUSH\atlas_telegram_bot.py

echo.
echo [INFO] Bot detenido.
pause

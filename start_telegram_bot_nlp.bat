@echo off
title ATLAS Telegram Bot NLP v3.0
color 0B
echo.
echo  ============================================
echo   ATLAS Telegram Bot NLP v3.0
echo   Interaccion en lenguaje natural
echo   Requiere: MCP server + Ollama (opcional)
echo  ============================================
echo.
echo [INFO] Verificando dependencias...
python -c "import requests" >nul 2>&1
if %errorlevel% neq 0 pip install requests

echo [INFO] Iniciando bot conversacional...
echo [INFO] Ctrl+C para detener
echo.
python C:\ATLAS_PUSH\atlas_telegram_bot_nlp.py
pause

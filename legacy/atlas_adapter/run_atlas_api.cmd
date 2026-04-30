@echo off
set "ROOT=%~dp0.."
cd /d "%ROOT%"
if not exist "venv\Scripts\activate.bat" (
  echo Creando venv en %ROOT%...
  python -m venv venv
)
call venv\Scripts\activate.bat
pip install -q fastapi uvicorn pydantic python-dotenv 2>nul
set PYTHONPATH=%CD%
python -m uvicorn atlas_adapter.atlas_http_api:app --host 127.0.0.1 --port 8791

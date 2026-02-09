@echo off
cd /d C:\ATLAS
python -m venv venv_api
call venv_api\Scripts\activate
pip install fastapi uvicorn pydantic
uvicorn atlas_adapter.atlas_http_api:app --host 0.0.0.0 --port 8791

# RUNBOOK: smoke_fastapi_minimal

## Setup
```bash
cd C:\ATLAS_PUSH\_generated\smoke_fastapi_minimal
python -m venv .venv
.venv\Scripts\activate   # Windows
pip install -r requirements.txt
```

## Run
```bash
uvicorn main:app --reload --host 127.0.0.1 --port 8000
```

## Smoke tests
- GET http://127.0.0.1:8000/ -> {"ok": true}
- GET http://127.0.0.1:8000/health -> {"ok": true}

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from fastapi import FastAPI
from fastapi.responses import HTMLResponse, FileResponse
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

from atlas_runtime import handle, status, doctor, modules_report

app = FastAPI(title="ATLAS Bridge API", version="3.1.0")

# Ruta al dashboard
DASHBOARD_PATH = ROOT / "atlas_adapter" / "static" / "dashboard.html"

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/status")
def _status():
    return {"ok": True, "atlas": status()}

@app.get("/tools")
def _tools():
    return {
        "ok": True,
        "tools": [
            {"name": "atlas.status", "path": "/status", "method": "GET"},
            {"name": "atlas.doctor", "path": "/doctor", "method": "GET"},
            {"name": "atlas.modules", "path": "/modules", "method": "GET"},
            {"name": "atlas.run", "path": "/run", "method": "POST"},
        ],
    }

@app.get("/doctor")
def _doctor():
    return {"ok": True, "result": doctor()}

@app.get("/modules")
def _modules():
    return {"ok": True, "result": modules_report()}

@app.post("/run")
def _run(payload: dict):
    text = (payload or {}).get("text", "")
    return {"ok": True, "result": handle(text)}

# ═══════════════════════════════════════════════════════════════
# Dashboard UI
# ═══════════════════════════════════════════════════════════════
@app.get("/ui", response_class=HTMLResponse)
def serve_dashboard():
    """Serve the main ATLAS Dashboard"""
    if DASHBOARD_PATH.exists():
        return HTMLResponse(content=DASHBOARD_PATH.read_text(encoding="utf-8"))
    return HTMLResponse(content="<h1>Dashboard not found</h1><p>Path: " + str(DASHBOARD_PATH) + "</p>", status_code=404)

@app.get("/", response_class=HTMLResponse)
def root_redirect():
    """Redirect root to dashboard"""
    if DASHBOARD_PATH.exists():
        return HTMLResponse(content=DASHBOARD_PATH.read_text(encoding="utf-8"))
    return HTMLResponse(content="<h1>ATLAS API</h1><p><a href='/docs'>API Docs</a></p>")

@app.get("/version")
def get_version():
    """Return current version for update checks"""
    return {
        "ok": True,
        "version": "3.1.0",
        "build_date": "2026-02-16",
        "name": "ATLAS Dashboard"
    }

if __name__ == "__main__":
    uvicorn.run(app, host="127.0.0.1", port=8791)


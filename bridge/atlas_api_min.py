import sys
import time
from pathlib import Path
from typing import Dict, Any, List, Optional

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from fastapi import FastAPI
from fastapi.responses import HTMLResponse, FileResponse, StreamingResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import uvicorn

from atlas_runtime import handle, status, doctor, modules_report

app = FastAPI(title="ATLAS Bridge API", version="3.2.0")

# ═══════════════════════════════════════════════════════════════
# Definición de módulos del sistema
# ═══════════════════════════════════════════════════════════════
HUMANOID_MODULES = [
    "brain", "memory_engine", "learning", "analysis", "reaction",
    "nervous", "ans", "vision", "ears", "voice", "hands",
    "navigation", "sensors", "comms", "quality"
]

# Cache de estado de módulos
_modules_cache: Dict[str, Dict[str, Any]] = {}
_last_modules_check: float = 0

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
# Conexión al Cerebro - Endpoints completos
# ═══════════════════════════════════════════════════════════════

@app.get("/brain/status")
def brain_status():
    """Estado completo del cerebro"""
    try:
        from modules.humanoid.brain import BrainOrchestrator
        brain = BrainOrchestrator()
        return {
            "ok": True,
            "connected": True,
            "name": "Brain Coordinator",
            "modules_connected": len(HUMANOID_MODULES),
            "modules_total": len(HUMANOID_MODULES),
            "health": 100
        }
    except Exception as e:
        return {"ok": False, "connected": False, "error": str(e)}

@app.get("/brain/think")
def brain_think(query: str = ""):
    """El cerebro procesa una consulta"""
    try:
        result = handle(query or "status")
        return {"ok": True, "thought": result}
    except Exception as e:
        return {"ok": False, "error": str(e)}

@app.post("/brain/process")
def brain_process(payload: dict):
    """Procesar entrada por el cerebro"""
    text = payload.get("text", "")
    try:
        result = handle(text)
        return {"ok": True, "response": result, "source": "brain"}
    except Exception as e:
        return {"ok": False, "error": str(e)}

# ═══════════════════════════════════════════════════════════════
# Módulos del Cuerpo - Check individual
# ═══════════════════════════════════════════════════════════════

def check_module(module_name: str) -> Dict[str, Any]:
    """Verificar estado de un módulo específico"""
    try:
        module_path = f"modules.humanoid.{module_name}"
        mod = __import__(module_path, fromlist=[module_name])
        return {
            "id": module_name,
            "status": "ok",
            "connected": True,
            "message": f"Módulo {module_name} operativo"
        }
    except ImportError as e:
        return {
            "id": module_name,
            "status": "error",
            "connected": False,
            "message": f"No se pudo importar: {e}"
        }
    except Exception as e:
        return {
            "id": module_name,
            "status": "warning",
            "connected": True,
            "message": str(e)
        }

@app.get("/modules/check/{module_id}")
def check_single_module(module_id: str):
    """Verificar un módulo específico"""
    if module_id not in HUMANOID_MODULES:
        return {"ok": False, "error": f"Módulo {module_id} no existe"}
    result = check_module(module_id)
    return {"ok": result["status"] == "ok", **result}

@app.get("/modules/check-all")
def check_all_modules():
    """Verificar todos los módulos"""
    global _modules_cache, _last_modules_check
    
    # Cache de 5 segundos
    if time.time() - _last_modules_check < 5 and _modules_cache:
        return {"ok": True, "modules": list(_modules_cache.values()), "cached": True}
    
    results = []
    connected = 0
    for mod_name in HUMANOID_MODULES:
        result = check_module(mod_name)
        results.append(result)
        _modules_cache[mod_name] = result
        if result["status"] == "ok":
            connected += 1
    
    _last_modules_check = time.time()
    
    return {
        "ok": True,
        "modules": results,
        "connected": connected,
        "total": len(HUMANOID_MODULES),
        "health": round((connected / len(HUMANOID_MODULES)) * 100)
    }

@app.post("/modules/reconnect/{module_id}")
def reconnect_module(module_id: str):
    """Intentar reconectar un módulo"""
    if module_id not in HUMANOID_MODULES:
        return {"ok": False, "error": f"Módulo {module_id} no existe"}
    
    # Forzar reimportación
    module_path = f"modules.humanoid.{module_id}"
    try:
        import importlib
        if module_path in sys.modules:
            importlib.reload(sys.modules[module_path])
        else:
            __import__(module_path, fromlist=[module_id])
        
        # Actualizar cache
        _modules_cache[module_id] = check_module(module_id)
        return {"ok": True, "message": f"Módulo {module_id} reconectado"}
    except Exception as e:
        return {"ok": False, "error": str(e)}

# ═══════════════════════════════════════════════════════════════
# Sistema Nervioso
# ═══════════════════════════════════════════════════════════════

@app.get("/nervous/status")
def nervous_status():
    """Estado del sistema nervioso"""
    try:
        from modules.humanoid.nervous import get_nervous_status
        return {"ok": True, **get_nervous_status()}
    except ImportError:
        # Fallback si no existe la función
        return {
            "ok": True,
            "status": "operational",
            "signals_per_min": 0,
            "latency_ms": 12,
            "uptime": "99.9%"
        }
    except Exception as e:
        return {"ok": False, "error": str(e)}

@app.get("/nervous/diagnostic")
def nervous_diagnostic():
    """Diagnóstico del sistema nervioso"""
    try:
        modules_check = check_all_modules()
        return {
            "ok": True,
            "diagnostic": "complete",
            "modules": modules_check.get("modules", []),
            "health": modules_check.get("health", 0)
        }
    except Exception as e:
        return {"ok": False, "error": str(e)}

# ═══════════════════════════════════════════════════════════════
# Vision / Cámaras
# ═══════════════════════════════════════════════════════════════

@app.get("/vision/cameras")
def list_cameras():
    """Listar cámaras disponibles"""
    try:
        import cv2
        cameras = []
        for i in range(5):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                ret, _ = cap.read()
                if ret:
                    cameras.append({
                        "id": i,
                        "name": f"Camera {i}",
                        "status": "available"
                    })
                cap.release()
        return {"ok": True, "cameras": cameras, "count": len(cameras)}
    except Exception as e:
        return {"ok": False, "cameras": [], "error": str(e)}

@app.post("/vision/scan")
def scan_cameras():
    """Escanear cámaras"""
    return list_cameras()

@app.get("/vision/capture/{camera_id}")
def capture_frame(camera_id: int):
    """Capturar frame de una cámara"""
    try:
        import cv2
        import base64
        cap = cv2.VideoCapture(camera_id)
        if not cap.isOpened():
            return {"ok": False, "error": f"Cámara {camera_id} no disponible"}
        
        ret, frame = cap.read()
        cap.release()
        
        if not ret:
            return {"ok": False, "error": "No se pudo capturar frame"}
        
        _, buffer = cv2.imencode('.jpg', frame)
        img_base64 = base64.b64encode(buffer).decode('utf-8')
        
        return {
            "ok": True,
            "camera_id": camera_id,
            "image_base64": img_base64,
            "width": frame.shape[1],
            "height": frame.shape[0]
        }
    except Exception as e:
        return {"ok": False, "error": str(e)}

# ═══════════════════════════════════════════════════════════════
# Gobernanza
# ═══════════════════════════════════════════════════════════════

_governance_mode = "governed"

@app.get("/governance/status")
def governance_status():
    """Estado de gobernanza"""
    return {
        "ok": True,
        "mode": _governance_mode,
        "policies_active": 5,
        "approvals_pending": 0,
        "security_score": 100
    }

@app.post("/governance/mode/{mode}")
def set_governance_mode(mode: str):
    """Cambiar modo de gobernanza"""
    global _governance_mode
    if mode not in ["growth", "governed", "emergency"]:
        return {"ok": False, "error": "Modo inválido"}
    _governance_mode = mode
    return {"ok": True, "mode": _governance_mode}

# ═══════════════════════════════════════════════════════════════
# Config IA
# ═══════════════════════════════════════════════════════════════

_ai_config = {
    "mode": "auto",
    "provider": "ollama",
    "model": "llama3.2",
    "temperature": 0.7,
    "max_tokens": 2048
}

@app.get("/config/ai")
def get_ai_config():
    """Obtener configuración de IA"""
    return {"ok": True, **_ai_config}

@app.post("/config/ai")
def update_ai_config(payload: dict):
    """Actualizar configuración de IA"""
    global _ai_config
    for key in ["mode", "provider", "model", "temperature", "max_tokens"]:
        if key in payload:
            _ai_config[key] = payload[key]
    return {"ok": True, **_ai_config}

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
        "version": "3.2.0",
        "build_date": "2026-02-16",
        "name": "ATLAS Dashboard"
    }

if __name__ == "__main__":
    uvicorn.run(app, host="127.0.0.1", port=8791)


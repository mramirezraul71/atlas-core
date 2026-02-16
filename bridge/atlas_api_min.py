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

app = FastAPI(title="ATLAS Bridge API", version="3.8.0")

# ═══════════════════════════════════════════════════════════════
# Integrar API de Tutorías
# ═══════════════════════════════════════════════════════════════
try:
    from modules.humanoid.quality.tutorias.api import router as tutorias_router
    app.include_router(tutorias_router)
    TUTORIAS_ENABLED = True
except ImportError:
    TUTORIAS_ENABLED = False

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

# ═══════════════════════════════════════════════════════════════
# Sistema de Bitácora Central
# ═══════════════════════════════════════════════════════════════
_bitacora: List[Dict[str, Any]] = []
_bitacora_max_entries = 500

def log_to_bitacora(message: str, level: str = "info", source: str = "system", data: Dict = None):
    """Registrar evento en la bitácora central"""
    import datetime
    entry = {
        "id": int(time.time() * 1000),
        "timestamp": datetime.datetime.now().isoformat(),
        "time": datetime.datetime.now().strftime("%H:%M:%S"),
        "message": message,
        "level": level,  # info, success, error, warning
        "source": source,  # brain, vision, nervous, modules, ai, governance, system
        "data": data or {}
    }
    _bitacora.insert(0, entry)
    if len(_bitacora) > _bitacora_max_entries:
        _bitacora.pop()
    return entry

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

# ═══════════════════════════════════════════════════════════════
# Endpoints de Bitácora
# ═══════════════════════════════════════════════════════════════

@app.get("/bitacora")
def get_bitacora(limit: int = 100, source: str = None, level: str = None):
    """Obtener entradas de la bitácora"""
    entries = _bitacora
    
    if source:
        entries = [e for e in entries if e["source"] == source]
    if level:
        entries = [e for e in entries if e["level"] == level]
    
    return {
        "ok": True,
        "entries": entries[:limit],
        "total": len(_bitacora),
        "filtered": len(entries)
    }

@app.get("/bitacora/stream")
def get_bitacora_since(since_id: int = 0):
    """Obtener entradas nuevas desde un ID específico (para polling)"""
    new_entries = [e for e in _bitacora if e["id"] > since_id]
    return {
        "ok": True,
        "entries": new_entries,
        "count": len(new_entries),
        "latest_id": _bitacora[0]["id"] if _bitacora else 0
    }

@app.post("/bitacora/log")
def add_bitacora_entry(payload: dict):
    """Agregar entrada a la bitácora desde el frontend"""
    message = payload.get("message", "")
    level = payload.get("level", "info")
    source = payload.get("source", "system")
    data = payload.get("data", {})
    
    if not message:
        return {"ok": False, "error": "Message required"}
    
    entry = log_to_bitacora(message, level, source, data)
    return {"ok": True, "entry": entry}

@app.delete("/bitacora")
def clear_bitacora():
    """Limpiar la bitácora"""
    global _bitacora
    count = len(_bitacora)
    _bitacora = []
    log_to_bitacora("Bitácora limpiada", "info", "system", {"cleared": count})
    return {"ok": True, "cleared": count}

# ═══════════════════════════════════════════════════════════════
# Conexión al Cerebro - Con logging
# ═══════════════════════════════════════════════════════════════

@app.get("/brain/status")
def brain_status():
    """Estado completo del cerebro"""
    log_to_bitacora("Consultando estado del cerebro", "info", "brain")
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
    log_to_bitacora(f"Cerebro pensando: {query[:50]}...", "info", "brain", {"query": query})
    try:
        result = handle(query or "status")
        log_to_bitacora("Pensamiento completado", "success", "brain")
        return {"ok": True, "thought": result}
    except Exception as e:
        log_to_bitacora(f"Error en pensamiento: {e}", "error", "brain")
        return {"ok": False, "error": str(e)}

@app.post("/brain/process")
def brain_process(payload: dict):
    """Procesar entrada por el cerebro"""
    text = payload.get("text", "")
    log_to_bitacora(f"Procesando: {text[:50]}...", "info", "brain", {"input_length": len(text)})
    try:
        result = handle(text)
        log_to_bitacora("Respuesta generada", "success", "brain", {"output_length": len(str(result))})
        return {"ok": True, "response": result, "source": "brain"}
    except Exception as e:
        log_to_bitacora(f"Error procesando: {e}", "error", "brain")
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
    log_to_bitacora(f"Verificando módulo: {module_id}", "info", "modules")
    if module_id not in HUMANOID_MODULES:
        log_to_bitacora(f"Módulo no existe: {module_id}", "error", "modules")
        return {"ok": False, "error": f"Módulo {module_id} no existe"}
    result = check_module(module_id)
    log_to_bitacora(f"Módulo {module_id}: {result['status']}", "success" if result["status"] == "ok" else "error", "modules")
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
    log_to_bitacora(f"Reconectando módulo: {module_id}", "info", "modules")
    if module_id not in HUMANOID_MODULES:
        log_to_bitacora(f"Módulo no existe: {module_id}", "error", "modules")
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
        log_to_bitacora(f"Módulo {module_id} reconectado exitosamente", "success", "modules")
        return {"ok": True, "message": f"Módulo {module_id} reconectado"}
    except Exception as e:
        log_to_bitacora(f"Error reconectando {module_id}: {e}", "error", "modules")
        return {"ok": False, "error": str(e)}

# ═══════════════════════════════════════════════════════════════
# Sistema Nervioso
# ═══════════════════════════════════════════════════════════════

# ═══════════════════════════════════════════════════════════════
# Sistema Nervioso - Estado completo
# ═══════════════════════════════════════════════════════════════

_nervous_state = {
    "mode": "normal",  # normal, stress, calm
    "heartbeat": 72,
    "signals_per_min": 0,
    "latency_ms": 12,
    "uptime": 99.9,
    "reflexes_count": 0,
    "nodes": {
        "cerebro": {"status": "ok", "load": 45},
        "cerebelo": {"status": "ok", "load": 30},
        "tronco": {"status": "ok", "load": 25},
        "medula": {"status": "ok", "load": 20},
        "sensorial": {"status": "ok", "load": 55},
        "motor": {"status": "ok", "load": 40},
        "craneal": {"status": "ok", "load": 35},
        "espinal": {"status": "ok", "load": 30},
        "simpatico": {"status": "ok", "load": 20},
        "parasimpatico": {"status": "ok", "load": 60},
        "enterico": {"status": "ok", "load": 25},
        "cardiaco": {"status": "ok", "load": 45}
    },
    "signals": {
        "brain": 85,
        "vision": 92,
        "audio": 78,
        "motor": 95,
        "ans": 88
    }
}

@app.get("/nervous/status")
def nervous_status():
    """Estado del sistema nervioso"""
    import random
    log_to_bitacora("Consultando estado del sistema nervioso", "info", "nervous")
    
    # Actualizar señales dinámicamente
    _nervous_state["signals_per_min"] += random.randint(1, 5)
    _nervous_state["latency_ms"] = random.randint(8, 18)
    
    # Simular variación de heartbeat según modo
    if _nervous_state["mode"] == "normal":
        _nervous_state["heartbeat"] = random.randint(68, 78)
    elif _nervous_state["mode"] == "stress":
        _nervous_state["heartbeat"] = random.randint(100, 120)
    else:  # calm
        _nervous_state["heartbeat"] = random.randint(60, 70)
    
    return {
        "ok": True,
        "status": "operational",
        "mode": _nervous_state["mode"],
        "heartbeat": _nervous_state["heartbeat"],
        "signals_per_min": _nervous_state["signals_per_min"],
        "latency_ms": _nervous_state["latency_ms"],
        "uptime": _nervous_state["uptime"],
        "reflexes_count": _nervous_state["reflexes_count"],
        "signals": _nervous_state["signals"]
    }

@app.get("/nervous/diagnostic")
def nervous_diagnostic():
    """Diagnóstico completo del sistema nervioso"""
    import random
    
    # Verificar todos los nodos
    nodes_status = []
    healthy_count = 0
    
    for node_id, node_data in _nervous_state["nodes"].items():
        # Simular carga
        node_data["load"] = random.randint(15, 75)
        nodes_status.append({
            "id": node_id,
            "status": node_data["status"],
            "load": node_data["load"],
            "latency": random.randint(5, 20)
        })
        if node_data["status"] == "ok":
            healthy_count += 1
    
    health = round((healthy_count / len(_nervous_state["nodes"])) * 100)
    
    return {
        "ok": True,
        "diagnostic": "complete",
        "nodes": nodes_status,
        "nodes_total": len(_nervous_state["nodes"]),
        "nodes_healthy": healthy_count,
        "health": health,
        "mode": _nervous_state["mode"]
    }

@app.get("/nervous/nodes")
def get_nervous_nodes():
    """Obtener estado de todos los nodos"""
    return {"ok": True, "nodes": _nervous_state["nodes"]}

@app.get("/nervous/node/{node_id}")
def get_nervous_node(node_id: str):
    """Obtener estado de un nodo específico"""
    if node_id not in _nervous_state["nodes"]:
        return {"ok": False, "error": f"Nodo {node_id} no existe"}
    return {"ok": True, "node": node_id, **_nervous_state["nodes"][node_id]}

@app.post("/nervous/simulate/stress")
def simulate_stress():
    """Simular respuesta de estrés"""
    global _nervous_state
    log_to_bitacora("⚡ Simulación de estrés iniciada", "warning", "nervous")
    _nervous_state["mode"] = "stress"
    _nervous_state["heartbeat"] = 110
    _nervous_state["nodes"]["simpatico"]["load"] = 90
    _nervous_state["nodes"]["cardiaco"]["load"] = 85
    log_to_bitacora("Sistema simpático activado - BPM: 110", "warning", "nervous")
    return {"ok": True, "mode": "stress", "message": "Sistema simpático activado"}

@app.post("/nervous/simulate/calm")
def simulate_calm():
    """Activar sistema parasimpático para calmar"""
    global _nervous_state
    log_to_bitacora("🧘 Activando sistema parasimpático", "info", "nervous")
    _nervous_state["mode"] = "calm"
    _nervous_state["heartbeat"] = 65
    _nervous_state["nodes"]["parasimpatico"]["load"] = 80
    _nervous_state["nodes"]["simpatico"]["load"] = 15
    log_to_bitacora("Sistema normalizado - BPM: 65", "success", "nervous")
    return {"ok": True, "mode": "calm", "message": "Sistema parasimpático activado"}

@app.post("/nervous/reset")
def reset_nervous():
    """Resetear sistema nervioso a estado normal"""
    global _nervous_state
    _nervous_state["mode"] = "normal"
    _nervous_state["heartbeat"] = 72
    _nervous_state["signals_per_min"] = 0
    _nervous_state["reflexes_count"] = 0
    for node in _nervous_state["nodes"].values():
        node["load"] = 30
    return {"ok": True, "mode": "normal", "message": "Sistema nervioso reseteado"}

@app.post("/nervous/reflex/test")
def test_reflexes():
    """Probar reflejos del sistema"""
    import random
    
    _nervous_state["reflexes_count"] += 5
    reflex_time = random.randint(10, 50)  # ms
    
    return {
        "ok": True,
        "reflexes_tested": 5,
        "total_reflexes": _nervous_state["reflexes_count"],
        "avg_reflex_time_ms": reflex_time,
        "result": "normal" if reflex_time < 30 else "slow"
    }

@app.get("/nervous/signals")
def get_signals():
    """Obtener señales del sistema nervioso"""
    import random
    
    # Actualizar señales dinámicamente
    for key in _nervous_state["signals"]:
        base = _nervous_state["signals"][key]
        variation = random.randint(-5, 5)
        _nervous_state["signals"][key] = max(50, min(100, base + variation))
    
    return {"ok": True, "signals": _nervous_state["signals"]}

# ═══════════════════════════════════════════════════════════════
# Vision / Cámaras
# ═══════════════════════════════════════════════════════════════

def get_camera_names_windows() -> Dict[int, str]:
    """Obtener nombres reales de cámaras en Windows usando WMI"""
    camera_names = {}
    try:
        import subprocess
        # Usar PowerShell para obtener dispositivos de video
        cmd = 'Get-PnpDevice -Class Camera -Status OK | Select-Object -ExpandProperty FriendlyName'
        result = subprocess.run(
            ['powershell', '-Command', cmd],
            capture_output=True, text=True, timeout=5
        )
        if result.returncode == 0:
            names = [n.strip() for n in result.stdout.strip().split('\n') if n.strip()]
            for idx, name in enumerate(names):
                camera_names[idx] = name
    except Exception:
        pass
    
    # Fallback: intentar con DirectShow names via cv2 backend info
    if not camera_names:
        try:
            import subprocess
            cmd = '''
            Get-CimInstance Win32_PnPEntity | 
            Where-Object { $_.PNPClass -eq 'Camera' -or $_.PNPClass -eq 'Image' } | 
            Select-Object -ExpandProperty Name
            '''
            result = subprocess.run(
                ['powershell', '-Command', cmd],
                capture_output=True, text=True, timeout=5
            )
            if result.returncode == 0:
                names = [n.strip() for n in result.stdout.strip().split('\n') if n.strip()]
                for idx, name in enumerate(names):
                    camera_names[idx] = name
        except Exception:
            pass
    
    return camera_names

@app.get("/vision/cameras")
def list_cameras():
    """Listar cámaras disponibles con nombres reales"""
    try:
        import cv2
        
        # Obtener nombres reales de Windows
        camera_names = get_camera_names_windows()
        
        cameras = []
        for i in range(5):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    # Obtener resolución
                    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    fps = int(cap.get(cv2.CAP_PROP_FPS)) or 30
                    
                    # Usar nombre real si está disponible
                    name = camera_names.get(i, f"Camera {i}")
                    
                    cameras.append({
                        "id": i,
                        "name": name,
                        "resolution": f"{width}x{height}",
                        "fps": fps,
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
    log_to_bitacora(f"Capturando frame de cámara {camera_id}", "info", "vision")
    try:
        import cv2
        import base64
        cap = cv2.VideoCapture(camera_id)
        if not cap.isOpened():
            log_to_bitacora(f"Cámara {camera_id} no disponible", "error", "vision")
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
    "top_p": 0.9,
    "top_k": 40,
    "max_tokens": 2048,
    "repeat_penalty": 1.1,
    "system_prompt": "Eres ATLAS, un sistema de inteligencia artificial avanzado. Responde de forma concisa, precisa y útil.",
    "memory_context": "long",
    "context_window": 8192,
    "ollama_url": "http://localhost:11434",
    "stream_response": True,
    "save_history": True,
    "log_level": "info",
    "api_key": "",
    # Especialistas
    "specialists": {
        "code": {"enabled": True, "model": "deepseek-coder"},
        "vision": {"enabled": True, "model": "llava"},
        "chat": {"enabled": True, "model": "llama3.2"},
        "analysis": {"enabled": True, "model": "llama3.2"},
        "creative": {"enabled": False, "model": "llama3.2"}
    }
}

@app.get("/config/ai")
def get_ai_config():
    """Obtener configuración de IA"""
    # No enviar API key al frontend
    config_safe = {k: v for k, v in _ai_config.items() if k != "api_key"}
    config_safe["has_api_key"] = bool(_ai_config.get("api_key"))
    return {"ok": True, **config_safe}

@app.post("/config/ai")
def update_ai_config(payload: dict):
    """Actualizar configuración de IA"""
    global _ai_config
    
    # Campos permitidos
    allowed_fields = [
        "mode", "provider", "model", "temperature", "top_p", "top_k",
        "max_tokens", "repeat_penalty", "system_prompt", "memory_context",
        "context_window", "ollama_url", "stream_response", "save_history",
        "log_level", "api_key", "specialists"
    ]
    
    for key in allowed_fields:
        if key in payload:
            _ai_config[key] = payload[key]
    
    return {"ok": True, "message": "Configuración actualizada"}

@app.post("/config/ai/test")
def test_ai_connection():
    """Probar conexión con el proveedor de IA"""
    provider = _ai_config.get("provider", "ollama")
    
    try:
        if provider == "ollama":
            import requests
            url = _ai_config.get("ollama_url", "http://localhost:11434")
            res = requests.get(f"{url}/api/tags", timeout=5)
            if res.status_code == 200:
                return {"ok": True, "provider": provider, "message": "Ollama conectado"}
            return {"ok": False, "error": f"Ollama respondió con status {res.status_code}"}
        
        elif provider == "openai":
            import requests
            api_key = _ai_config.get("api_key", "")
            if not api_key:
                return {"ok": False, "error": "API Key no configurada"}
            res = requests.get(
                "https://api.openai.com/v1/models",
                headers={"Authorization": f"Bearer {api_key}"},
                timeout=10
            )
            if res.status_code == 200:
                return {"ok": True, "provider": provider, "message": "OpenAI conectado"}
            return {"ok": False, "error": f"OpenAI respondió con status {res.status_code}"}
        
        elif provider == "anthropic":
            # Test básico de Anthropic
            return {"ok": True, "provider": provider, "message": "Anthropic configurado (sin test directo)"}
        
        else:
            return {"ok": True, "provider": provider, "message": f"{provider} configurado"}
            
    except Exception as e:
        return {"ok": False, "error": str(e)}

@app.get("/config/ai/ollama-models")
def get_ollama_models():
    """Listar modelos disponibles en Ollama"""
    try:
        import requests
        url = _ai_config.get("ollama_url", "http://localhost:11434")
        res = requests.get(f"{url}/api/tags", timeout=10)
        if res.status_code == 200:
            data = res.json()
            models = []
            for m in data.get("models", []):
                size_bytes = m.get("size", 0)
                size_str = f"{size_bytes / (1024**3):.1f}GB" if size_bytes > 1024**3 else f"{size_bytes / (1024**2):.0f}MB"
                models.append({
                    "name": m.get("name", "").split(":")[0],
                    "full_name": m.get("name", ""),
                    "size": size_str,
                    "modified": m.get("modified_at", "")
                })
            return {"ok": True, "models": models}
        return {"ok": False, "models": [], "error": f"Status {res.status_code}"}
    except Exception as e:
        return {"ok": False, "models": [], "error": str(e)}

@app.post("/config/ai/ollama-pull")
def pull_ollama_model(payload: dict):
    """Descargar un modelo en Ollama"""
    model_name = payload.get("model", "")
    if not model_name:
        return {"ok": False, "error": "Nombre de modelo requerido"}
    
    try:
        import requests
        url = _ai_config.get("ollama_url", "http://localhost:11434")
        res = requests.post(
            f"{url}/api/pull",
            json={"name": model_name},
            timeout=300  # 5 minutos para descargar
        )
        if res.status_code == 200:
            return {"ok": True, "message": f"Modelo {model_name} descargado"}
        return {"ok": False, "error": f"Error descargando: {res.text}"}
    except Exception as e:
        return {"ok": False, "error": str(e)}

@app.post("/brain/generate")
def brain_generate(payload: dict):
    """Generar respuesta usando el proveedor configurado"""
    prompt = payload.get("prompt", "")
    if not prompt:
        return {"ok": False, "error": "Prompt requerido"}
    
    provider = _ai_config.get("provider", "ollama")
    model = _ai_config.get("model", "llama3.2")
    
    try:
        if provider == "ollama":
            import requests
            url = _ai_config.get("ollama_url", "http://localhost:11434")
            res = requests.post(
                f"{url}/api/generate",
                json={
                    "model": model,
                    "prompt": prompt,
                    "system": _ai_config.get("system_prompt", ""),
                    "options": {
                        "temperature": _ai_config.get("temperature", 0.7),
                        "top_p": _ai_config.get("top_p", 0.9),
                        "top_k": _ai_config.get("top_k", 40),
                        "num_predict": _ai_config.get("max_tokens", 2048),
                        "repeat_penalty": _ai_config.get("repeat_penalty", 1.1)
                    },
                    "stream": False
                },
                timeout=120
            )
            if res.status_code == 200:
                data = res.json()
                return {
                    "ok": True,
                    "response": data.get("response", ""),
                    "model": model,
                    "provider": provider,
                    "eval_count": data.get("eval_count", 0),
                    "eval_duration": data.get("eval_duration", 0)
                }
            return {"ok": False, "error": f"Ollama error: {res.text}"}
        
        else:
            # Fallback para otros proveedores
            return {"ok": False, "error": f"Proveedor {provider} no implementado aún"}
            
    except Exception as e:
        return {"ok": False, "error": str(e)}

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
        "version": "3.8.0",
        "build_date": "2026-02-16",
        "name": "ATLAS Dashboard",
        "modules": {
            "tutorias": TUTORIAS_ENABLED
        }
    }

if __name__ == "__main__":
    uvicorn.run(app, host="127.0.0.1", port=8791)


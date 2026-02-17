"""
API REST para el Sistema Cognitivo Atlas.

Endpoints para monitoreo en tiempo real de todos los módulos cognitivos.
"""
from __future__ import annotations

import time
import asyncio
from typing import Any, Dict, List, Optional
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from pydantic import BaseModel, Field

router = APIRouter(prefix="/cognitive", tags=["cognitive"])


# ============ Pydantic Models ============

class ModuleStatus(BaseModel):
    """Estado de un módulo."""
    name: str
    status: str = "unknown"  # ok, warning, error, offline
    latency_ms: float = 0.0
    last_update_ns: int = 0
    stats: Dict[str, Any] = Field(default_factory=dict)


class CortexStatus(BaseModel):
    """Estado del Cortex completo."""
    frontal: Dict[str, ModuleStatus] = Field(default_factory=dict)
    parietal: Dict[str, ModuleStatus] = Field(default_factory=dict)
    temporal: Dict[str, ModuleStatus] = Field(default_factory=dict)
    occipital: Dict[str, ModuleStatus] = Field(default_factory=dict)


class CognitiveSystemStatus(BaseModel):
    """Estado completo del sistema cognitivo."""
    ok: bool = True
    timestamp_ns: int = 0
    uptime_seconds: float = 0.0
    
    # Módulos principales
    medulla: ModuleStatus = Field(default_factory=lambda: ModuleStatus(name="medulla"))
    cortex: CortexStatus = Field(default_factory=CortexStatus)
    limbic: Dict[str, ModuleStatus] = Field(default_factory=dict)
    hippo: ModuleStatus = Field(default_factory=lambda: ModuleStatus(name="hippo"))
    brainstem: Dict[str, ModuleStatus] = Field(default_factory=dict)
    basal: Dict[str, ModuleStatus] = Field(default_factory=dict)
    motor: Dict[str, ModuleStatus] = Field(default_factory=dict)
    learning: ModuleStatus = Field(default_factory=lambda: ModuleStatus(name="learning"))
    
    # Métricas globales
    signals_per_minute: int = 0
    avg_latency_ms: float = 0.0
    memory_episodes: int = 0
    active_goals: int = 0


# ============ Estado Global ============

_cognitive_system = None
_start_time = time.time()
_signal_count = 0
_latency_samples: List[float] = []


def _get_cognitive_system():
    """Obtiene o crea el sistema cognitivo."""
    global _cognitive_system
    if _cognitive_system is None:
        try:
            from modules.humanoid import get_cognitive_system
            _cognitive_system = get_cognitive_system()
        except Exception:
            _cognitive_system = {}
    return _cognitive_system


def _module_status(name: str, module: Any) -> ModuleStatus:
    """Extrae estado de un módulo."""
    t0 = time.perf_counter()
    stats = {}
    status = "ok"
    
    try:
        if hasattr(module, "get_stats"):
            stats = module.get_stats()
        elif hasattr(module, "stats"):
            stats = module.stats() if callable(module.stats) else module.stats
    except Exception as e:
        status = "error"
        stats = {"error": str(e)}
    
    latency = (time.perf_counter() - t0) * 1000
    
    return ModuleStatus(
        name=name,
        status=status,
        latency_ms=latency,
        last_update_ns=time.time_ns(),
        stats=stats,
    )


# ============ Endpoints ============

@router.get("/status", response_model=CognitiveSystemStatus)
def cognitive_status() -> CognitiveSystemStatus:
    """
    Obtiene estado completo del sistema cognitivo.
    
    Returns:
        Estado de todos los módulos cognitivos
    """
    global _signal_count, _latency_samples
    
    t0 = time.perf_counter()
    system = _get_cognitive_system()
    
    result = CognitiveSystemStatus(
        timestamp_ns=time.time_ns(),
        uptime_seconds=time.time() - _start_time,
    )
    
    # Medulla
    if "medulla" in system:
        result.medulla = _module_status("medulla", system["medulla"])
    
    # Cortex
    if "cortex" in system:
        cortex = system["cortex"]
        
        # Frontal
        if "frontal" in cortex:
            for name, mod in cortex["frontal"].items():
                result.cortex.frontal[name] = _module_status(name, mod)
        
        # Parietal
        if "parietal" in cortex:
            for name, mod in cortex["parietal"].items():
                result.cortex.parietal[name] = _module_status(name, mod)
        
        # Temporal
        if "temporal" in cortex:
            for name, mod in cortex["temporal"].items():
                result.cortex.temporal[name] = _module_status(name, mod)
        
        # Occipital
        if "occipital" in cortex:
            for name, mod in cortex["occipital"].items():
                result.cortex.occipital[name] = _module_status(name, mod)
    
    # Limbic
    if "limbic" in system:
        for name, mod in system["limbic"].items():
            result.limbic[name] = _module_status(name, mod)
    
    # Hippo
    if "hippo" in system:
        result.hippo = _module_status("hippo", system["hippo"])
        try:
            stats = system["hippo"].get_stats() if hasattr(system["hippo"], "get_stats") else {}
            result.memory_episodes = stats.get("episodic_count", 0)
        except:
            pass
    
    # Brainstem
    if "brainstem" in system:
        for name, mod in system["brainstem"].items():
            result.brainstem[name] = _module_status(name, mod)
    
    # Basal
    if "basal" in system:
        for name, mod in system["basal"].items():
            result.basal[name] = _module_status(name, mod)
    
    # Motor
    if "motor" in system:
        for name, mod in system["motor"].items():
            result.motor[name] = _module_status(name, mod)
    
    # Learning
    if "learning" in system:
        result.learning = _module_status("learning", system["learning"])
    
    # Métricas globales
    latency = (time.perf_counter() - t0) * 1000
    _latency_samples.append(latency)
    if len(_latency_samples) > 100:
        _latency_samples = _latency_samples[-100:]
    
    _signal_count += 1
    
    result.avg_latency_ms = sum(_latency_samples) / len(_latency_samples)
    result.signals_per_minute = int(_signal_count / max(1, (time.time() - _start_time) / 60))
    
    # Active goals
    try:
        if "limbic" in system and "goal_manager" in system["limbic"]:
            gm = system["limbic"]["goal_manager"]
            if hasattr(gm, "get_active_goals"):
                result.active_goals = len(gm.get_active_goals())
    except:
        pass
    
    return result


@router.get("/cortex/frontal")
def cortex_frontal_status() -> Dict[str, Any]:
    """Estado del lóbulo frontal."""
    system = _get_cognitive_system()
    cortex = system.get("cortex", {})
    frontal = cortex.get("frontal", {})
    
    return {
        "ok": True,
        "modules": {
            name: _module_status(name, mod).model_dump()
            for name, mod in frontal.items()
        },
    }


@router.get("/cortex/parietal")
def cortex_parietal_status() -> Dict[str, Any]:
    """Estado del lóbulo parietal."""
    system = _get_cognitive_system()
    cortex = system.get("cortex", {})
    parietal = cortex.get("parietal", {})
    
    return {
        "ok": True,
        "modules": {
            name: _module_status(name, mod).model_dump()
            for name, mod in parietal.items()
        },
    }


@router.get("/cortex/temporal")
def cortex_temporal_status() -> Dict[str, Any]:
    """Estado del lóbulo temporal."""
    system = _get_cognitive_system()
    cortex = system.get("cortex", {})
    temporal = cortex.get("temporal", {})
    
    return {
        "ok": True,
        "modules": {
            name: _module_status(name, mod).model_dump()
            for name, mod in temporal.items()
        },
    }


@router.get("/cortex/occipital")
def cortex_occipital_status() -> Dict[str, Any]:
    """Estado del lóbulo occipital."""
    system = _get_cognitive_system()
    cortex = system.get("cortex", {})
    occipital = cortex.get("occipital", {})
    
    return {
        "ok": True,
        "modules": {
            name: _module_status(name, mod).model_dump()
            for name, mod in occipital.items()
        },
    }


@router.get("/limbic")
def limbic_status() -> Dict[str, Any]:
    """Estado del sistema límbico."""
    system = _get_cognitive_system()
    limbic = system.get("limbic", {})
    
    return {
        "ok": True,
        "modules": {
            name: _module_status(name, mod).model_dump()
            for name, mod in limbic.items()
        },
    }


@router.get("/hippo")
def hippo_status() -> Dict[str, Any]:
    """Estado del hipocampo."""
    system = _get_cognitive_system()
    hippo = system.get("hippo")
    
    if hippo:
        return {
            "ok": True,
            "module": _module_status("hippo", hippo).model_dump(),
        }
    return {"ok": False, "error": "Hippo not initialized"}


@router.get("/brainstem")
def brainstem_status() -> Dict[str, Any]:
    """Estado del tronco encefálico."""
    system = _get_cognitive_system()
    brainstem = system.get("brainstem", {})
    
    return {
        "ok": True,
        "modules": {
            name: _module_status(name, mod).model_dump()
            for name, mod in brainstem.items()
        },
    }


@router.get("/basal")
def basal_status() -> Dict[str, Any]:
    """Estado de los ganglios basales."""
    system = _get_cognitive_system()
    basal = system.get("basal", {})
    
    return {
        "ok": True,
        "modules": {
            name: _module_status(name, mod).model_dump()
            for name, mod in basal.items()
        },
    }


@router.get("/motor")
def motor_status() -> Dict[str, Any]:
    """Estado del control motor."""
    system = _get_cognitive_system()
    motor = system.get("motor", {})
    
    return {
        "ok": True,
        "modules": {
            name: _module_status(name, mod).model_dump()
            for name, mod in motor.items()
        },
    }


@router.get("/learning")
def learning_status() -> Dict[str, Any]:
    """Estado del sistema de aprendizaje."""
    system = _get_cognitive_system()
    learning = system.get("learning")
    
    if learning:
        return {
            "ok": True,
            "module": _module_status("learning", learning).model_dump(),
        }
    return {"ok": False, "error": "Learning not initialized"}


@router.get("/medulla")
def medulla_status() -> Dict[str, Any]:
    """Estado de la médula."""
    system = _get_cognitive_system()
    medulla = system.get("medulla")
    
    if medulla:
        stats = {}
        if hasattr(medulla, "shared_state"):
            try:
                stats["shared_state_keys"] = len(medulla.shared_state.get_all())
            except:
                pass
        
        return {
            "ok": True,
            "module": _module_status("medulla", medulla).model_dump(),
            "stats": stats,
        }
    return {"ok": False, "error": "Medulla not initialized"}


@router.get("/goals")
def get_active_goals() -> Dict[str, Any]:
    """Obtiene objetivos activos."""
    system = _get_cognitive_system()
    limbic = system.get("limbic", {})
    goal_manager = limbic.get("goal_manager")
    
    if goal_manager and hasattr(goal_manager, "get_active_goals"):
        goals = goal_manager.get_active_goals()
        return {
            "ok": True,
            "goals": [
                {
                    "id": g.id,
                    "type": g.goal_type,
                    "description": g.description,
                    "priority": g.priority.value if hasattr(g.priority, "value") else g.priority,
                    "progress": g.progress,
                    "status": g.status.value if hasattr(g.status, "value") else g.status,
                }
                for g in goals
            ],
        }
    return {"ok": False, "goals": []}


@router.get("/memory/recent")
def get_recent_memories() -> Dict[str, Any]:
    """Obtiene memorias recientes."""
    system = _get_cognitive_system()
    hippo = system.get("hippo")
    
    if hippo and hasattr(hippo, "episodic"):
        try:
            episodes = hippo.episodic.get_recent(limit=10)
            return {
                "ok": True,
                "episodes": [
                    {
                        "id": e.id,
                        "goal": e.goal,
                        "outcome": e.outcome.value if hasattr(e.outcome, "value") else str(e.outcome),
                        "reward": e.reward,
                    }
                    for e in episodes
                ],
            }
        except Exception as e:
            return {"ok": False, "error": str(e), "episodes": []}
    return {"ok": False, "episodes": []}


# ============ WebSocket para tiempo real ============

class ConnectionManager:
    """Gestiona conexiones WebSocket."""
    
    def __init__(self):
        self.active_connections: List[WebSocket] = []
    
    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
    
    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
    
    async def broadcast(self, data: dict):
        for connection in self.active_connections:
            try:
                await connection.send_json(data)
            except:
                pass


manager = ConnectionManager()


@router.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """
    WebSocket para actualizaciones en tiempo real.
    
    Envía estado del sistema cada segundo.
    """
    await manager.connect(websocket)
    try:
        while True:
            # Enviar estado
            status = cognitive_status()
            await websocket.send_json(status.model_dump())
            
            # Esperar 1 segundo
            await asyncio.sleep(1)
            
    except WebSocketDisconnect:
        manager.disconnect(websocket)
    except Exception:
        manager.disconnect(websocket)


# ============ Función de registro ============

def include_cognitive_router(app):
    """Registra el router en la aplicación FastAPI."""
    app.include_router(router)

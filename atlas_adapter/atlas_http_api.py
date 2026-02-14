"""ATLAS HTTP API adapter
Expone /status /tools /execute usando el command_router.handle de C:\ATLAS\modules\command_router.py
"""
import os
from pathlib import Path

# Cargar config ANTES de importar módulos que usan os.getenv (audit, policy, etc.)
BASE_DIR = Path(__file__).resolve().parent.parent
ENV_PATH = BASE_DIR / "config" / "atlas.env"
if ENV_PATH.exists():
    from dotenv import load_dotenv
    load_dotenv(ENV_PATH, override=True)
else:
    import logging
    logging.warning("atlas.env not found at %s", ENV_PATH)

_logs = str(BASE_DIR / "logs")


def _ensure_db_path(env_var: str, filename: str) -> None:
    """Force env var to writable path under project logs."""
    p = Path(os.getenv(env_var) or str(Path(_logs) / filename))
    try:
        p.parent.mkdir(parents=True, exist_ok=True)
        p.touch()
    except Exception:
        p = Path(_logs) / filename
        p.parent.mkdir(parents=True, exist_ok=True)
        try:
            p.touch()
        except Exception:
            pass
    os.environ[env_var] = str(p)


Path(_logs).mkdir(parents=True, exist_ok=True)
_ensure_db_path("SCHED_DB_PATH", "atlas_sched.sqlite")
_ensure_db_path("ATLAS_MEMORY_DB_PATH", "atlas_memory.sqlite")
_ensure_db_path("MEMORY_DB_PATH", "atlas_memory.sqlite")
_ensure_db_path("AUDIT_DB_PATH", "atlas_audit.sqlite")

from typing import List, Optional

from fastapi import FastAPI, Request, Header, WebSocket, File, UploadFile
from fastapi.responses import FileResponse, HTMLResponse
from pydantic import BaseModel
import importlib.util

ATLAS_ROOT = Path(r"C:\ATLAS")
ROUTER_PATH = ATLAS_ROOT / "modules" / "command_router.py"
LOCAL_ROUTER = BASE_DIR / "modules" / "command_router.py"

def load_handle():
    if ROUTER_PATH.exists():
        spec = importlib.util.spec_from_file_location("command_router", str(ROUTER_PATH))
    else:
        spec = importlib.util.spec_from_file_location("command_router", str(LOCAL_ROUTER))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)  # type: ignore
    return mod.handle  # type: ignore

handle = load_handle()

from contextlib import asynccontextmanager


@asynccontextmanager
async def _lifespan(app):
    try:
        from modules.humanoid.deploy.healthcheck import _set_app_start_time
        _set_app_start_time()
    except Exception:
        pass
    try:
        from modules.humanoid.owner.emergency import set_emergency_from_env
        set_emergency_from_env()
    except Exception:
        pass
    humanoid = os.getenv("HUMANOID_ENABLED", "true").strip().lower() in ("1", "true", "yes", "y", "on")
    sched = os.getenv("SCHED_ENABLED", "true").strip().lower() in ("1", "true", "yes", "y", "on")
    worker_only = os.getenv("WORKER_ONLY", "false").strip().lower() in ("1", "true", "yes")
    if humanoid and sched:
        from modules.humanoid.scheduler import start_scheduler
        from modules.humanoid.watchdog import start_watchdog
        from concurrent.futures import ThreadPoolExecutor
        _executor = ThreadPoolExecutor(max_workers=2)
        start_scheduler(executor=_executor)
        start_watchdog()
        if not worker_only:
            try:
                from modules.humanoid.ci import ensure_ci_jobs
                ensure_ci_jobs()
            except Exception:
                pass
            try:
                from modules.humanoid.ga.scheduler_jobs import ensure_ga_jobs
                ensure_ga_jobs()
            except Exception:
                pass
            try:
                from modules.humanoid.metalearn.scheduler_jobs import ensure_metalearn_jobs
                ensure_metalearn_jobs()
            except Exception:
                pass
            try:
                from modules.humanoid.ans.scheduler_jobs import ensure_ans_jobs
                ensure_ans_jobs()
                # Ejecutar triada ANS al arranque (una vez) para que la bitácora tenga acción desde el inicio
                if os.getenv("ANS_ENABLED", "true").strip().lower() in ("1", "true", "yes") and os.getenv("ANS_RUN_AT_STARTUP", "true").strip().lower() in ("1", "true", "yes"):
                    def _run_triada_at_startup():
                        try:
                            from modules.humanoid.ans.engine import run_ans_cycle
                            run_ans_cycle(mode=os.getenv("ANS_MODE", "auto"), timeout_sec=60)
                        except Exception:
                            pass
                    import threading
                    t = threading.Thread(target=_run_triada_at_startup, daemon=True)
                    t.start()
            except Exception:
                pass
            try:
                from modules.humanoid.comms.makeplay_scheduler import ensure_makeplay_jobs
                ensure_makeplay_jobs()
            except Exception:
                pass
        # Heartbeat NEXUS: ping 8000/health, auto-reactivación con start_all.ps1, registro en Bitácora
        try:
            from modules.nexus_heartbeat import start_heartbeat, register_status_callback
            _dashboard_base = (os.getenv("ATLAS_DASHBOARD_URL") or "http://127.0.0.1:8791").rstrip("/")
            def _on_nexus_change(connected: bool, message: str) -> None:
                text = "[CONEXIÓN] Buscando NEXUS en puerto 8000... OK." if connected else "[CONEXIÓN] Buscando NEXUS en puerto 8000... Desconectado."
                try:
                    import urllib.request
                    import json
                    req = urllib.request.Request(
                        _dashboard_base + "/ans/evolution-log",
                        data=json.dumps({"message": text, "ok": connected}).encode("utf-8"),
                        headers={"Content-Type": "application/json"},
                        method="POST",
                    )
                    urllib.request.urlopen(req, timeout=3)
                except Exception:
                    pass
            register_status_callback(_on_nexus_change)
            start_heartbeat()
            # Prueba de Nervios: autodiagnóstico de motores, mouse, cámara; actualiza Dashboard a CONECTADO | ACTIVO
            def _run_nerve_test_background():
                try:
                    import sys
                    from pathlib import Path
                    push_base = Path(__file__).resolve().parent.parent
                    if str(push_base) not in sys.path:
                        sys.path.insert(0, str(push_base))
                    import nexus_actions
                    nexus_actions.run_nerve_test()
                except Exception:
                    pass
            import threading
            _nerve_thread = threading.Thread(target=_run_nerve_test_background, daemon=True)
            _nerve_thread.start()
        except Exception:
            pass
    # ATLAS AUTONOMOUS: background tasks (Health, Alert, Learning)
    try:
        import asyncio
        import sys
        if str(BASE_DIR) not in sys.path:
            sys.path.insert(0, str(BASE_DIR))
        from autonomous.health_monitor.health_aggregator import HealthAggregator
        from autonomous.telemetry.alert_manager import AlertManager
        from autonomous.learning.learning_orchestrator import LearningOrchestrator
        import logging
        _log = logging.getLogger(__name__)
        _log.info("Iniciando background tasks de ATLAS AUTONOMOUS...")
        health_agg = HealthAggregator()
        health_agg.start_monitoring()
        _log.info("✓ Health Monitoring activo")
        alert_mgr = AlertManager()
        asyncio.create_task(alert_mgr.start_evaluation_loop(60))
        _log.info("✓ Alert Manager activo")
        learning = LearningOrchestrator()

        async def _learning_loop():
            while True:
                await asyncio.sleep(3600)
                try:
                    learning.run_learning_cycle()
                    _log.info("✓ Learning cycle ejecutado")
                except Exception as e:
                    _log.error("Error en learning cycle: %s", e)

        asyncio.create_task(_learning_loop())
        _log.info("✓ Learning Engine activo")
    except Exception as e:
        import logging
        logging.getLogger(__name__).debug("ATLAS AUTONOMOUS background tasks no iniciados: %s", e)
    # Actualización periódica de métricas Prometheus (memoria, etc.)
    try:
        from modules.observability.metrics import update_system_metrics
        async def _metrics_loop():
            while True:
                await asyncio.sleep(10)
                try:
                    update_system_metrics()
                except Exception:
                    pass
        asyncio.create_task(_metrics_loop())
    except Exception:
        pass
    yield


app = FastAPI(
    title="ATLAS Adapter",
    version="1.0.0",
    lifespan=_lifespan,
    openapi_tags=[
        {"name": "Health", "description": "Health check extendido (score 0-100, LLM, scheduler, memory, DB, uptime)."},
        {"name": "Deploy", "description": "Blue-green deployment, canary ramp-up, deploy status y reportes."},
        {"name": "Cluster", "description": "Atlas Cluster: nodos, heartbeat, routing, ejecución remota (hands/web/vision/voice)."},
        {"name": "Gateway", "description": "Stealth Gateway: Cloudflare / Tailscale / SSH / LAN, bootstrap, health."},
    ],
)

# Metrics middleware: request count + latency per path
from modules.humanoid.metrics import MetricsMiddleware
app.add_middleware(MetricsMiddleware)
# Observabilidad Prometheus (request count, duration, active)
try:
    from modules.observability.middleware import ObservabilityMiddleware
    app.add_middleware(ObservabilityMiddleware)
except Exception:
    pass

class Step(BaseModel):
    tool: str
    args: dict = {}

def _robot_connected() -> bool:
    """Ping Robot (8002) para estado cámaras."""
    import urllib.request
    import os
    url = (os.getenv("NEXUS_ROBOT_API_URL") or os.getenv("NEXUS_ROBOT_URL") or "http://127.0.0.1:8002").rstrip("/") + "/"
    try:
        req = urllib.request.Request(url, method="GET")
        with urllib.request.urlopen(req, timeout=2) as r:
            return r.status == 200
    except Exception:
        return False


@app.get("/status")
def status():
    try:
        from modules.nexus_heartbeat import ping_nexus, set_nexus_connected, get_nexus_connection_state
        ok, msg = ping_nexus()
        set_nexus_connected(ok, "" if ok else msg)
        nexus = get_nexus_connection_state()
    except Exception:
        nexus = {"connected": False, "active": False, "last_check_ts": 0, "last_error": ""}
    robot_ok = _robot_connected()
    return {
        "ok": True,
        "atlas": handle("/status"),
        "nexus_connected": nexus.get("connected", False),
        "nexus_active": nexus.get("active", False),
        "nexus_last_check_ts": nexus.get("last_check_ts", 0),
        "nexus_last_error": nexus.get("last_error", ""),
        "nexus_dashboard_url": "http://127.0.0.1:8000/dashboard",
        "robot_connected": robot_ok,
    }


@app.get("/api/nexus/connection", tags=["NEXUS"])
def get_nexus_connection():
    """CEREBRO — CUERPO (NEXUS): estado de conexión en tiempo real."""
    try:
        from modules.nexus_heartbeat import get_nexus_connection_state
        return {"ok": True, **get_nexus_connection_state()}
    except Exception as e:
        return {"ok": False, "connected": False, "last_error": str(e)}


class NexusConnectionBody(BaseModel):
    connected: bool = False
    message: str = ""
    active: Optional[bool] = None


@app.post("/api/nexus/connection", tags=["NEXUS"])
def post_nexus_connection(body: NexusConnectionBody):
    """Actualiza estado de conexión NEXUS (heartbeat o Prueba de Nervios). active=True → CONECTADO | ACTIVO."""
    try:
        from modules.nexus_heartbeat import set_nexus_connected, set_nexus_active
        set_nexus_connected(body.connected, body.message or "")
        if body.active is not None:
            set_nexus_active(body.active)
        return {"ok": True, "connected": body.connected, "active": body.active}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@app.post("/api/nexus/reconnect", tags=["NEXUS"])
def nexus_reconnect(clear_cache: bool = False):
    """Reconectar NEXUS: mata proceso en 8000, opcionalmente limpia cache, arranca servicio y hace ping."""
    try:
        if clear_cache:
            from modules.nexus_heartbeat import clear_nexus_cache
            clear_nexus_cache()
        from modules.nexus_heartbeat import reconnect_nexus_and_poll
        state = reconnect_nexus_and_poll(wait_after_start_sec=6, poll_interval_sec=2, max_polls=15)
        return {"ok": True, "connected": state.get("connected", False), **state}
    except Exception as e:
        return {"ok": False, "connected": False, "error": str(e)}


@app.post("/api/cache/clear", tags=["NEXUS"])
def cache_clear():
    """Limpia cache del servidor (__pycache__, temp_models_cache). Para uso con botón Limpiar caché en navegador."""
    try:
        from modules.nexus_heartbeat import clear_nexus_cache
        clear_nexus_cache()
        return {"ok": True, "message": "Cache servidor limpiado"}
    except Exception as e:
        return {"ok": False, "error": str(e)}


# ---------- Semantic Memory (Fase 1 - Fundación Cognitiva) ----------
@app.post("/api/memory/add", tags=["Memory"])
def api_memory_add(
    description: str,
    context: Optional[str] = None,
    outcome: Optional[str] = None,
    tags: Optional[List[str]] = None,
):
    """Añade una experiencia a la memoria semántica."""
    try:
        from modules.humanoid.memory_engine.semantic_memory import get_semantic_memory
        exp_id = get_semantic_memory().add_experience(
            description=description,
            context=context,
            outcome=outcome,
            tags=tags,
        )
        return {"id": exp_id, "status": "stored"}
    except Exception as e:
        return {"id": None, "status": "error", "error": str(e)}


@app.post("/api/memory/search", tags=["Memory"])
def api_memory_search(
    query: str,
    top_k: int = 5,
    min_similarity: float = 0.6,
):
    """Búsqueda por similaridad en memoria semántica."""
    try:
        from modules.humanoid.memory_engine.semantic_memory import get_semantic_memory
        results = get_semantic_memory().recall_similar(
            query, top_k=top_k, min_similarity=min_similarity
        )
        return {"results": results, "query": query}
    except Exception as e:
        return {"results": [], "query": query, "error": str(e)}


@app.get("/api/memory/stats", tags=["Memory"])
def api_memory_stats():
    """Estadísticas de la memoria semántica."""
    try:
        from modules.humanoid.memory_engine.semantic_memory import get_semantic_memory
        return get_semantic_memory().get_statistics()
    except Exception as e:
        return {"error": str(e), "total_experiences": 0, "embedding_dimension": 0, "storage_size_mb": 0}


# ---------- Fase 2: Vision (depth + scene) ----------
@app.post("/api/vision/depth/estimate", tags=["Vision"])
async def api_vision_depth_estimate(file: UploadFile = File(..., description="Image file")):
    """Estima mapa de profundidad desde imagen. Devuelve depth coloreado base64 y shape."""
    try:
        import numpy as np
        import cv2
        import base64
        contents = await file.read()
        arr = np.frombuffer(contents, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            return {"ok": False, "error": "invalid image"}
        from modules.humanoid.vision.depth_estimation import estimate_depth, depth_map_to_colored
        depth = estimate_depth(frame)
        colored = depth_map_to_colored(depth)
        _, buf = cv2.imencode(".png", colored)
        b64 = base64.b64encode(buf).decode("utf-8")
        return {"ok": True, "depth_map_b64": b64, "shape": list(depth.shape), "dtype": str(depth.dtype)}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@app.post("/api/vision/scene/describe", tags=["Vision"])
async def api_vision_scene_describe(file: UploadFile = File(..., description="Image file"), detail_level: str = "medium"):
    """Descripción de escena desde imagen. detail_level: brief | medium | detailed."""
    try:
        import numpy as np
        import cv2
        from modules.humanoid.vision.scene_understanding import describe_scene
        contents = await file.read()
        arr = np.frombuffer(contents, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            return {"ok": False, "error": "invalid image", "description": ""}
        data = describe_scene(frame, detail_level=detail_level)
        return {"ok": True, **data}
    except Exception as e:
        return {"ok": False, "error": str(e), "description": ""}


# ---------- Fase 1: World model (stub) ----------
@app.post("/api/world-model/simulate", tags=["WorldModel"])
def api_world_model_simulate():
    """Simulación de acción en world model (stub). Fase 1.2 completo: PyBullet + MCTS."""
    try:
        # TODO: integrar physics_simulator.load_scene + simulate_action
        return {
            "ok": True,
            "stub": True,
            "final_state": {},
            "collision_detected": False,
            "objects_moved": [],
            "success": True,
            "message": "World model stub; implementar physics_simulator.py",
        }
    except Exception as e:
        return {"ok": False, "error": str(e)}


# ---------- Fase 4: Meta-Learning (MAML) ----------
_maml_instance = None


class MetaAdaptBody(BaseModel):
    """Body para POST /api/meta-learning/adapt."""
    demonstrations: List[dict] = []
    num_steps: int = 5


def _get_maml():
    """Lazy singleton MAML (requiere torch)."""
    global _maml_instance
    if _maml_instance is not None:
        return _maml_instance
    try:
        from autonomous.learning.meta_learning.maml import MAML
        if MAML is None:
            return None
        _maml_instance = MAML(state_dim=128, action_dim=64, meta_lr=1e-3, inner_lr=1e-2, inner_steps=5)
        return _maml_instance
    except Exception:
        return None


@app.post("/api/meta-learning/train", tags=["MetaLearning"])
def api_meta_learning_train(num_tasks: int = 8, num_steps: int = 100):
    """Meta-entrenar MAML sobre distribución de tareas."""
    maml = _get_maml()
    if maml is None:
        return {"status": "error", "error": "MAML no disponible (PyTorch requerido)"}
    try:
        import numpy as np
        results = []
        for step in range(num_steps):
            metrics = maml.meta_train_step(num_tasks=num_tasks)
            if "error" in metrics:
                return {"status": "error", "error": metrics["error"]}
            if step % 10 == 0:
                results.append({"step": step, "meta_loss": metrics["meta_loss"], "avg_task_loss": metrics["avg_task_loss"]})
        snap_dir = BASE_DIR / "snapshots"
        snap_dir.mkdir(parents=True, exist_ok=True)
        maml.save_checkpoint(str(snap_dir / "maml_checkpoint.pt"))
        return {
            "status": "meta_training_complete",
            "steps": num_steps,
            "final_loss": results[-1]["meta_loss"] if results else 0,
            "training_curve": results,
        }
    except Exception as e:
        return {"status": "error", "error": str(e)}


@app.post("/api/meta-learning/adapt", tags=["MetaLearning"])
def api_meta_learning_adapt(body: MetaAdaptBody):
    """Adaptar a nueva tarea con pocas demostraciones (few-shot)."""
    maml = _get_maml()
    if maml is None:
        return {"status": "error", "error": "MAML no disponible (PyTorch requerido)"}
    demonstrations = body.demonstrations or []
    num_steps = body.num_steps or 5
    try:
        import numpy as np
        demo_tuples = [
            (np.array(d.get("state", [])), np.array(d.get("action", [])), float(d.get("reward", 0)))
            for d in demonstrations
        ]
        adapted_policy = maml.adapt_to_new_task(demo_tuples, num_adaptation_steps=num_steps)
        snap_dir = BASE_DIR / "snapshots"
        snap_dir.mkdir(parents=True, exist_ok=True)
        import torch
        torch.save(adapted_policy.state_dict(), str(snap_dir / "adapted_policy.pt"))
        return {
            "status": "adapted",
            "num_demonstrations": len(demonstrations),
            "adaptation_steps": num_steps,
            "policy_path": "snapshots/adapted_policy.pt",
        }
    except Exception as e:
        return {"status": "error", "error": str(e)}


@app.get("/api/meta-learning/stats", tags=["MetaLearning"])
def api_meta_learning_stats():
    """Estadísticas de meta-learning."""
    maml = _get_maml()
    if maml is None:
        return {"num_tasks_in_buffer": 0, "error": "MAML no disponible"}
    return {
        "num_tasks_in_buffer": len(maml.task_buffer),
        "policy_parameters": sum(p.numel() for p in maml.policy.parameters()),
        "inner_lr": maml.inner_lr,
        "inner_steps": maml.inner_steps,
    }


@app.post("/api/meta-learning/generate-tasks", tags=["MetaLearning"])
def api_meta_learning_generate_tasks(task_type: str = "navigation", num_tasks: int = 50):
    """Generar tareas sintéticas para meta-entrenamiento."""
    maml = _get_maml()
    if maml is None:
        return {"status": "error", "error": "MAML no disponible (PyTorch requerido)"}
    try:
        from autonomous.learning.meta_learning.task_generator import TaskGenerator
        if task_type == "navigation":
            tasks = TaskGenerator.generate_navigation_tasks(num_tasks)
        elif task_type == "manipulation":
            tasks = TaskGenerator.generate_manipulation_tasks(num_tasks)
        else:
            return {"error": f"Unknown task type: {task_type}"}
        for task in tasks:
            maml.add_task(task)
        return {
            "status": "tasks_generated",
            "task_type": task_type,
            "num_tasks": num_tasks,
            "total_in_buffer": len(maml.task_buffer),
        }
    except Exception as e:
        return {"status": "error", "error": str(e)}


# ---------- Fase 4: Causal Reasoning ----------
_causal_reasoner = None


def _get_causal_reasoner():
    global _causal_reasoner
    if _causal_reasoner is not None:
        return _causal_reasoner
    try:
        from brain.reasoning.causal_model import CausalReasoner
        _causal_reasoner = CausalReasoner()
        return _causal_reasoner
    except Exception:
        return None


@app.post("/api/causal/create-domain", tags=["Causal"])
def api_causal_create_domain(domain_name: str, structure: dict):
    """Crear grafo causal para un dominio."""
    reasoner = _get_causal_reasoner()
    if reasoner is None:
        return {"error": "CausalReasoner no disponible (networkx requerido)"}
    try:
        graph = reasoner.create_domain(domain_name, structure)
        return {
            "domain": domain_name,
            "variables": structure.get("variables", []),
            "num_edges": len(structure.get("edges", [])),
            "graph": graph.visualize(),
        }
    except Exception as e:
        return {"error": str(e)}


@app.post("/api/causal/reason", tags=["Causal"])
def api_causal_reason(domain: str, action: str, current_state: dict):
    """Razonar consecuencias de una acción."""
    reasoner = _get_causal_reasoner()
    if reasoner is None:
        return {"error": "CausalReasoner no disponible"}
    return reasoner.reason_about_action(domain, action, current_state or {})


@app.post("/api/causal/explain", tags=["Causal"])
def api_causal_explain(domain: str, effect: str, potential_causes: List[str]):
    """Explicar por qué ocurrió effect dados potential_causes."""
    reasoner = _get_causal_reasoner()
    if reasoner is None:
        return {"error": "CausalReasoner no disponible", "explanation": ""}
    explanation = reasoner.explain_why(domain, effect, potential_causes or [])
    return {"explanation": explanation}


@app.post("/api/causal/counterfactual", tags=["Causal"])
def api_causal_counterfactual(domain: str, variable: str, value: float, reality: dict):
    """Razonamiento contrafáctico: qué si variable hubiera sido value."""
    reasoner = _get_causal_reasoner()
    if reasoner is None:
        return {"error": "CausalReasoner no disponible"}
    if domain not in reasoner.graphs:
        return {"error": f"Unknown domain: {domain}"}
    result = reasoner.graphs[domain].counterfactual(variable, value, reality or {})
    return result


# ---------- Fase 4.3: Self-Programming ----------
class SelfProgramValidateBody(BaseModel):
    """Body para POST /api/self-programming/validate."""
    code: str = ""
    function_name: str = "run"
    test_cases: List[dict] = []


class SelfProgramOptimizeBody(BaseModel):
    code: str = ""


class SelfProgramExecuteBody(BaseModel):
    code: str = ""
    function_name: str = "run"
    test_input: dict = {}


_skill_validator = None
_skill_optimizer = None


def _get_skill_validator():
    global _skill_validator
    if _skill_validator is None:
        try:
            from brain.self_programming.skill_sandbox import SkillValidator
            _skill_validator = SkillValidator()
        except Exception:
            pass
    return _skill_validator


def _get_skill_optimizer():
    global _skill_optimizer
    if _skill_optimizer is None:
        try:
            from brain.self_programming.skill_optimizer import SkillOptimizer
            _skill_optimizer = SkillOptimizer()
        except Exception:
            pass
    return _skill_optimizer


@app.post("/api/self-programming/validate", tags=["SelfProgramming"])
def api_self_programming_validate(body: SelfProgramValidateBody):
    """Valida skill: sintaxis, seguridad, tests en sandbox."""
    validator = _get_skill_validator()
    if validator is None:
        return {"overall_valid": False, "error": "SkillValidator no disponible"}
    return validator.validate_comprehensive(
        body.code, body.function_name, body.test_cases or []
    )


@app.post("/api/self-programming/optimize", tags=["SelfProgramming"])
def api_self_programming_optimize(body: SelfProgramOptimizeBody):
    """Optimiza código generado (errores, type hints, docstrings)."""
    optimizer = _get_skill_optimizer()
    if optimizer is None:
        return {"error": "SkillOptimizer no disponible"}
    return optimizer.optimize_code(body.code or "")


@app.post("/api/self-programming/execute-sandbox", tags=["SelfProgramming"])
def api_self_programming_execute_sandbox(body: SelfProgramExecuteBody):
    """Ejecuta código en sandbox Docker."""
    try:
        from brain.self_programming.skill_sandbox import SkillSandbox
        sandbox = SkillSandbox()
        return sandbox.execute_safely(
            body.code or "",
            body.function_name or "run",
            body.test_input or {},
        )
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.get("/api/robot/status", tags=["NEXUS"])
def robot_status():
    """Estado del Robot (cámaras, visión) en puerto 8002. Para panel de cámaras."""
    import urllib.request
    import os
    url = (os.getenv("NEXUS_ROBOT_URL") or os.getenv("NEXUS_ROBOT_API_URL") or "http://127.0.0.1:8002").rstrip("/") + "/api/camera/service/status"
    try:
        req = urllib.request.Request(url, method="GET", headers={"Accept": "application/json"})
        with urllib.request.urlopen(req, timeout=4) as r:
            if r.status == 200:
                return {"ok": True, "connected": True, "robot_url": url.rsplit("/", 1)[0]}
    except Exception as e:
        pass
    url_health = (os.getenv("NEXUS_ROBOT_API_URL") or "http://127.0.0.1:8002").rstrip("/") + "/"
    try:
        req = urllib.request.Request(url_health, method="GET")
        with urllib.request.urlopen(req, timeout=3) as r:
            return {"ok": True, "connected": True, "robot_url": url_health.rstrip("/")}
    except Exception:
        pass
    return {"ok": False, "connected": False, "robot_url": "http://127.0.0.1:8002"}


@app.get("/health", tags=["Health"])
def health():
    """Health verificable: ok, score 0-100, checks (api_up, memory_writable, audit_writable, scheduler_alive, llm_reachable, avg_latency_ms, error_rate, active_port, version, channel), ms, error."""
    try:
        from modules.humanoid.deploy.healthcheck import run_health_verbose
        from modules.humanoid.deploy.ports import get_ports
        active_port = get_ports()[0]
        return run_health_verbose(base_url=None, active_port=active_port)
    except Exception as e:
        return {"ok": False, "score": 0, "checks": {}, "ms": 0, "error": str(e)}


@app.get("/health/debug", tags=["Health"])
def health_debug():
    """Raw check results con mensajes de error para diagnóstico."""
    try:
        from modules.humanoid.deploy.healthcheck import (
            _check_memory_writable,
            _check_audit_writable,
            _check_scheduler_running,
        )
        return {
            "ok": True,
            "AUDIT_DB_PATH": os.getenv("AUDIT_DB_PATH"),
            "SCHED_DB_PATH": os.getenv("SCHED_DB_PATH"),
            "ATLAS_MEMORY_DB_PATH": os.getenv("ATLAS_MEMORY_DB_PATH"),
            "memory": _check_memory_writable(),
            "audit": _check_audit_writable(),
            "scheduler": _check_scheduler_running(),
        }
    except Exception as e:
        return {"ok": False, "error": str(e)}


@app.get("/version")
def version():
    """Release version, git sha, channel (stable/canary), edition, product_mode."""
    try:
        from modules.humanoid.release import get_version_info
        out = {"ok": True, **get_version_info()}
        if os.getenv("PRODUCT_MODE", "").strip().lower() in ("1", "true", "yes"):
            try:
                from modules.humanoid.product.license import license_status
                out["license"] = license_status()
            except Exception:
                out["license"] = {"status": "unknown"}
        return out
    except Exception as e:
        return {"ok": False, "version": "0.0.0", "git_sha": "", "channel": "canary", "error": str(e)}


@app.get("/product/status")
def product_status():
    """Product dashboard: version, edition, license, service hint, cluster, gateway, health score, last GA/metalearn, support_contact."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.release import get_version_info
        from modules.humanoid.product.license import license_status
        from modules.humanoid.deploy.healthcheck import run_health_verbose
        from modules.humanoid.deploy.ports import get_ports
        version_info = get_version_info()
        license_info = license_status()
        port = get_ports()[0] if get_ports() else int(os.getenv("SERVICE_PORT", "8791") or 8791)
        health = run_health_verbose(base_url=None, active_port=port)
        ga_last = None
        meta_last = None
        try:
            from modules.humanoid.ga.cycle import get_status as ga_status
            ga_last = ga_status().get("last_run_ts")
        except Exception:
            pass
        try:
            from modules.humanoid.metalearn.cycle import get_status as meta_status
            meta_last = meta_status().get("last_update_ts")
        except Exception:
            pass
        cluster_nodes = 0
        gateway_state = "unknown"
        try:
            from modules.humanoid.cluster.registry import list_nodes
            cluster_nodes = len(list_nodes())
        except Exception:
            pass
        try:
            from modules.humanoid.gateway.store import build_gateway_status
            gw = build_gateway_status()
            gateway_state = gw.get("mode", "unknown")
        except Exception:
            pass
        service_name = os.getenv("SERVICE_NAME", "ATLAS_PUSH")
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(True, {
            "version": version_info.get("version", "0.0.0"),
            "edition": version_info.get("edition", "community"),
            "license": license_info,
            "service_name": service_name,
            "active_port": port,
            "health_score": health.get("score", 0),
            "cluster_nodes": cluster_nodes,
            "gateway_state": gateway_state,
            "last_ga_run_ts": ga_last,
            "last_metalearn_run_ts": meta_last,
            "support_contact": os.getenv("SUPPORT_CONTACT", ""),
        }, ms, None)
    except Exception as e:
        return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), str(e))


# Dashboard UI (fallback: if UI fails, API still works)
STATIC_DIR = BASE_DIR / "atlas_adapter" / "static"
STATIC_DIR.mkdir(parents=True, exist_ok=True)


@app.get("/ui")
def serve_ui():
    """Dashboard: estado, versión, salud, métricas, programador, actualización, despliegue, aprobaciones e interacción con el robot."""
    path = STATIC_DIR / "dashboard.html"
    if path.exists():
        return FileResponse(path)
    return {"ok": False, "error": "dashboard.html not found"}


@app.get("/ui/lang")
def ui_lang(locale: str = "es"):
    """Cadenas i18n para el dashboard (es | en)."""
    try:
        from modules.humanoid.config import SUPPORTED_LOCALES
        from modules.humanoid.config.i18n import get_all_strings
        loc = locale.strip().lower() if locale else "es"
        if loc not in SUPPORTED_LOCALES:
            loc = "es"
        return {"ok": True, "locale": loc, "strings": get_all_strings(loc)}
    except Exception as e:
        return {"ok": False, "locale": "es", "strings": {}, "error": str(e)}


@app.get("/tools")
def tools():
    return {"ok": True, "tools": [
        "atlas.status","atlas.doctor","atlas.modules","atlas.snapshot",
        "atlas.note.create","atlas.note.append","atlas.note.view","atlas.inbox"
    ]}

@app.post("/execute")
def execute(step: Step):
    t = step.tool
    a = step.args or {}
    if t == "atlas.status": cmd = "/status"
    elif t == "atlas.doctor": cmd = "/doctor"
    elif t == "atlas.modules": cmd = "/modules"
    elif t == "atlas.snapshot": cmd = f"/snapshot {a.get('label','snapshot')}"
    elif t == "atlas.note.create": cmd = f"/note create {a.get('title','Nota')}"
    elif t == "atlas.note.append": cmd = f"/note append {a.get('title','Inbox')} | {a.get('text','')}"
    elif t == "atlas.note.view": cmd = f"/note view {a.get('title','Inbox')}"
    elif t == "atlas.inbox": cmd = a.get("text","")
    else:
        return {"ok": False, "error": f"Tool no soportada aún: {t}"}
    out = handle(cmd)
    return {"ok": True, "tool": t, "output": out}

@app.get("/modules")
def modules():
    return {
        "ok": True,
        "modules": [
            {"name": "vision", "enabled": False},
            {"name": "voice", "enabled": False},
            {"name": "agent_router", "enabled": False},
            {"name": "telegram", "enabled": True},
        ]
    }

from pydantic import BaseModel
from typing import Any, List, Optional
import time

class IntentIn(BaseModel):
    user: str = "raul"
    text: str
    meta: Optional[dict[str, Any]] = None

@app.post("/intent")
def intent(payload: IntentIn):
    # Respuesta mínima (v1): eco + timestamp.
    # Luego conectamos command_router/agent_router.
    return {
        "ok": True,
        "received": payload.model_dump(),
        "result": {
            "message": "Intent recibido",
            "ts": time.time(),
        }
    }
# --- Canonical Intent API (v1) ---
from pydantic import BaseModel
from typing import Any, List, Optional
import time

from modules.command_router import handle as route_command

class IntentIn(BaseModel):
    user: str = "raul"
    text: str = ""
    meta: Optional[dict[str, Any]] = None

@app.post("/intent")
def intent(payload: IntentIn):
    t0 = time.time()
    if not (payload.text or "").strip():
        return {
            "ok": False,
            "received": payload.model_dump(),
            "error": "Campo 'text' es requerido (ej: {\"text\": \"/status\"})",
            "ms": int((time.time() - t0) * 1000),
        }
    try:
        out = route_command((payload.text or "").strip())
        return {
            "ok": True,
            "received": payload.model_dump(),
            "output": out,
            "ms": int((time.time() - t0) * 1000),
        }
    except Exception as e:
        return {
            "ok": False,
            "received": payload.model_dump(),
            "error": str(e),
            "ms": int((time.time() - t0) * 1000),
        }


# --- Push Dashboard (ATLAS_PUSH -> Dashboard -> ATLAS_NEXUS) ---
_push_state: dict = {"last_command": None, "last_ack": None, "updated_at": None}

# --- Evolution (ATLAS_EVOLUTION): gobernanza y pendientes de aprobación ---
_evolution_pending: list = []  # [{ package, old_version, new_version, ts, worker }]
_evolution_governed: bool = os.environ.get("EVOLUTION_GOVERNED", "").strip().upper() in ("1", "TRUE", "GOVERNED")
_evolution_last_report: dict = {}  # Último reporte del daemon: { state, message, asimilacion_exitosa, ts }

class PushCommandBody(BaseModel):
    target: str = "NEXUS_ARM"
    action: str = "update_state"
    value: Any = 1

@app.post("/api/push/command", tags=["Push"])
def push_command(body: PushCommandBody):
    """Recibe comando del Cerebro; NEXUS lee el estado y ejecuta. Si target=EVOLUTION_REPORT, registra pendientes y último reporte (Asimilación Exitosa)."""
    global _push_state, _evolution_pending, _evolution_last_report
    cmd = {"target": body.target, "action": body.action, "value": body.value, "ts": time.time()}
    _push_state["last_command"] = cmd
    _push_state["updated_at"] = time.time()
    if body.target == "EVOLUTION_REPORT" and body.action == "worker_result" and isinstance(body.value, str):
        try:
            import json
            payload = json.loads(body.value)
            if payload.get("status") == "pending_approval" or payload.get("event") == "evolution_pending_approval":
                _evolution_pending.append({
                    "package": payload.get("package"),
                    "old_version": payload.get("old_version"),
                    "new_version": payload.get("new_version"),
                    "ts": time.time(),
                    "worker": payload.get("worker", "pypi"),
                })
            if payload.get("event") == "ans_bitacora" or payload.get("asimilacion_exitosa") is not None or payload.get("state"):
                _evolution_last_report = {
                    "state": payload.get("state", ""),
                    "message": payload.get("message", ""),
                    "asimilacion_exitosa": payload.get("asimilacion_exitosa", False),
                    "ts": time.time(),
                }
        except Exception:
            pass
    return {"ok": True, "command": cmd}

@app.get("/api/push/state", tags=["Push"])
def push_state():
    """Estado actual del Dashboard para evitar comandos duplicados."""
    return {"ok": True, "state": _push_state.get("last_command"), "ack": _push_state.get("last_ack"), "updated_at": _push_state.get("updated_at")}

@app.post("/api/push/ack", tags=["Push"])
def push_ack(body: dict):
    """NEXUS confirma recepción/ejecución."""
    global _push_state
    _push_state["last_ack"] = body
    _push_state["updated_at"] = time.time()
    return {"ok": True}


@app.get("/api/evolution/status", tags=["Evolution"])
def evolution_status():
    """Estado de gobernanza, pendientes de aprobación y último reporte del daemon (Asimilación Exitosa)."""
    return {
        "governed": _evolution_governed,
        "pending": list(_evolution_pending),
        "last_report": dict(_evolution_last_report),
        "updated_at": _push_state.get("updated_at") or time.time(),
    }


class EvolutionApproveBody(BaseModel):
    package: str
    new_version: str


@app.post("/api/evolution/approve", tags=["Evolution"])
def evolution_approve(body: EvolutionApproveBody):
    """Confirma y aplica un cambio pendiente al requirements.txt del proyecto (núcleo)."""
    global _evolution_pending
    req_txt = Path(os.environ.get("ATLAS_BASE", BASE_DIR)) / "requirements.txt"
    if not req_txt.exists():
        return {"ok": False, "error": "requirements.txt no encontrado"}
    for i, p in enumerate(_evolution_pending):
        if p.get("package") == body.package and p.get("new_version") == body.new_version:
            try:
                text = req_txt.read_text(encoding="utf-8")
                lines = text.splitlines()
                new_lines = []
                for line in lines:
                    stripped = line.strip()
                    if stripped.startswith("#") or not stripped:
                        new_lines.append(line)
                        continue
                    name = line.split("==")[0].split(">=")[0].split("[")[0].strip()
                    if name == body.package:
                        new_lines.append(f"{body.package}=={body.new_version}")
                        continue
                    new_lines.append(line)
                req_txt.write_text("\n".join(new_lines) + ("\n" if text.endswith("\n") else ""), encoding="utf-8")
            except Exception as e:
                return {"ok": False, "error": str(e)}
            _evolution_pending.pop(i)
            return {"ok": True, "applied": f"{body.package}=={body.new_version}"}
    return {"ok": False, "error": "Pendiente no encontrado"}


# --- LLM híbrido (router + Ollama) ---
from modules.llm.schemas import LLMRequest, LLMResponse
from modules.llm.service import LLMService

llm_service = LLMService()


@app.post(
    "/llm",
    response_model=LLMResponse,
    summary="Hybrid LLM (Ollama)",
    description="Route prompt to FAST/CHAT/CODE/REASON/TOOLS via HybridRouter and call Ollama. Returns ok, output, route, model_used, ms, tokens_est, error, meta.",
)
def llm_endpoint(req: LLMRequest) -> LLMResponse:
    """Run LLM request through router + Ollama and return structured response."""
    resp = llm_service.run(req)
    try:
        from modules.humanoid.metalearn.collector import record_router_call
        record_router_call(getattr(resp, "route", "CHAT") or "CHAT", getattr(resp, "model_used", "") or "", "ok" if getattr(resp, "ok", True) else "fail", getattr(resp, "ms", None))
    except Exception:
        pass
    return resp


# --- Multi-AI (registry, router, status) ---
@app.get("/ai/status", tags=["AI"])
def ai_status():
    """Multi-AI status: providers, route_to_model, budget, policies, telemetry."""
    from modules.humanoid.ai.status import get_ai_status
    return get_ai_status()


class AIRouteBody(BaseModel):
    prompt: Optional[str] = None
    intent_hint: Optional[str] = None
    modality: Optional[str] = "text"
    intent: Optional[str] = None
    complexity: Optional[str] = None
    latency_need: Optional[str] = None
    safety: Optional[str] = None
    prefer_free: Optional[bool] = True


@app.post("/ai/route", tags=["AI"])
def ai_route(body: AIRouteBody):
    """Return RouteDecision for TaskProfile. Use prompt+intent_hint or intent/complexity/modality."""
    try:
        from modules.humanoid.ai.router import infer_task_profile, decide_route
        from modules.humanoid.ai.models import TaskProfile
        if body.prompt:
            profile = infer_task_profile(
                body.prompt,
                intent_hint=body.intent_hint or body.intent,
                modality=body.modality or "text",
            )
        else:
            profile = TaskProfile(
                intent=body.intent or "chat",
                complexity=body.complexity or "med",
                latency_need=body.latency_need or "normal",
                modality=body.modality or "text",
                safety=body.safety or "med",
            )
        decision = decide_route(profile, prefer_free=body.prefer_free if body.prefer_free is not None else True)
        return {
            "ok": True,
            "decision": {
                "provider_id": decision.provider_id,
                "model_key": decision.model_key,
                "route": decision.route,
                "reason": decision.reason,
                "is_free": decision.is_free,
                "cost_estimate_usd": decision.cost_estimate_usd,
            },
        }
    except Exception as e:
        return {"ok": False, "decision": None, "error": str(e)}


@app.get("/mode/capabilities", tags=["Mode"])
def mode_capabilities():
    """ATLAS_MODE capabilities: screen_act, record_replay, playwright, benchmark."""
    try:
        from modules.humanoid.mode import get_mode_capabilities
        return {"ok": True, "data": get_mode_capabilities()}
    except Exception as e:
        return {"ok": False, "data": None, "error": str(e)}


# --- Cursor mode (end-to-end orchestration) ---
class CursorRunBody(BaseModel):
    goal: str
    mode: Optional[str] = "plan_only"  # plan_only | controlled | auto
    depth: Optional[int] = 1  # 1-5
    prefer_free: Optional[bool] = True
    allow_paid: Optional[bool] = False
    profile: Optional[str] = "owner"  # owner | operario | contadora
    context: Optional[dict] = None


@app.post("/cursor/run", tags=["Cursor"])
def cursor_run_endpoint(body: CursorRunBody):
    """Cursor-mode run: plan with AI router, optional execution. Returns summary, evidence, model_routing."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.cursor import cursor_run
        result = cursor_run(
            goal=body.goal,
            mode=body.mode or "plan_only",
            depth=max(1, min(5, body.depth or 1)),
            context=body.context,
            prefer_free=body.prefer_free if body.prefer_free is not None else True,
            allow_paid=body.allow_paid if body.allow_paid is not None else False,
            profile=body.profile or "owner",
        )
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(result.get("ok", False), result.get("data"), ms, result.get("error"))
    except Exception as e:
        return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), str(e))


class CursorStepBody(BaseModel):
    step_index: int = 0
    approve: Optional[bool] = False


@app.post("/cursor/step/execute", tags=["Cursor"])
def cursor_step_execute(body: CursorStepBody):
    """Execute one step from last cursor run (policy/approvals)."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.cursor import get_cursor_status
        status = get_cursor_status()
        data = status.get("data")
        if not data or not data.get("steps"):
            return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), "no cursor run or no steps")
        idx = max(0, min(body.step_index, len(data["steps"]) - 1))
        step = data["steps"][idx]
        if not body.approve:
            return _std_resp(True, {"step": step, "executed": False, "message": "Set approve=true to execute"}, int((time.perf_counter() - t0) * 1000), None)
        data["steps"][idx]["status"] = "executed"
        return _std_resp(True, {"step": step, "executed": True}, int((time.perf_counter() - t0) * 1000), None)
    except Exception as e:
        return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), str(e))


@app.get("/cursor/status", tags=["Cursor"])
def cursor_status_endpoint():
    """Last cursor run state: goal, steps, model_routing, evidence."""
    try:
        from modules.humanoid.cursor import get_cursor_status
        return get_cursor_status()
    except Exception as e:
        return {"ok": False, "data": None, "message": str(e)}


# --- Humanoid (kernel + modules) ---
from modules.humanoid.api import router as humanoid_router
from modules.humanoid.ga.api import router as ga_router
from modules.humanoid.metalearn.api import router as metalearn_router

from modules.humanoid.ans.api import router as ans_router
from modules.humanoid.governance.api import router as governance_router

app.include_router(humanoid_router)
app.include_router(ga_router)
app.include_router(metalearn_router)
app.include_router(ans_router)
app.include_router(governance_router)

# ATLAS AUTONOMOUS (health, healing, evolution, telemetry, resilience, learning)
try:
    import sys
    if str(BASE_DIR) not in sys.path:
        sys.path.insert(0, str(BASE_DIR))
    from autonomous.api_routes import router as autonomous_router
    app.include_router(autonomous_router)
except Exception as e:
    import logging
    logging.getLogger(__name__).warning("Autonomous module not loaded: %s", e)


@app.get("/autonomous/dashboard", response_class=HTMLResponse)
async def autonomous_dashboard():
    """Dashboard UI de ATLAS AUTONOMOUS: health, servicios, healing, learning."""
    html_file = BASE_DIR / "templates" / "autonomous_dashboard.html"
    if not html_file.exists():
        return HTMLResponse("<h1>Dashboard no encontrado</h1>", status_code=404)
    return HTMLResponse(html_file.read_text(encoding="utf-8"))

# --- Proxy NEXUS: /actions/log y /ws (evitar 404/403 cuando el dashboard usa origen PUSH) ---
NEXUS_BASE = (os.getenv("NEXUS_BASE_URL") or "http://127.0.0.1:8000").rstrip("/")


@app.get("/actions/log")
async def proxy_actions_log(limit: int = 50):
    """Proxy a NEXUS: log de acciones para panel tipo Cursor."""
    try:
        import httpx
        async with httpx.AsyncClient(timeout=10) as client:
            r = await client.get(f"{NEXUS_BASE}/actions/log", params={"limit": limit})
            return r.json() if r.headers.get("content-type", "").startswith("application/json") else {"ok": False, "entries": []}
    except Exception as e:
        import logging
        logging.getLogger(__name__).debug("Proxy /actions/log: %s", e)
        return {"ok": False, "entries": [], "error": str(e)}


@app.websocket("/ws")
async def proxy_websocket(websocket: WebSocket):
    """Proxy WebSocket a NEXUS para tiempo real (evitar 403 cuando el cliente usa origen PUSH)."""
    import asyncio
    import logging
    log = logging.getLogger(__name__)
    ws_nexus_url = NEXUS_BASE.replace("http://", "ws://").replace("https://", "wss://") + "/ws"
    try:
        import websockets
    except ImportError:
        try:
            await websocket.accept()
            await websocket.close(code=1011)
        except Exception:
            pass
        log.warning("Proxy /ws: instalar 'websockets' para reenviar a NEXUS")
        return
    try:
        async with websockets.connect(ws_nexus_url, close_timeout=5) as ws_nexus:
            await websocket.accept()

            async def client_to_nexus():
                try:
                    while True:
                        msg = await websocket.receive()
                        if "text" in msg:
                            await ws_nexus.send(msg["text"])
                        elif "bytes" in msg:
                            await ws_nexus.send(msg["bytes"])
                except Exception:
                    pass

            async def nexus_to_client():
                try:
                    while True:
                        msg = await ws_nexus.recv()
                        if isinstance(msg, bytes):
                            await websocket.send_bytes(msg)
                        else:
                            await websocket.send_text(msg)
                except Exception:
                    pass

            await asyncio.gather(
                asyncio.create_task(client_to_nexus()),
                asyncio.create_task(nexus_to_client()),
            )
    except Exception as e:
        log.debug("Proxy /ws: %s", e)
        try:
            await websocket.accept()
        except Exception:
            pass
        try:
            await websocket.close(code=1011)
        except Exception:
            pass

# --- Cuerpo (Robot): proxy cámaras y API Robot 8002 ---
from modules.cuerpo_proxy import proxy_to_cuerpo
from fastapi import Request
from fastapi.responses import Response

@app.api_route("/cuerpo", methods=["GET", "POST"])
@app.api_route("/cuerpo/{path:path}", methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"])
async def cuerpo_proxy_route(request: Request, path: str = "") -> Response:
    """Proxy a Robot (cámaras, visión) en NEXUS_ROBOT_URL (8002). Panel de cámaras integrado."""
    return await proxy_to_cuerpo(request, path)


# --- Metrics / Policy / Audit endpoints ---
from modules.humanoid.metrics import get_metrics_store
from modules.humanoid.policy import ActorContext, get_policy_engine
from modules.humanoid.audit import get_audit_logger


@app.get("/metrics")
def metrics():
    """JSON metrics: counters and latencies per endpoint."""
    return get_metrics_store().snapshot()


# Prometheus scrape endpoint (text/plain)
@app.get("/metrics/prometheus", include_in_schema=False)
def prometheus_metrics():
    """Métricas en formato Prometheus para scraping."""
    try:
        from modules.observability.metrics import MetricsCollector
        from fastapi.responses import Response
        return Response(
            content=MetricsCollector.get_prometheus_text(),
            media_type="text/plain; charset=utf-8",
        )
    except Exception as e:
        from fastapi.responses import Response
        return Response(content=f"# error: {e}".encode(), media_type="text/plain", status_code=500)


@app.get("/api/observability/metrics", tags=["Observability"])
def api_observability_metrics():
    """Resumen de métricas (requests, memoria, health score)."""
    try:
        from modules.observability.metrics import MetricsCollector
        return MetricsCollector.get_metrics_summary()
    except Exception as e:
        return {"error": str(e), "total_requests": 0, "active_requests": 0, "memory_mb": 0}


class PolicyTestBody(BaseModel):
    actor: str = "api"
    role: str = "owner"
    module: str = "hands"
    action: str = "exec_command"
    target: Optional[str] = None


@app.post("/policy/test")
def policy_test(body: PolicyTestBody):
    """Test policy: {actor, role, module, action, target} -> {allow, reason}."""
    ctx = ActorContext(actor=body.actor, role=body.role)
    decision = get_policy_engine().can(ctx, body.module, body.action, body.target)
    return {"allow": decision.allow, "reason": decision.reason, "details": decision.details}


@app.get("/audit/tail")
def audit_tail(n: int = 50, module: Optional[str] = None):
    """Last n audit log entries, optional filter by module."""
    entries = get_audit_logger().tail(n=n, module=module)
    if not entries and get_audit_logger()._db is None:
        return {"ok": True, "entries": [], "error": "AUDIT_DB_PATH not set"}
    return {"ok": True, "entries": entries, "error": None}


def _redact_tokens(text: str) -> str:
    """Redact token/secret-like substrings for support bundle."""
    import re
    if not text:
        return text
    for pattern in [r"Bearer\s+[A-Za-z0-9_\-\.]+", r"token[=:]\s*[\w\-]+", r"api_key[=:]\s*[\w\-]+", r"secret[=:]\s*[\w\-]+"]:
        text = re.sub(pattern, "***REDACTED***", text, flags=re.IGNORECASE)
    return text


@app.get("/support/bundle")
def support_bundle():
    """Export bundle: health, metrics, GA/metalearn/CI reports, deploy/gateway/cluster status, logs (redacted), config (no secrets). Returns {ok, data: {path}, ms, error}. Audited."""
    t0 = time.perf_counter()
    import zipfile
    import json
    from datetime import datetime, timezone
    support_dir = BASE_DIR / "snapshots" / "support"
    support_dir.mkdir(parents=True, exist_ok=True)
    ts = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    zip_path = support_dir / f"support_bundle_{ts}.zip"
    try:
        with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as zf:
            try:
                from modules.humanoid.deploy.healthcheck import run_health_verbose
                from modules.humanoid.deploy.ports import get_ports
                port = get_ports()[0] if get_ports() else 8791
                health = run_health_verbose(base_url=None, active_port=port)
                zf.writestr("health.json", json.dumps(health, indent=2))
            except Exception:
                zf.writestr("health.json", "{}")
            try:
                snap = get_metrics_store().snapshot()
                zf.writestr("metrics.json", json.dumps(snap, indent=2))
            except Exception:
                zf.writestr("metrics.json", "{}")
            log_file = BASE_DIR / "logs" / "atlas.log"
            if log_file.exists():
                try:
                    tail = log_file.read_text(encoding="utf-8", errors="replace")[-50000:]
                    zf.writestr("atlas_log_tail.txt", _redact_tokens(tail))
                except Exception:
                    pass
            svc_log = BASE_DIR / "logs" / "service.log"
            if svc_log.exists():
                try:
                    tail = svc_log.read_text(encoding="utf-8", errors="replace")[-20000:]
                    zf.writestr("service_log_tail.txt", _redact_tokens(tail))
                except Exception:
                    pass
            ci_dir = Path(os.getenv("CI_EXPORT_DIR", str(BASE_DIR / "snapshots" / "ci")))
            if ci_dir.exists():
                reports = sorted(ci_dir.glob("CI_REPORT_*.md"), key=lambda p: p.stat().st_mtime, reverse=True)
                if reports:
                    zf.write(reports[0], "ci_last_report.md")
            ga_dir = Path(os.getenv("GA_REPORT_DIR", str(BASE_DIR / "snapshots" / "ga")))
            if ga_dir.exists():
                reports = sorted(ga_dir.glob("GA_REPORT_*.md"), key=lambda p: p.stat().st_mtime, reverse=True)
                if reports:
                    zf.write(reports[0], "ga_last_report.md")
            meta_dir = Path(os.getenv("METALEARN_REPORT_DIR", str(BASE_DIR / "snapshots" / "metalearn")))
            if meta_dir.exists():
                reports = sorted(meta_dir.glob("META_REPORT_*.md"), key=lambda p: p.stat().st_mtime, reverse=True)
                if reports:
                    zf.write(reports[0], "metalearn_last_report.md")
            try:
                from modules.humanoid.deploy.switcher import get_deploy_state
                deploy = get_deploy_state()
                zf.writestr("deploy_status.json", json.dumps(deploy, indent=2))
            except Exception:
                zf.writestr("deploy_status.json", "{}")
            try:
                from modules.humanoid.gateway.store import build_gateway_status
                gw = build_gateway_status()
                zf.writestr("gateway_status.json", json.dumps(gw, indent=2))
            except Exception:
                zf.writestr("gateway_status.json", "{}")
            try:
                from modules.humanoid.cluster.registry import list_nodes
                nodes = list_nodes()
                zf.writestr("cluster_status.json", json.dumps({"nodes": nodes}, indent=2))
            except Exception:
                zf.writestr("cluster_status.json", "{}")
            env_file = BASE_DIR / "config" / "atlas.env"
            if env_file.exists():
                lines = []
                for line in env_file.read_text(encoding="utf-8", errors="replace").splitlines():
                    if "=" in line and any(s in line.lower() for s in ["secret", "password", "token", "key"]):
                        lines.append(line.split("=")[0] + "=***REDACTED***")
                    else:
                        lines.append(line)
                zf.writestr("config_redacted.env", "\n".join(lines))
        get_audit_logger().log_event("support", "api", "bundle", True, int((time.perf_counter() - t0) * 1000), None, {"path": str(zip_path)}, None)
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(True, {"path": str(zip_path)}, ms, None)
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        get_audit_logger().log_event("support", "api", "bundle", False, ms, str(e), None, None)
        return _std_resp(False, None, ms, str(e))


@app.get("/support/selfcheck")
def support_selfcheck():
    """Run quick diagnostics; return problems and suggestions. {ok, data: {problems, suggestions}, ms, error}."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.product.selfcheck import run_selfcheck
        result = run_selfcheck()
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(result.get("ok", True), {"problems": result.get("problems", []), "suggestions": result.get("suggestions", [])}, ms, None)
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, {"problems": [{"id": "selfcheck_error", "severity": "critical", "message": str(e), "suggestion": "Check logs"}], "suggestions": []}, ms, str(e))


# --- Approvals (policy + audit) ---
@app.get("/approvals/list")
def approvals_list(status: Optional[str] = None, risk: Optional[str] = None, limit: int = 50):
    """List approval items. ?status=pending&risk=high. Response: {ok, data: [items], ms, error}."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.approvals import list_all
        st = status if status else "pending"
        items = list_all(limit=limit, status=st, risk=risk)
        ms = int((time.perf_counter() - t0) * 1000)
        return {"ok": True, "data": [getattr(i, "__dict__", i) if hasattr(i, "__dict__") else i for i in items], "ms": ms, "error": None}
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return {"ok": False, "data": None, "ms": ms, "error": str(e)}


class ApprovalActionBody(BaseModel):
    id: Optional[str] = None
    approval_id: Optional[str] = None
    approve: Optional[bool] = True
    session_token: Optional[str] = None
    confirm_token: Optional[str] = None


@app.post("/approvals/approve")
def approvals_approve(body: ApprovalActionBody, x_owner_session: Optional[str] = Header(None, alias="X-Owner-Session")):
    """Approve item by id. Body: {id, approve?, session_token?, confirm_token?}. Risk>=medium require X-Owner-Session."""
    aid = body.id or body.approval_id
    if not aid:
        return {"ok": False, "id": None, "status": "missing_id", "error": "id or approval_id required"}
    token = body.session_token or x_owner_session
    try:
        from modules.humanoid.approvals import approve as approval_approve
        out = approval_approve(aid, resolved_by="api", owner_session_token=token, confirm_token=body.confirm_token)
        return {"ok": out.get("ok"), "id": aid, "status": out.get("status"), "error": out.get("error")}
    except Exception as e:
        return {"ok": False, "id": aid, "status": "error", "error": str(e)}


@app.get("/approvals/chain/verify")
def approvals_chain_verify():
    """Verify approval chain integrity. Response: {ok, valid, first_invalid_id?, expected_hash?, got_hash?}."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.approvals import verify_chain
        result = verify_chain()
        ms = int((time.perf_counter() - t0) * 1000)
        return {"ok": result.get("ok", True), "valid": result.get("valid", True), "ms": ms, "broken_at_id": result.get("broken_at_id"), "first_invalid_id": result.get("broken_at_id"), "last_hash": result.get("last_hash"), "count": result.get("count"), "error": None}
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return {"ok": False, "valid": False, "ms": ms, "first_invalid_id": None, "expected_hash": None, "got_hash": None, "error": str(e)}


# --- Owner (zero-trust session + emergency) ---
class OwnerSessionStartBody(BaseModel):
    actor: str
    method: Optional[str] = "ui"  # ui | telegram | voice | api


@app.post("/owner/session/start")
def owner_session_start(body: OwnerSessionStartBody):
    """Start owner session. Returns session_token (TTL). Required for risky approvals via X-Owner-Session."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.owner.session import start as owner_start
        out = owner_start(actor=body.actor or "owner", method=body.method or "ui")
        ms = int((time.perf_counter() - t0) * 1000)
        return {"ok": out.get("ok"), "session_token": out.get("session_token"), "ttl_seconds": out.get("ttl_seconds"), "method": out.get("method"), "expires_at": out.get("expires_at"), "ms": ms, "error": out.get("error")}
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return {"ok": False, "session_token": None, "ms": ms, "error": str(e)}


@app.get("/owner/windows-user")
def owner_windows_user(request: Request):
    """Usuario Windows actual (solo localhost). Para pre-llenar login."""
    if request.client and request.client.host not in ("127.0.0.1", "localhost", "::1"):
        return {"ok": False, "username": "", "error": "Solo localhost"}
    try:
        from modules.humanoid.owner.windows_auth import current_windows_user
        return {"ok": True, "username": current_windows_user(), "error": None}
    except Exception as e:
        return {"ok": False, "username": "", "error": str(e)}


class OwnerSessionStartWithFaceBody(BaseModel):
    image_base64: Optional[str] = None


@app.post("/owner/session/start-with-face")
def owner_session_start_with_face(body: OwnerSessionStartWithFaceBody):
    """Inicia sesión owner si se detecta rostro en la imagen."""
    t0 = time.perf_counter()
    try:
        import base64
        from modules.humanoid.face import face_check_image
        from modules.humanoid.owner.session import start as owner_start
        if not body.image_base64:
            return {"ok": False, "session_token": None, "error": "Imagen requerida", "ms": int((time.perf_counter() - t0) * 1000)}
        raw = base64.b64decode(body.image_base64)
        result = face_check_image(raw)
        faces = result.get("faces_detected", 0) or result.get("count", 0)
        if faces < 1:
            return {"ok": False, "session_token": None, "error": "No se detectó rostro", "ms": int((time.perf_counter() - t0) * 1000)}
        out = owner_start(actor="face", method="ui")
        ms = int((time.perf_counter() - t0) * 1000)
        return {"ok": out.get("ok"), "session_token": out.get("session_token"), "ttl_seconds": out.get("ttl_seconds"), "method": "face", "expires_at": out.get("expires_at"), "ms": ms, "error": out.get("error")}
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return {"ok": False, "session_token": None, "ms": ms, "error": str(e)}


class OwnerSessionStartWithWindowsBody(BaseModel):
    username: str = ""
    password: str = ""


@app.post("/owner/session/start-with-windows")
def owner_session_start_with_windows(body: OwnerSessionStartWithWindowsBody):
    """Inicia sesión owner si la contraseña de usuario Windows es correcta."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.owner.windows_auth import verify_windows_user
        from modules.humanoid.owner.session import start as owner_start
        ok, err = verify_windows_user(body.username.strip(), body.password)
        if not ok:
            return {"ok": False, "session_token": None, "error": err or "Credenciales incorrectas", "ms": int((time.perf_counter() - t0) * 1000)}
        out = owner_start(actor=body.username.strip(), method="ui")
        ms = int((time.perf_counter() - t0) * 1000)
        return {"ok": out.get("ok"), "session_token": out.get("session_token"), "ttl_seconds": out.get("ttl_seconds"), "method": "windows", "expires_at": out.get("expires_at"), "ms": ms, "error": out.get("error")}
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return {"ok": False, "session_token": None, "ms": ms, "error": str(e)}


class OwnerSessionEndBody(BaseModel):
    session_token: str


@app.post("/owner/session/end")
def owner_session_end(body: OwnerSessionEndBody):
    """End (revoke) owner session."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.owner.session import end as owner_end
        ok = owner_end(body.session_token)
        ms = int((time.perf_counter() - t0) * 1000)
        return {"ok": ok, "ms": ms, "error": None}
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return {"ok": False, "ms": ms, "error": str(e)}


@app.get("/owner/session/status")
def owner_session_status():
    """List active sessions (redacted tokens)."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.owner.session import list_active
        sessions = list_active()
        ms = int((time.perf_counter() - t0) * 1000)
        return {"ok": True, "sessions": sessions, "ms": ms, "error": None}
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return {"ok": False, "sessions": [], "ms": ms, "error": str(e)}


@app.get("/owner/emergency/status")
def owner_emergency_status():
    """Emergency mode status: enabled, block flags."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.owner.emergency import get_emergency_state
        data = get_emergency_state()
        ms = int((time.perf_counter() - t0) * 1000)
        return {"ok": True, "data": data, "ms": ms, "error": None}
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return {"ok": False, "data": None, "ms": ms, "error": str(e)}


class OwnerEmergencySetBody(BaseModel):
    enable: bool
    reason: Optional[str] = None


@app.post("/owner/emergency/set")
def owner_emergency_set(body: OwnerEmergencySetBody):
    """Enable/disable emergency mode. When enabled: block deploys, remote_exec, shell; allow status/health."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.owner.emergency import set_emergency, is_emergency
        set_emergency(body.enable, reason=body.reason or "")
        try:
            from modules.humanoid.audit import get_audit_logger
            get_audit_logger().log_event("owner", "api", "emergency_set", True, 0, None, {"enable": body.enable, "reason": body.reason}, None)
        except Exception:
            pass
        ms = int((time.perf_counter() - t0) * 1000)
        return {"ok": True, "emergency": is_emergency(), "ms": ms, "error": None}
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return {"ok": False, "emergency": None, "ms": ms, "error": str(e)}


class OwnerEmergencyBody(BaseModel):
    enable: bool


@app.post("/owner/emergency")
def owner_emergency(body: OwnerEmergencyBody):
    """Enable/disable emergency mode (legacy). Prefer POST /owner/emergency/set."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.owner.emergency import set_emergency, is_emergency
        set_emergency(body.enable)
        try:
            from modules.humanoid.audit import get_audit_logger
            get_audit_logger().log_event("owner", "api", "emergency", True, 0, None, {"enable": body.enable}, None)
        except Exception:
            pass
        ms = int((time.perf_counter() - t0) * 1000)
        return {"ok": True, "emergency": is_emergency(), "ms": ms, "error": None}
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return {"ok": False, "emergency": None, "ms": ms, "error": str(e)}


def _owner_status_data():
    """Aggregate owner console data: sessions, pending, expired, emergency, chain, critical history."""
    from modules.humanoid.owner.session import list_active
    from modules.humanoid.owner.emergency import is_emergency
    from modules.humanoid.approvals import list_pending, verify_chain
    from modules.humanoid.approvals.store import list_items
    from datetime import datetime, timezone
    sessions = list_active()
    pending = list_pending(limit=50)
    all_items = list_items(status=None, limit=100)
    expired = []
    for it in all_items:
        if it.get("status") != "pending":
            continue
        et = it.get("expires_at")
        if not et:
            continue
        try:
            dt = datetime.fromisoformat(et.replace("Z", "+00:00"))
            if dt.tzinfo is None:
                dt = dt.replace(tzinfo=timezone.utc)
            if datetime.now(timezone.utc) >= dt:
                expired.append(it)
        except Exception:
            pass
    chain = verify_chain()
    critical_history = [x for x in all_items if x.get("status") == "approved" and (x.get("risk") or "").strip().lower() == "critical"][:20]
    return {
        "active_sessions": sessions,
        "pending_approvals": pending,
        "expired_approvals": expired,
        "emergency": is_emergency(),
        "chain_valid": chain.get("valid", False),
        "chain_ok": chain.get("ok", False),
        "critical_history": critical_history,
    }


@app.get("/owner/status")
def owner_status():
    """Owner console: active sessions, pending/expired approvals, emergency, chain integrity, critical history."""
    t0 = time.perf_counter()
    try:
        data = _owner_status_data()
        ms = int((time.perf_counter() - t0) * 1000)
        return {"ok": True, "data": data, "ms": ms, "error": None}
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return {"ok": False, "data": None, "ms": ms, "error": str(e)}


class VoiceApproveBody(BaseModel):
    phrase: str
    session_token: Optional[str] = None


@app.post("/voice/approve")
def voice_approve(body: VoiceApproveBody):
    """Voice approval: 'aprobar <id>' -> 'di confirmar <id>'; 'confirmar <id>' -> approve. Replay-safe."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.owner.voice_approval import handle_voice_command, voice_approval_enabled
        if not voice_approval_enabled():
            return {"ok": False, "response": "", "approved": False, "ms": int((time.perf_counter() - t0) * 1000), "error": "voice_approvals disabled"}
        response, approved = handle_voice_command(body.phrase or "", device_source="api")
        ms = int((time.perf_counter() - t0) * 1000)
        return {"ok": True, "response": response, "approved": approved, "ms": ms, "error": None}
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return {"ok": False, "response": "", "approved": False, "ms": ms, "error": str(e)}


class OwnerVoiceCommandBody(BaseModel):
    transcript: str
    device_source: Optional[str] = None

@app.post("/owner/voice/command")
def owner_voice_command(body: OwnerVoiceCommandBody):
    """Voice approval: 'aprobar <id>' -> confirm; 'confirmar <id>' -> approve. Returns {response, approved}."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.owner.voice_approval import handle_voice_command
        response, approved = handle_voice_command(body.transcript or "", body.device_source)
        ms = int((time.perf_counter() - t0) * 1000)
        return {"ok": True, "response": response, "approved": approved, "ms": ms, "error": None}
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return {"ok": False, "response": "", "approved": False, "ms": ms, "error": str(e)}


class OwnerTelegramCallbackBody(BaseModel):
    callback_data: str   # approve:ID | reject:ID
    chat_id: str

@app.post("/owner/telegram/callback")
def owner_telegram_callback(body: OwnerTelegramCallbackBody):
    """Handle Telegram inline callback (bot sends callback_data + chat_id). Returns {ok, action, id, error}."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.comms.telegram_bridge import TelegramBridge
        out = TelegramBridge().handle_callback_data(body.callback_data or "", body.chat_id or "")
        ms = int((time.perf_counter() - t0) * 1000)
        return {"ok": out.get("ok"), "action": out.get("action"), "id": out.get("id"), "ms": ms, "error": out.get("error")}
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return {"ok": False, "action": None, "id": None, "ms": ms, "error": str(e)}


@app.post("/approvals/reject")
def approvals_reject(body: ApprovalActionBody):
    """Reject item by id. Body: {id} or {approval_id}. Audited."""
    aid = body.id or body.approval_id
    if not aid:
        return {"ok": False, "id": None, "status": "missing_id", "error": "id or approval_id required"}
    try:
        from modules.humanoid.approvals import reject as approval_reject
        out = approval_reject(aid, resolved_by="api")
        return {"ok": out.get("ok"), "id": aid, "status": out.get("status"), "error": None}
    except Exception as e:
        return {"ok": False, "id": aid, "status": "error", "error": str(e)}


# --- Scheduler / Watchdog / Healing ---
def _std_resp(ok: bool, data: Any = None, ms: int = 0, error: Optional[str] = None) -> dict:
    out = {"ok": ok, "data": data, "ms": ms, "error": error}
    return out


@app.get("/scheduler/jobs")
def scheduler_jobs(status: Optional[str] = None):
    """List jobs, optional filter by status. Response: {ok, data, ms, error}."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.scheduler import get_scheduler_db
        db = get_scheduler_db()
        jobs = db.list_jobs(status=status)
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(True, jobs, ms, None)
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


class SchedulerJobCreateBody(BaseModel):
    name: str
    kind: str  # update_check | llm_plan | shell_command | custom
    payload: dict = {}
    run_at: Optional[str] = None
    interval_seconds: Optional[int] = None
    max_retries: Optional[int] = None
    backoff_seconds: Optional[int] = None


@app.post("/scheduler/job/create")
def scheduler_job_create(body: SchedulerJobCreateBody):
    """Create a job. run_at ISO or now; interval_seconds for recurring. Response: {ok, data, ms, error}."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.scheduler import JobSpec, get_scheduler_db
        import os
        max_r = int(os.getenv("SCHED_DEFAULT_RETRIES", "3") or 3) if body.max_retries is None else body.max_retries
        backoff = int(os.getenv("SCHED_BACKOFF_SECONDS", "5") or 5) if body.backoff_seconds is None else body.backoff_seconds
        spec = JobSpec(
            name=body.name,
            kind=body.kind,
            payload=body.payload,
            run_at=body.run_at,
            interval_seconds=body.interval_seconds,
            max_retries=max_r,
            backoff_seconds=backoff,
        )
        db = get_scheduler_db()
        job = db.create_job(spec)
        get_audit_logger().log_event("api", "owner", "scheduler", "job_enqueue", True, 0, None, {"job_id": job.get("id"), "name": body.name, "kind": body.kind}, None)
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(True, job, ms, None)
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


class SchedulerJobIdBody(BaseModel):
    job_id: str


@app.post("/scheduler/job/pause")
def scheduler_job_pause(body: SchedulerJobIdBody):
    t0 = time.perf_counter()
    try:
        from modules.humanoid.scheduler import get_scheduler_db
        db = get_scheduler_db()
        j = db.get_job(body.job_id)
        if not j:
            return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), "job not found")
        db.set_paused(body.job_id)
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(True, {"job_id": body.job_id, "status": "paused"}, ms, None)
    except Exception as e:
        return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), str(e))


@app.post("/scheduler/job/resume")
def scheduler_job_resume(body: SchedulerJobIdBody):
    t0 = time.perf_counter()
    try:
        from modules.humanoid.scheduler import get_scheduler_db
        from datetime import datetime, timezone
        db = get_scheduler_db()
        j = db.get_job(body.job_id)
        if not j:
            return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), "job not found")
        now = datetime.now(timezone.utc).isoformat()
        db.set_queued(body.job_id, now)
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(True, {"job_id": body.job_id, "status": "queued", "next_run_ts": now}, ms, None)
    except Exception as e:
        return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), str(e))


@app.post("/scheduler/job/run-now")
def scheduler_job_run_now(body: SchedulerJobIdBody):
    """Schedule job to run immediately (set next_run_ts to now)."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.scheduler import get_scheduler_db
        from datetime import datetime, timezone
        db = get_scheduler_db()
        j = db.get_job(body.job_id)
        if not j:
            return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), "job not found")
        now = datetime.now(timezone.utc).isoformat()
        db.set_queued(body.job_id, now)
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(True, {"job_id": body.job_id, "status": "queued", "next_run_ts": now}, ms, None)
    except Exception as e:
        return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), str(e))


@app.get("/scheduler/job/runs")
def scheduler_job_runs(job_id: str, limit: int = 50):
    t0 = time.perf_counter()
    try:
        from modules.humanoid.scheduler import get_scheduler_db
        db = get_scheduler_db()
        runs = db.get_runs(job_id, limit=limit)
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(True, runs, ms, None)
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


@app.get("/update/status")
def update_status_endpoint():
    """Git-based update status: branch, head, remote, has_update. Response: {ok, data, ms, error}."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.update.update_engine import status as update_status
        data = update_status()
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(data.get("ok", True), data, ms, data.get("error"))
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


@app.post("/update/check")
def update_check_endpoint():
    """Update check: fetch, snapshot. No apply. Response: {ok, data, ms, error}."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.update.update_engine import check as update_check
        result = update_check()
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(result.get("ok", False), result.get("data"), ms, result.get("error"))
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


@app.post("/update/apply")
def update_apply_endpoint():
    """Git update apply: staging -> smoke -> promote or rollback. Policy must allow. Response: {ok, data, ms, error}."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.owner.emergency import is_emergency
        if is_emergency():
            return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), "emergency mode: updates suspended")
        from modules.humanoid.update.update_engine import apply as update_apply
        result = update_apply()
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(result.get("ok", False), result.get("data"), ms, result.get("error"))
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


@app.get("/deploy/status", tags=["Deploy"])
def deploy_status():
    """Deploy status: ok, mode, active_port, staging_port, active_pid, staging_pid, last_deploy, last_health, canary."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.deploy.status import build_deploy_status
        from modules.humanoid.deploy.canary import should_disable_canary, set_canary_disabled_fallback
        from modules.humanoid.deploy.switcher import get_deploy_state
        state = get_deploy_state()
        if should_disable_canary() and not state.get("canary_disabled_fallback"):
            set_canary_disabled_fallback(True)
        data = build_deploy_status()
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(True, data, ms, None)
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


class DeployApplyBody(BaseModel):
    ref: Optional[str] = None   # origin/main | tag:vX.Y.Z | sha:....
    mode: Optional[str] = None  # bluegreen | single
    force: Optional[bool] = False


@app.post("/deploy/apply", tags=["Deploy"])
def deploy_apply(body: Optional[DeployApplyBody] = None):
    """Apply deploy: policy gate deploy_apply, optional fetch+staging ref, then blue-green flow (staging -> smoke -> health -> promote or rollback)."""
    t0 = time.perf_counter()
    body = body or DeployApplyBody()
    try:
        from modules.humanoid.owner.emergency import is_emergency
        if is_emergency():
            return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), "emergency mode: deploys suspended")
        decision = get_policy_engine().can(_memory_actor(), "deploy", "deploy_apply")
        if not decision.allow:
            return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), decision.reason or "policy denied")
        ref = body.ref or "origin/main"
        if body.mode and body.mode.strip().lower() == "single":
            return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), "mode=single not implemented for apply")
        from modules.humanoid.update.git_manager import fetch, create_staging_branch
        from modules.humanoid.deploy.bluegreen import run_bluegreen_flow
        r_fetch = fetch(os.getenv("UPDATE_REMOTE", "origin"))
        if not r_fetch.get("ok"):
            return _std_resp(False, {"step": "fetch", "error": r_fetch.get("error")}, int((time.perf_counter() - t0) * 1000), r_fetch.get("error"))
        remote = os.getenv("UPDATE_REMOTE", "origin")
        branch = (ref.replace("origin/", "").strip() if ref.startswith("origin/") else "main")
        staging_name = os.getenv("UPDATE_STAGING_BRANCH", "staging").strip() or "staging"
        r_staging = create_staging_branch(remote=remote, branch=branch, staging_name=staging_name)
        if not r_staging.get("ok"):
            return _std_resp(False, {"step": "create_staging", "error": r_staging.get("error")}, int((time.perf_counter() - t0) * 1000), r_staging.get("error"))
        result = run_bluegreen_flow(ref=ref, dry_run=False)
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(result.get("ok", False), result.get("data"), ms, result.get("error"))
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


@app.post("/deploy/bluegreen", tags=["Deploy"])
def deploy_bluegreen(dry_run: bool = False):
    """Run blue-green flow: launch staging, smoke, healthcheck x3, switch or rollback. Use dry_run=true to simulate without switching."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.owner.emergency import is_emergency
        if is_emergency():
            return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), "emergency mode: deploys suspended")
        from modules.humanoid.deploy import run_bluegreen_flow
        from modules.humanoid.metrics import get_metrics_store
        get_metrics_store().inc("deploy_bluegreen_runs")
        result = run_bluegreen_flow(dry_run=dry_run)
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(result.get("ok", False), result.get("data"), ms, result.get("error"))
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


@app.get("/canary/status", tags=["Deploy"])
def canary_status():
    """Canary status: enabled, percentage, features, stats (calls, error rate, latency)."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.deploy.canary import get_canary_stats
        data = get_canary_stats()
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(True, data, ms, None)
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


class CanaryConfigBody(BaseModel):
    enabled: Optional[bool] = None
    percentage: Optional[float] = None
    features: Optional[List[str]] = None


@app.post("/canary/config", tags=["Deploy"])
def canary_config(body: Optional[CanaryConfigBody] = None):
    """Runtime canary config (owner only, policy-gated). Body: enabled?, percentage?, features?."""
    t0 = time.perf_counter()
    body = body or CanaryConfigBody()
    try:
        decision = get_policy_engine().can(_memory_actor(), "deploy", "canary_config")
        if not decision.allow:
            return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), decision.reason or "policy denied")
        from modules.humanoid.deploy.canary import set_canary_config, get_canary_stats
        set_canary_config(enabled=body.enabled, percentage=body.percentage, features=body.features)
        data = get_canary_stats()
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(True, data, ms, None)
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


@app.get("/deploy/canary/report", tags=["Deploy"])
def deploy_canary_report(hours: float = 24.0):
    """Canary vs stable comparison over the last N hours: counts, error rate, avg latency."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.deploy.canary import get_canary_report
        data = get_canary_report(hours=hours)
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(True, data, ms, None)
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


# --- Cluster (multi-node) ---
@app.get("/cluster/status", tags=["Cluster"])
def cluster_status():
    """Cluster status: enabled, node_id, role, nodes count."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.cluster import registry
        if not registry.cluster_enabled():
            return _std_resp(True, {"enabled": False, "node_id": registry.node_id(), "role": registry.node_role(), "nodes": []}, int((time.perf_counter() - t0) * 1000), None)
        nodes = registry.list_nodes()
        data = {"enabled": True, "node_id": registry.node_id(), "role": registry.node_role(), "nodes": nodes, "count": len(nodes)}
        return _std_resp(True, data, int((time.perf_counter() - t0) * 1000), None)
    except Exception as e:
        return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), str(e))


@app.get("/cluster/nodes", tags=["Cluster"])
def cluster_nodes(status_filter: Optional[str] = None):
    """List registered nodes (optionally filter by status). Marks stale nodes offline."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.cluster import registry
        registry.mark_offline_stale()
        nodes = registry.list_nodes(status_filter=status_filter)
        return _std_resp(True, nodes, int((time.perf_counter() - t0) * 1000), None)
    except Exception as e:
        return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), str(e))


@app.get("/cluster/events", tags=["Cluster"])
def cluster_events(limit: int = 50, node_id_filter: Optional[str] = None):
    """Last node_events for UI (register, heartbeat, offline, route, error)."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.cluster import db as cluster_db
        events = cluster_db.get_events(limit=limit, node_id=node_id_filter)
        return _std_resp(True, events, int((time.perf_counter() - t0) * 1000), None)
    except Exception as e:
        return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), str(e))


class ClusterRegisterBody(BaseModel):
    node_id: str
    role: str = "worker"
    base_url: str
    capabilities: Optional[dict] = None
    tags: Optional[dict] = None


@app.post("/cluster/node/register", tags=["Cluster"])
def cluster_node_register(body: ClusterRegisterBody):
    """Register or update a node (policy: remote_execute or owner)."""
    t0 = time.perf_counter()
    try:
        decision = get_policy_engine().can(_memory_actor(), "cluster", "remote_execute")
        if not decision.allow:
            return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), decision.reason or "policy denied")
        from modules.humanoid.cluster import registry
        result = registry.register_node(body.node_id, body.role, body.base_url, body.capabilities, body.tags)
        return _std_resp(result.get("ok", False), result, int((time.perf_counter() - t0) * 1000), result.get("error"))
    except Exception as e:
        return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), str(e))


class ClusterHeartbeatBody(BaseModel):
    node_id: str
    capabilities: dict = {}
    health: dict = {}
    version: str = ""
    channel: str = "canary"
    base_url: Optional[str] = None


@app.post("/cluster/heartbeat", tags=["Cluster"])
def cluster_heartbeat(body: ClusterHeartbeatBody, request: Request = None):
    """HQ: receive heartbeat from worker. Registers node if new (base_url from request)."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.cluster.heartbeat import receive_heartbeat
        result = receive_heartbeat(body.node_id, body.capabilities, body.health, body.version, body.channel, base_url=body.base_url)
        return _std_resp(result.get("ok", False), result, int((time.perf_counter() - t0) * 1000), result.get("error"))
    except Exception as e:
        return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), str(e))


@app.get("/cluster/ping", tags=["Cluster"])
def cluster_ping():
    """Ping: respond 200 if alive (for HQ->worker check)."""
    return _std_resp(True, {"pong": True}, 0, None)


class ClusterRouteBody(BaseModel):
    task: str = "hands"
    prefer_remote: bool = False
    require_capability: Optional[str] = None


@app.post("/cluster/route/decision", tags=["Cluster"])
def cluster_route_decision(body: ClusterRouteBody):
    """Debug: get routing decision for a task."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.cluster.router import route_decision
        decision = route_decision(body.task, prefer_remote=body.prefer_remote, require_capability=body.require_capability)
        return _std_resp(True, decision, int((time.perf_counter() - t0) * 1000), None)
    except Exception as e:
        return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), str(e))


# --- Gateway (Stealth multi-path) ---
@app.get("/gateway/status", tags=["Gateway"])
def gateway_status():
    """Gateway status: enabled, mode, tools detected, last success, recommendations."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.gateway.store import build_gateway_status
        data = build_gateway_status()
        return _std_resp(True, data, int((time.perf_counter() - t0) * 1000), None)
    except Exception as e:
        return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), str(e))


@app.post("/gateway/check", tags=["Gateway"])
def gateway_check():
    """Check gateway candidates (policy: gateway_check)."""
    t0 = time.perf_counter()
    try:
        decision = get_policy_engine().can(_memory_actor(), "gateway", "gateway_check")
        if not decision.allow:
            return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), decision.reason or "policy denied")
        from modules.humanoid.gateway.selector import check_candidates
        data = check_candidates()
        return _std_resp(True, data, int((time.perf_counter() - t0) * 1000), None)
    except Exception as e:
        return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), str(e))


@app.post("/gateway/bootstrap", tags=["Gateway"])
def gateway_bootstrap():
    """Bootstrap worker to HQ (policy: gateway_bootstrap). Returns quickly if no_gateway_config."""
    t0 = time.perf_counter()
    try:
        decision = get_policy_engine().can(_memory_actor(), "gateway", "gateway_bootstrap")
        if not decision.allow:
            return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), decision.reason or "policy denied")
        from modules.humanoid.gateway import bootstrap as gw_bootstrap
        from modules.humanoid.gateway import selector as gw_selector
        target = gw_selector.resolve_worker_url(None)
        if not target:
            return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), "no_gateway_config")
        result = gw_bootstrap.bootstrap()
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(result.get("ok", False), result, ms, result.get("error"))
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


class GatewayModeBody(BaseModel):
    mode: str  # auto | cloudflare | tailscale | ssh | lan


@app.post("/gateway/mode", tags=["Gateway"])
def gateway_set_mode(body: GatewayModeBody):
    """Set gateway mode (policy: gateway_set_mode)."""
    t0 = time.perf_counter()
    try:
        decision = get_policy_engine().can(_memory_actor(), "gateway", "gateway_set_mode")
        if not decision.allow:
            return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), decision.reason or "policy denied")
        from modules.humanoid.gateway import store as gateway_store
        gateway_store.set_mode(body.mode)
        data = gateway_store.build_gateway_status()
        return _std_resp(True, data, int((time.perf_counter() - t0) * 1000), None)
    except Exception as e:
        return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), str(e))


def _cluster_remote_result(ok: bool, data: Any = None, ms: int = 0, error: Optional[str] = None, correlation_id: Optional[str] = None) -> dict:
    out = {"ok": ok, "data": data, "ms": ms, "error": error}
    if correlation_id:
        out["correlation_id"] = correlation_id
    return out


@app.post("/remote/hands", tags=["Cluster"])
def remote_hands(body: dict):
    """Worker: execute hands (shell). Returns {ok, data, ms, error, correlation_id}."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.owner.emergency import is_action_blocked
        if is_action_blocked("remote_hands"):
            return _cluster_remote_result(False, None, int((time.perf_counter() - t0) * 1000), "emergency_mode_block", body.get("correlation_id"))
        from modules.humanoid.cluster import trace
        from modules.humanoid.cluster.remote_server import execute_remote_hands
        cid = body.get("correlation_id") or trace.new_correlation_id()
        r = execute_remote_hands(body)
        ms = int((time.perf_counter() - t0) * 1000)
        return _cluster_remote_result(r.get("ok", False), r.get("data"), ms, r.get("error"), cid)
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _cluster_remote_result(False, None, ms, str(e), body.get("correlation_id"))


@app.post("/remote/web", tags=["Cluster"])
def remote_web(body: dict):
    """Worker: execute web action."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.owner.emergency import is_action_blocked
        if is_action_blocked("remote_web"):
            return _cluster_remote_result(False, None, int((time.perf_counter() - t0) * 1000), "emergency_mode_block", body.get("correlation_id"))
        from modules.humanoid.cluster import trace
        from modules.humanoid.cluster.remote_server import execute_remote_web
        cid = body.get("correlation_id") or trace.new_correlation_id()
        r = execute_remote_web(body)
        ms = int((time.perf_counter() - t0) * 1000)
        return _cluster_remote_result(r.get("ok", False), r.get("data"), ms, r.get("error"), cid)
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _cluster_remote_result(False, None, ms, str(e), body.get("correlation_id"))


@app.post("/remote/vision", tags=["Cluster"])
def remote_vision(body: dict):
    """Worker: execute vision (analyze image)."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.owner.emergency import is_action_blocked
        if is_action_blocked("remote_vision"):
            return _cluster_remote_result(False, None, int((time.perf_counter() - t0) * 1000), "emergency_mode_block", body.get("correlation_id"))
        from modules.humanoid.cluster import trace
        from modules.humanoid.cluster.remote_server import execute_remote_vision
        cid = body.get("correlation_id") or trace.new_correlation_id()
        r = execute_remote_vision(body)
        ms = int((time.perf_counter() - t0) * 1000)
        return _cluster_remote_result(r.get("ok", False), r.get("data"), ms, r.get("error"), cid)
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _cluster_remote_result(False, None, ms, str(e), body.get("correlation_id"))


@app.post("/remote/voice", tags=["Cluster"])
def remote_voice(body: dict):
    """Worker: execute voice action."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.owner.emergency import is_action_blocked
        if is_action_blocked("remote_voice"):
            return _cluster_remote_result(False, None, int((time.perf_counter() - t0) * 1000), "emergency_mode_block", body.get("correlation_id"))
        from modules.humanoid.cluster import trace
        from modules.humanoid.cluster.remote_server import execute_remote_voice
        cid = body.get("correlation_id") or trace.new_correlation_id()
        r = execute_remote_voice(body)
        ms = int((time.perf_counter() - t0) * 1000)
        return _cluster_remote_result(r.get("ok", False), r.get("data"), ms, r.get("error"), cid)
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _cluster_remote_result(False, None, ms, str(e), body.get("correlation_id"))


@app.get("/watchdog/status")
def watchdog_status_endpoint():
    t0 = time.perf_counter()
    try:
        from modules.humanoid.watchdog import watchdog_status
        data = watchdog_status()
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(True, data, ms, None)
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


@app.get("/healing/status")
def healing_status_endpoint():
    t0 = time.perf_counter()
    try:
        from modules.humanoid.healing import healing_status
        data = healing_status()
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(True, data, ms, None)
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


# --- Agent / Scaffold / Scripts / Web / Voice / Deps ---

class AgentGoalBody(BaseModel):
    goal: str
    mode: str = "plan_only"  # plan_only | controlled | auto
    fast: Optional[bool] = True
    depth: Optional[int] = 1  # 1-5: multi-agent depth (1=single planner, 2+=Executive pipeline)


def _agent_goal_mode(mode: str) -> str:
    if mode in ("execute_controlled", "controlled"):
        return "execute_controlled"
    if mode in ("execute", "execute_auto", "auto"):
        return "execute"
    return "plan_only"


@app.post("/agent/goal")
def agent_goal(body: AgentGoalBody):
    """Run goal: plan_only | controlled | auto. depth 1-5: multi-agent pipeline. Respuesta: resumen, data, pipeline, siguientes_pasos."""
    t0 = time.perf_counter()
    mode = _agent_goal_mode(body.mode or "plan_only")
    depth = max(1, min(5, body.depth or 1))
    try:
        if depth >= 2:
            from modules.humanoid.agents import run_multi_agent_goal
            from modules.humanoid.orchestrator import run_goal_with_plan
            ma = run_multi_agent_goal(body.goal, depth=depth, mode=mode)
            if not ma.get("ok"):
                ms = int((time.perf_counter() - t0) * 1000)
                return _professional_resp(False, ma, ms, ma.get("error"), resumen=ma.get("error") or "Multi-agent failed")
            if ma.get("decision") == "replan":
                ms = int((time.perf_counter() - t0) * 1000)
                data = {k: v for k, v in ma.items() if k not in ("ok", "error")}
                return _professional_resp(True, data, ms, None, resumen="Reviewer rechazó; replan sugerido", siguientes_pasos=["Ajuste objetivo o replan"])
            steps_raw = ma.get("steps") or []
            steps_list = [s.get("description", s) if isinstance(s, dict) else str(s) for s in steps_raw]
            if mode != "plan_only" and steps_list and ma.get("decision") == "approve":
                result = run_goal_with_plan(body.goal, steps_list, mode=mode)
            else:
                result = {"ok": True, "plan": ma.get("plan"), "steps": steps_raw, "task_id": ma.get("task_id"), "execution_log": [], "artifacts": [], "pipeline": ma.get("pipeline", [])}
        else:
            from modules.humanoid.orchestrator import run_goal
            result = run_goal(body.goal, mode=mode, fast=body.fast if body.fast is not None else True)
        ms = int((time.perf_counter() - t0) * 1000)
        ok = result.get("ok", False)
        data = {k: v for k, v in result.items() if k not in ("ok", "error", "fallback", "message")}
        steps_count = len(result.get("steps") or [])
        exec_log = result.get("execution_log") or []
        done = len([e for e in exec_log if e.get("status") == "success"])
        resumen = f"Goal: {steps_count} pasos, {done} ejecutados" if ok else (result.get("message") or result.get("error") or "Error")
        archivos = result.get("artifacts") or []
        siguientes = []
        if mode == "execute_controlled":
            siguientes = ["Ejecute cada paso vía POST /agent/step/execute con approve=true"]
        elif result.get("message"):
            siguientes = [result.get("message")]
        out = _professional_resp(ok, data, ms, result.get("error"), resumen=resumen, archivos=archivos if archivos else None, siguientes_pasos=siguientes if siguientes else None)
        if result.get("fallback"):
            out["fallback"] = True
            out["message"] = result.get("message")
        return out
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


class AgentStepBody(BaseModel):
    task_id: str
    step_id: str
    approve: bool = False


@app.post("/agent/step/execute")
def agent_step_execute(body: AgentStepBody):
    """Execute one step after approval. Returns {ok, data: {result, artifacts}, ms, error}."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.orchestrator import execute_step
        result = execute_step(body.task_id, body.step_id, approve=body.approve)
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(result.get("ok", False), {"result": result.get("result"), "artifacts": result.get("artifacts", [])}, ms, result.get("error"))
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


@app.post("/agent/benchmark")
def agent_benchmark():
    """Competitive intelligence: architecture, latencies, complexity, maintainability, risk, scalability. Strengths, Weaknesses, Gaps, Improvement map."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.metrics import get_metrics_store
        store = get_metrics_store()
        snap = store.snapshot()
        counters = snap.get("counters") or {}
        latencies = snap.get("latencies") or {}
        avg_lat = {k: v.get("avg_ms", 0) for k, v in latencies.items()}
        strengths = [
            "Multi-agent pipeline (Executive, Strategist, Architect, Engineer, Reviewer, Optimizer)",
            "Policy + Audit on all executable paths",
            "Persistent memory (SQLite) + FTS fallback",
            "Orchestrator with critic and replan",
        ]
        weaknesses = []
        if not latencies:
            weaknesses.append("Pocas métricas de latencia registradas")
        gaps = ["Embeddings opcional pendiente", "Adaptive routing por modelo en refinamiento"]
        improvement_map = [
            "Aumentar depth en /agent/goal para tareas complejas",
            "Revisar /agent/system-intel para cuellos de botella",
        ]
        data = {
            "strengths": strengths,
            "weaknesses": weaknesses,
            "gaps": gaps,
            "improvement_map": improvement_map,
            "metrics": {"counters": counters, "latency_avg_ms": avg_lat},
        }
        ms = int((time.perf_counter() - t0) * 1000)
        return _professional_resp(True, data, ms, None, resumen=f"Benchmark: {len(strengths)} strengths, {len(gaps)} gaps")
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


@app.get("/agent/system-intel")
@app.post("/agent/system-intel")
def agent_system_intel():
    """Performance intelligence: analyze logs, repeated errors, bottlenecks, optimization suggestions."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.audit import get_audit_logger
        from modules.humanoid.metrics import get_metrics_store
        entries = get_audit_logger().tail(n=100, module=None)
        store = get_metrics_store()
        snap = store.snapshot()
        errors = [e for e in (entries or []) if not e.get("ok")]
        error_msgs = {}
        for e in errors:
            msg = (e.get("error") or "unknown")[:80]
            error_msgs[msg] = error_msgs.get(msg, 0) + 1
        repeated = [{"error": k, "count": v} for k, v in error_msgs.items() if v >= 2][:10]
        latencies = snap.get("latencies") or {}
        bottlenecks = []
        for name, stat in latencies.items():
            avg = stat.get("avg_ms", 0)
            if avg > 5000:
                bottlenecks.append({"path": name, "avg_ms": round(avg, 0)})
        suggestions = []
        if repeated:
            suggestions.append("Revisar errores repetidos y añadir fallback o retry")
        if bottlenecks:
            suggestions.append("Revisar endpoints con latencia >5s para optimizar o cache")
        data = {
            "repeated_errors": repeated,
            "bottlenecks": bottlenecks[:10],
            "suggestions": suggestions or ["Sistema estable; sin sugerencias urgentes"],
            "audit_sample_size": len(entries or []),
        }
        ms = int((time.perf_counter() - t0) * 1000)
        return _professional_resp(True, data, ms, None, resumen=f"System intel: {len(repeated)} errores repetidos, {len(bottlenecks)} cuellos")
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


# --- Continuous Improvement ---
class AgentImproveBody(BaseModel):
    scope: str = "all"  # repo | runtime | all
    mode: str = "plan_only"  # plan_only | controlled | auto
    depth: Optional[int] = 2
    max_items: Optional[int] = None


@app.post("/agent/improve")
def agent_improve(body: AgentImproveBody):
    """CI cycle: scan -> plan -> optional execute. scope=repo|runtime|all. mode=plan_only|controlled|auto. Timeout-safe."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.ci import run_improve
        result = run_improve(
            scope=body.scope or "all",
            mode=body.mode or "plan_only",
            depth=max(1, min(5, body.depth or 2)),
            max_items=body.max_items,
        )
        ms = int((time.perf_counter() - t0) * 1000)
        if not result.get("ok"):
            return _std_resp(False, None, ms, result.get("error"))
        data = result.get("data") or {}
        return _professional_resp(
            True,
            {
                "findings": data.get("findings", []),
                "plan": data.get("plan", {}),
                "approvals_required": data.get("approvals_required", []),
                "auto_executed": data.get("auto_executed", []),
                "artifacts": data.get("artifacts", []),
                "report": data.get("report", {}),
            },
            ms,
            None,
            resumen=f"CI: {len(data.get('findings') or [])} findings, {len(data.get('approvals_required') or [])} pending approval",
        )
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


@app.get("/agent/improve/status")
def agent_improve_status():
    """Last CI cycle: plan_id, timestamp, executed, pending. Links to memory/audit."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.ci import get_improve_status
        data = get_improve_status()
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(True, data, ms, None)
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


class AgentImproveApplyBody(BaseModel):
    plan_id: str = ""
    approve: bool = False
    items: Optional[list] = None  # item_ids to apply


@app.post("/agent/improve/apply")
def agent_improve_apply(body: AgentImproveApplyBody):
    """Execute only approved items allowed by policy. plan_id must match last run."""
    t0 = time.perf_counter()
    if not body.approve:
        return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), "approve must be true")
    try:
        from modules.humanoid.ci import get_last_plan, execute_plan
        plan = get_last_plan()
        if not plan:
            return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), "no previous plan; run POST /agent/improve first")
        if body.plan_id and plan.get("plan_id") != body.plan_id:
            return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), "plan_id does not match last cycle")
        result = execute_plan(plan)
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(result.get("ok", True), {"executed": result.get("executed"), "errors": result.get("errors")}, ms, result.get("errors") and result["errors"][0] or None)
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


class ScaffoldBody(BaseModel):
    type: str = "fastapi"  # fastapi | pwa | flutter | node
    name: str
    options: Optional[dict] = None


@app.post("/scaffold/app")
def scaffold_app(body: ScaffoldBody):
    """Generate app structure + RUNBOOK. Returns {ok, data: {tree, files_created, runbook_path}, ms, error}."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.scaffolder import generate
        result = generate(body.type, body.name, options=body.options or {})
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(result.get("ok", False), {k: v for k, v in result.items() if k != "ok"}, ms, result.get("error"))
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


class ScriptsGenerateBody(BaseModel):
    kind: str = "powershell"  # powershell | python
    purpose: str
    options: Optional[dict] = None


@app.post("/scripts/generate")
def scripts_generate(body: ScriptsGenerateBody):
    """Generate script. Returns {ok, data: {path, preview, how_to_run}, ms, error}."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.scripts import generate_script
        from modules.humanoid import get_humanoid_kernel
        k = get_humanoid_kernel()
        hands = k.registry.get("hands")
        fs = getattr(hands, "fs", None) if hands else None
        result = generate_script(body.kind, body.purpose, options=body.options or {}, fs_controller=fs)
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(result.get("ok", False), {"path": result.get("path"), "preview": result.get("preview"), "how_to_run": result.get("how_to_run"), "validated": result.get("validated")}, ms, result.get("error"))
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


@app.post("/web/session/start")
def web_session_start():
    """Start browser session (no-op if Playwright not installed). Returns {ok, data, ms, error}."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.web import status as web_status
        data = web_status()
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(True, data, ms, None)
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


class WebNavigateBody(BaseModel):
    url: str


@app.post("/web/navigate")
def web_navigate(body: WebNavigateBody):
    t0 = time.perf_counter()
    try:
        from modules.humanoid.web import open_url
        result = open_url(body.url or "")
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(result.get("ok", False), result, ms, result.get("error"))
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


@app.post("/web/extract")
def web_extract():
    t0 = time.perf_counter()
    try:
        from modules.humanoid.web import extract_text
        result = extract_text()
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(result.get("ok", False), result, ms, result.get("error"))
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


@app.post("/web/screenshot")
def web_screenshot():
    t0 = time.perf_counter()
    try:
        from modules.humanoid.web import screenshot
        result = screenshot()
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(result.get("ok", False), result, ms, result.get("error"))
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


@app.get("/web/status")
def web_status_endpoint():
    t0 = time.perf_counter()
    try:
        from modules.humanoid.web import status as web_status
        data = web_status()
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(True, data, ms, None)
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


class VoiceSpeakBody(BaseModel):
    text: str


@app.post("/voice/speak")
def voice_speak_endpoint(body: VoiceSpeakBody):
    t0 = time.perf_counter()
    try:
        from modules.humanoid.voice import voice_speak
        result = voice_speak(body.text or "")
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(result.get("ok", False), result, ms, result.get("error"))
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


@app.post("/voice/listen")
def voice_listen_endpoint():
    t0 = time.perf_counter()
    try:
        from modules.humanoid.voice import voice_listen_stub
        result = voice_listen_stub()
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(result.get("ok", True), result, ms, result.get("error"))
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


@app.get("/voice/status")
def voice_status_endpoint():
    t0 = time.perf_counter()
    try:
        from modules.humanoid.voice import voice_status
        data = voice_status()
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(True, data, ms, None)
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


class VoiceTranscribeBody(BaseModel):
    audio_base64: Optional[str] = None
    language: Optional[str] = "es-ES"


@app.post("/voice/transcribe")
def voice_transcribe_endpoint(body: VoiceTranscribeBody):
    """Transcribe audio (WAV en base64). Devuelve { ok, data: { text }, ms, error }."""
    t0 = time.perf_counter()
    try:
        import base64
        import tempfile
        from pathlib import Path
        from modules.humanoid.voice.stt import transcribe, is_available
        if not body.audio_base64:
            return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), "audio_base64 requerido")
        if not is_available():
            return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), "STT no disponible")
        raw = base64.b64decode(body.audio_base64)
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
            f.write(raw)
            path = f.name
        try:
            result = transcribe(path, options={"language": body.language or "es-ES"})
            ms = int((time.perf_counter() - t0) * 1000)
            return _std_resp(result.get("ok", False), {"text": result.get("text", "")}, ms, result.get("error"))
        finally:
            Path(path).unlink(missing_ok=True)
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


class FaceCheckBody(BaseModel):
    image_base64: Optional[str] = None


@app.post("/face/check")
def face_check_endpoint(body: FaceCheckBody):
    """Detección de rostro en imagen (PNG/JPEG en base64). Devuelve { ok, data: { faces_detected, message }, ms, error }."""
    t0 = time.perf_counter()
    try:
        import base64
        from modules.humanoid.face import face_check_image
        if not body.image_base64:
            return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), "image_base64 requerido")
        raw = base64.b64decode(body.image_base64)
        result = face_check_image(raw)
        ms = int((time.perf_counter() - t0) * 1000)
        data = {"faces_detected": result.get("faces_detected", 0), "message": result.get("message", ""), "count": result.get("count", 0)}
        return _std_resp(result.get("ok", False), data, ms, result.get("error"))
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


@app.get("/face/status")
def face_status_endpoint():
    """Estado del módulo de reconocimiento facial."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.face.detector import is_available, _check_deps
        data = {"available": is_available(), "missing_deps": _check_deps()}
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(True, data, ms, None)
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


@app.post("/deps/check")
def deps_check_endpoint():
    """Return missing_deps per module and suggested commands. No install."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.deps_checker import check_all
        data = check_all()
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(data.get("ok", True), data, ms, None)
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


@app.get("/deps/check")
def deps_check_get():
    """GET variant of deps check."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.deps_checker import check_all
        data = check_all()
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(data.get("ok", True), data, ms, None)
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


# --- Vision ---
class VisionAnalyzeBody(BaseModel):
    image_path: str
    use_ocr: Optional[bool] = True
    use_llm_vision: Optional[bool] = True


def _professional_resp(ok: bool, data: Any, ms: int, error: Optional[str], resumen: str = "", archivos: Optional[list] = None, siguientes_pasos: Optional[list] = None) -> dict:
    """Respuesta profesional: resumen, resultado técnico, evidencia, siguientes pasos."""
    out = _std_resp(ok, data, ms, error)
    if resumen:
        out["resumen"] = resumen
    if archivos:
        out["archivos_creados"] = archivos
    if siguientes_pasos:
        out["siguientes_pasos"] = siguientes_pasos
    return out


@app.post("/vision/analyze")
def vision_analyze(body: VisionAnalyzeBody):
    """Analyze image: OCR + LLM vision. Returns texto, estructura, insights, acciones_sugeridas + formato profesional."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.vision import analyze
        result = analyze(body.image_path, use_ocr=body.use_ocr if body.use_ocr is not None else True, use_llm_vision=body.use_llm_vision if body.use_llm_vision is not None else True)
        ms = int((time.perf_counter() - t0) * 1000)
        ok = result.get("ok", False)
        result.setdefault("suggested_actions", result.get("acciones_sugeridas") or [])
        resumen = f"Imagen analizada: {len(result.get('extracted_text', ''))} chars texto, {len(result.get('entities', []))} entidades" if ok else "Análisis fallido"
        siguientes = result.get("acciones_sugeridas") or result.get("suggested_actions") or []
        return _professional_resp(ok, result, ms, result.get("error"), resumen=resumen, siguientes_pasos=siguientes)
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


class VisionOcrBody(BaseModel):
    image_path: str


@app.post("/vision/ocr")
def vision_ocr(body: VisionOcrBody):
    t0 = time.perf_counter()
    try:
        from modules.humanoid.vision import ocr
        result = ocr(body.image_path or "")
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(result.get("ok", False), result, ms, result.get("error"))
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


class VisionScreenshotBody(BaseModel):
    screenshot_path: str


@app.post("/vision/screenshot")
def vision_screenshot(body: VisionScreenshotBody):
    t0 = time.perf_counter()
    try:
        from modules.humanoid.vision import screenshot_analyze
        result = screenshot_analyze(body.screenshot_path or "")
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(result.get("ok", False), result, ms, result.get("error"))
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


@app.get("/vision/status")
def vision_status_endpoint():
    t0 = time.perf_counter()
    try:
        from modules.humanoid.vision import vision_status
        data = vision_status()
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(True, data, ms, None)
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


# --- Screen (capture / analyze / locate / act, policy-gated) ---
@app.get("/screen/status")
def screen_status_endpoint():
    """GET /screen/status. Responds even when disabled (enabled=false, missing_deps)."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.screen import get_screen_status
        data = get_screen_status()
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(True, data, ms, None)
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


class ScreenCaptureBody(BaseModel):
    region: Optional[List[int]] = None  # [x, y, w, h]
    format: Optional[str] = "png"


@app.post("/screen/capture")
def screen_capture(body: Optional[ScreenCaptureBody] = None):
    """Capture screen or region. Returns {ok, data: {path?, b64?}, ms, error}."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.screen.capture import capture_screen, save_capture_to_file
        region = tuple(body.region) if body and body.region and len(body.region) == 4 else None
        png, err = capture_screen(region=region, format=body.format if body else "png")
        if err or not png:
            return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), err or "no capture")
        import base64
        import os
        out_dir = (os.getenv("ATLAS_REPO_PATH") or os.getenv("POLICY_ALLOWED_PATHS", "C:\\ATLAS_PUSH")).strip().split(",")[0].strip()
        logs_dir = os.path.join(out_dir, "logs", "screen")
        path = save_capture_to_file(png, logs_dir, "capture")
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(True, {"path": path, "b64": base64.b64encode(png).decode("ascii")[:200] + "..."}, ms, None)
    except Exception as e:
        return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), str(e))


class ScreenAnalyzeBody(BaseModel):
    image_path: Optional[str] = None
    image_b64: Optional[str] = None
    prompt: Optional[str] = None


@app.post("/screen/analyze")
def screen_analyze(body: Optional[ScreenAnalyzeBody] = None):
    """Layout + text + vision suggestions. Returns {ok, data: {layout, description, ...}, ms, error}."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.screen.layout import get_layout
        from modules.humanoid.screen.vision_llm import analyze_image
        layout = get_layout()
        desc_result = {"description": "", "suggestions": []}
        if body and (body.image_b64 or body.image_path):
            desc_result = analyze_image(image_base64=body.image_b64, image_path=body.image_path, prompt=body.prompt or "Describe UI and suggest actions.")
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(True, {"layout": layout, **desc_result}, ms, layout.get("error"))
    except Exception as e:
        return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), str(e))


class ScreenLocateBody(BaseModel):
    query: str
    region: Optional[List[int]] = None


@app.post("/screen/locate")
def screen_locate(body: ScreenLocateBody):
    """Find elements by text query. Returns {ok, data: {matches}, ms, error}."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.screen.locator import locate
        region = tuple(body.region) if body.region and len(body.region) == 4 else None
        result = locate(body.query, region=region)
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(result.get("ok", False), result, ms, result.get("error"))
    except Exception as e:
        return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), str(e))


class ScreenActBody(BaseModel):
    action: str  # click | type | hotkey | scroll
    payload: dict  # {x,y} | {text} | {keys:[]} | {clicks, x?, y?}
    destructive: Optional[bool] = None  # if True, may require approval


@app.post("/screen/act")
def screen_act(body: ScreenActBody):
    """Execute screen action (policy-gated). Evidence: screenshot_before/after, coords, ms. Destructive -> ApprovalItem."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.policy import ActorContext, get_policy_engine
        from modules.humanoid.screen.status import get_screen_status
        from modules.humanoid.screen.policy import check_rate_limit, is_destructive_action, record_screen_act
        from modules.humanoid.screen.capture import capture_screen, save_capture_to_file
        from modules.humanoid.screen.actions import execute_action
        import os
        if not get_screen_status().get("enabled", False):
            return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), "screen module disabled")
        try:
            from modules.humanoid.mode import is_screen_act_allowed
            if not is_screen_act_allowed():
                return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), "screen_act disabled (ATLAS_MODE=lite or deps missing)")
        except Exception:
            pass
        ctx = ActorContext(actor="api", role="owner")
        decision = get_policy_engine().can(ctx, "screen", "screen_act", None)
        if not decision.allow:
            return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), decision.reason)
        allowed, reason = check_rate_limit()
        if not allowed:
            return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), reason)
        before_png, _ = capture_screen()
        _base = (os.getenv("ATLAS_REPO_PATH") or os.getenv("POLICY_ALLOWED_PATHS", "C:\\ATLAS_PUSH")).strip().split(",")[0].strip()
        _screen_logs = os.path.join(_base, "logs", "screen")
        path_before = save_capture_to_file(before_png, _screen_logs, "before") if before_png else None
        result = execute_action(body.action, body.payload or {})
        try:
            from modules.humanoid.screen.record import is_recording, record_action
            if is_recording():
                record_action(body.action, body.payload or {})
        except Exception:
            pass
        after_png, _ = capture_screen()
        path_after = save_capture_to_file(after_png, _screen_logs, "after") if after_png else None
        record_screen_act()
        ms = int((time.perf_counter() - t0) * 1000)
        evidence = {"screenshot_before": path_before, "screenshot_after": path_after, "coords": body.payload, "ms": ms}
        destructive = body.destructive if body.destructive is not None else is_destructive_action(body.action, body.payload or {})
        if destructive:
            evidence["approval_required"] = True
        return _std_resp(result.get("ok", False), {"evidence": evidence, "result": result}, ms, result.get("error"))
    except Exception as e:
        return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), str(e))


@app.post("/screen/record/start", tags=["Screen"])
def screen_record_start():
    """Start recording screen actions (macro). Pro/Ultra only."""
    try:
        from modules.humanoid.mode import is_record_replay_allowed
        if not is_record_replay_allowed():
            return _std_resp(False, None, 0, "record/replay disabled (ATLAS_MODE)")
        from modules.humanoid.screen.record import start_recording
        r = start_recording()
        return _std_resp(r.get("ok", False), r, 0, r.get("error"))
    except Exception as e:
        return _std_resp(False, None, 0, str(e))


@app.post("/screen/record/stop", tags=["Screen"])
def screen_record_stop():
    """Stop recording and return recorded actions."""
    try:
        from modules.humanoid.screen.record import stop_recording
        r = stop_recording()
        return _std_resp(r.get("ok", False), r, 0, r.get("error"))
    except Exception as e:
        return _std_resp(False, None, 0, str(e))


class ScreenReplayBody(BaseModel):
    macro_id: Optional[str] = None
    actions: Optional[List[dict]] = None
    delay_ms: Optional[int] = 100


@app.post("/screen/replay", tags=["Screen"])
def screen_replay(body: ScreenReplayBody):
    """Replay macro by id or inline actions. Policy-gated."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.policy import ActorContext, get_policy_engine
        from modules.humanoid.mode import is_record_replay_allowed
        if not is_record_replay_allowed():
            return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), "record/replay disabled")
        ctx = ActorContext(actor="api", role="owner")
        decision = get_policy_engine().can(ctx, "screen", "screen_act", None)
        if not decision.allow:
            return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), decision.reason)
        actions = body.actions
        if not actions and body.macro_id:
            from modules.humanoid.macros import get_macro
            m = get_macro(body.macro_id)
            if not m:
                return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), "macro not found")
            actions = m.get("actions", [])
        if not actions:
            return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), "no actions")
        from modules.humanoid.screen.replay import replay_actions
        r = replay_actions(actions, delay_ms=body.delay_ms or 100)
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(r.get("ok", False), r, ms, r.get("errors", [None])[0] if r.get("errors") else None)
    except Exception as e:
        return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), str(e))


# --- Macros ---
@app.get("/macros/list", tags=["Macros"])
def macros_list(limit: int = 50):
    """List saved macros."""
    try:
        from modules.humanoid.macros import list_macros
        return {"ok": True, "data": list_macros(limit=limit)}
    except Exception as e:
        return {"ok": False, "data": [], "error": str(e)}


class MacrosSaveBody(BaseModel):
    id: str
    name: str
    actions: List[dict]


@app.post("/macros/save", tags=["Macros"])
def macros_save(body: MacrosSaveBody):
    """Save macro (e.g. from record/stop actions)."""
    try:
        from modules.humanoid.macros import save_macro
        r = save_macro(body.id, body.name, body.actions)
        return _std_resp(r.get("ok", False), r, 0, r.get("error"))
    except Exception as e:
        return _std_resp(False, None, 0, str(e))


@app.get("/macros/get", tags=["Macros"])
def macros_get(id: Optional[str] = None):
    """Get macro by id."""
    if not id:
        return {"ok": False, "data": None, "error": "id required"}
    try:
        from modules.humanoid.macros import get_macro
        m = get_macro(id)
        return {"ok": m is not None, "data": m, "error": None if m else "not found"}
    except Exception as e:
        return {"ok": False, "data": None, "error": str(e)}


class MacrosDeleteBody(BaseModel):
    id: str


@app.post("/macros/delete", tags=["Macros"])
def macros_delete(body: MacrosDeleteBody):
    """Delete macro. Denied by default; approval required."""
    try:
        from modules.humanoid.policy import ActorContext, get_policy_engine
        ctx = ActorContext(actor="api", role="owner")
        decision = get_policy_engine().can(ctx, "macros", "delete", body.id)
        if not decision.allow:
            return _std_resp(False, None, 0, decision.reason)
        from modules.humanoid.macros import delete_macro
        r = delete_macro(body.id)
        return _std_resp(r.get("ok", False), r, 0, r.get("error"))
    except Exception as e:
        return _std_resp(False, None, 0, str(e))


# --- Bench (model benchmarking, Ultra/Pro+metalearn) ---
class BenchRunBody(BaseModel):
    level: Optional[str] = "quick"  # quick | full


@app.post("/bench/run", tags=["Bench"])
def bench_run(body: Optional[BenchRunBody] = None):
    """Run benchmark: probes per route, latency, recommended model mapping. Ultra or Pro+metalearn."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.mode import is_benchmark_allowed
        if not is_benchmark_allowed():
            return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), "benchmark disabled (ATLAS_MODE or METALEARN)")
        from modules.humanoid.bench import run_bench
        level = (body and body.level) or "quick"
        r = run_bench(level=level)
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(True, r, ms, None)
    except Exception as e:
        return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), str(e))


# --- Memory (Policy: memory_read / memory_write / memory_export; memory_delete denied) ---
def _memory_actor():
    return ActorContext(actor="api", role=os.getenv("POLICY_DEFAULT_ROLE", "owner"))

@app.get("/memory/recall")
def memory_recall(query: str = "", limit: int = 20):
    """Recall by search query. FTS or LIKE fallback. Policy: memory_read."""
    t0 = time.perf_counter()
    try:
        decision = get_policy_engine().can(_memory_actor(), "memory", "memory_read")
        if not decision.allow:
            return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), decision.reason)
        from modules.humanoid.memory_engine import recall_by_query
        data = recall_by_query(query or "", limit=limit)
        ms = int((time.perf_counter() - t0) * 1000)
        return _professional_resp(True, {"results": data}, ms, None, resumen=f"Recall: {len(data)} resultados")
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


@app.get("/memory/thread")
def memory_thread(thread_id: str = "", limit: int = 50):
    """Recall by thread_id. Policy: memory_read."""
    t0 = time.perf_counter()
    try:
        decision = get_policy_engine().can(_memory_actor(), "memory", "memory_read")
        if not decision.allow:
            return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), decision.reason)
        from modules.humanoid.memory_engine import recall_by_thread
        data = recall_by_thread(thread_id or "", limit=limit)
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(True, data, ms, None)
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


class MemoryThreadCreateBody(BaseModel):
    title: str = ""

@app.post("/memory/thread/create")
def memory_thread_create(body: MemoryThreadCreateBody):
    """Create or get thread. Returns thread_id. Policy: memory_write."""
    t0 = time.perf_counter()
    try:
        decision = get_policy_engine().can(_memory_actor(), "memory", "memory_write")
        if not decision.allow:
            return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), decision.reason)
        from modules.humanoid.memory_engine import ensure_thread
        thread_id = ensure_thread(None, (body.title or "")[:200])
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(True, {"thread_id": thread_id}, ms, None)
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


@app.get("/memory/thread/list")
def memory_thread_list(limit: int = 50):
    """List threads. Policy: memory_read."""
    t0 = time.perf_counter()
    try:
        decision = get_policy_engine().can(_memory_actor(), "memory", "memory_read")
        if not decision.allow:
            return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), decision.reason)
        from modules.humanoid.memory_engine import list_threads
        data = list_threads(limit=limit)
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(True, {"threads": data}, ms, None)
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


class MemoryWriteBody(BaseModel):
    thread_id: str = ""
    kind: str = "summary"  # artifact | decision | summary
    payload: dict = {}
    task_id: Optional[str] = None
    run_id: Optional[int] = None

@app.post("/memory/write")
def memory_write_endpoint(body: MemoryWriteBody):
    """Write artifact/decision/summary. Policy: memory_write. Redacts secrets."""
    t0 = time.perf_counter()
    try:
        decision = get_policy_engine().can(_memory_actor(), "memory", "memory_write")
        if not decision.allow:
            return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), decision.reason)
        from modules.humanoid.memory_engine import memory_write as mem_write
        out = mem_write(body.thread_id or None, body.kind, body.payload or {}, body.task_id, body.run_id)
        ms = int((time.perf_counter() - t0) * 1000)
        if not out.get("ok"):
            return _std_resp(False, None, ms, out.get("error"))
        return _std_resp(True, {"id": out.get("id"), "kind": out.get("kind")}, ms, None)
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


class MemorySummarizeBody(BaseModel):
    thread_id: str
    use_llm: bool = True

@app.post("/memory/summarize")
def memory_summarize(body: MemorySummarizeBody):
    """Incremental summary for thread. FAST LLM if use_llm else deterministic. Policy: memory_write. Timeout 15s."""
    t0 = time.perf_counter()
    timeout_sec = 15
    try:
        decision = get_policy_engine().can(_memory_actor(), "memory", "memory_write")
        if not decision.allow:
            return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), decision.reason)
        from modules.humanoid.memory_engine import recall_by_thread, add_summary
        data = recall_by_thread(body.thread_id, limit=20)
        tasks = data.get("tasks", [])
        summaries = data.get("summaries", [])
        if body.use_llm:
            try:
                from modules.humanoid import get_humanoid_kernel
                k = get_humanoid_kernel()
                brain = k.registry.get("brain")
                if brain and hasattr(brain, "run_llm"):
                    goals = [t.get("goal", "")[:100] for t in tasks[:5]]
                    prompt = f"Resumen en una frase: hilo con {len(tasks)} tareas. Primeras: {'; '.join(goals)}."
                    r = brain.run_llm(prompt, route="FAST", max_tokens=128, timeout_override=timeout_sec)
                    text = (r.get("output") or "").strip()[:2000] if r.get("ok") else ""
                    if text:
                        add_summary(body.thread_id, text)
                        ms = int((time.perf_counter() - t0) * 1000)
                        return _std_resp(True, {"summary_preview": text[:200]}, ms, None)
            except Exception:
                pass
        text = f"Resumen: {len(tasks)} tareas, {len(summaries)} resúmenes previos."
        add_summary(body.thread_id, text)
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(True, {"summary_preview": text}, ms, None)
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


@app.get("/memory/export")
def memory_export(thread_id: str = "", task_id: str = "", limit: int = 100):
    """Export memory as markdown. Policy: memory_export."""
    t0 = time.perf_counter()
    try:
        decision = get_policy_engine().can(_memory_actor(), "memory", "memory_export")
        if not decision.allow:
            return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), decision.reason)
        from modules.humanoid.memory_engine import export_markdown
        md = export_markdown(thread_id=thread_id, task_id=task_id, limit=limit)
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(True, {"markdown": md}, ms, None)
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


@app.get("/memory/snapshot")
def memory_snapshot():
    """Snapshot counts for backup. Policy: memory_read."""
    t0 = time.perf_counter()
    try:
        decision = get_policy_engine().can(_memory_actor(), "memory", "memory_read")
        if not decision.allow:
            return _std_resp(False, None, int((time.perf_counter() - t0) * 1000), decision.reason)
        from modules.humanoid.memory_engine import snapshot
        data = snapshot()
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(data.get("ok", True), data, ms, None)
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))



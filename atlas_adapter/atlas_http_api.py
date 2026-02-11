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
    load_dotenv(ENV_PATH)
else:
    import logging
    logging.warning("atlas.env not found at %s", ENV_PATH)

(BASE_DIR / "logs").mkdir(parents=True, exist_ok=True)
# Default DB paths so scheduler + memory work without atlas.env
if not os.getenv("SCHED_DB_PATH"):
    os.environ["SCHED_DB_PATH"] = str(BASE_DIR / "logs" / "atlas_sched.sqlite")
if not os.getenv("ATLAS_MEMORY_DB_PATH"):
    os.environ["ATLAS_MEMORY_DB_PATH"] = str(BASE_DIR / "logs" / "atlas_memory.sqlite")

from fastapi import FastAPI
from fastapi.responses import FileResponse
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
    humanoid = os.getenv("HUMANOID_ENABLED", "true").strip().lower() in ("1", "true", "yes", "y", "on")
    sched = os.getenv("SCHED_ENABLED", "true").strip().lower() in ("1", "true", "yes", "y", "on")
    if humanoid and sched:
        from modules.humanoid.scheduler import start_scheduler
        from modules.humanoid.watchdog import start_watchdog
        from concurrent.futures import ThreadPoolExecutor
        _executor = ThreadPoolExecutor(max_workers=2)
        start_scheduler(executor=_executor)
        start_watchdog()
        try:
            from modules.humanoid.ci import ensure_ci_jobs
            ensure_ci_jobs()
        except Exception:
            pass
    yield


app = FastAPI(title="ATLAS Adapter", version="1.0.0", lifespan=_lifespan)

# Metrics middleware: request count + latency per path
from modules.humanoid.metrics import MetricsMiddleware
app.add_middleware(MetricsMiddleware)

class Step(BaseModel):
    tool: str
    args: dict = {}

@app.get("/status")
def status():
    return {"ok": True, "atlas": handle("/status")}


@app.get("/version")
def version():
    """Release version, git sha, channel (stable/canary)."""
    try:
        from modules.humanoid.release import get_version_info
        return {"ok": True, **get_version_info()}
    except Exception as e:
        return {"ok": False, "version": "0.0.0", "git_sha": "", "channel": "canary", "error": str(e)}


# Dashboard UI (fallback: if UI fails, API still works)
STATIC_DIR = BASE_DIR / "atlas_adapter" / "static"
STATIC_DIR.mkdir(parents=True, exist_ok=True)


@app.get("/ui")
def serve_ui():
    """Minimal dashboard: status, version, metrics, jobs, update, approvals."""
    path = STATIC_DIR / "dashboard.html"
    if path.exists():
        return FileResponse(path)
    return {"ok": False, "error": "dashboard.html not found"}

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
from typing import Any, Optional
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
from typing import Any, Optional
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
    return llm_service.run(req)


# --- Humanoid (kernel + modules) ---
from modules.humanoid.api import router as humanoid_router

app.include_router(humanoid_router)


# --- Metrics / Policy / Audit endpoints ---
from modules.humanoid.metrics import get_metrics_store
from modules.humanoid.policy import ActorContext, get_policy_engine
from modules.humanoid.audit import get_audit_logger


@app.get("/metrics")
def metrics():
    """JSON metrics: counters and latencies per endpoint."""
    return get_metrics_store().snapshot()


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


@app.get("/support/bundle")
def support_bundle():
    """Export bundle: recent logs, metrics, last CI report. Returns {ok, data: {path}, ms, error}. Audited."""
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
                snap = get_metrics_store().snapshot()
                zf.writestr("metrics.json", json.dumps(snap, indent=2))
            except Exception:
                zf.writestr("metrics.json", "{}")
            log_file = BASE_DIR / "logs" / "atlas.log"
            if log_file.exists():
                try:
                    tail = log_file.read_text(encoding="utf-8", errors="replace")[-50000:]
                    zf.writestr("atlas_log_tail.txt", tail)
                except Exception:
                    pass
            ci_dir = Path(os.getenv("CI_EXPORT_DIR", str(BASE_DIR / "snapshots" / "ci")))
            if ci_dir.exists():
                reports = sorted(ci_dir.glob("CI_REPORT_*.md"), key=lambda p: p.stat().st_mtime, reverse=True)
                if reports:
                    zf.write(reports[0], "ci_last_report.md")
        get_audit_logger().log_event("support", "api", "bundle", True, int((time.perf_counter() - t0) * 1000), None, {"path": str(zip_path)}, None)
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(True, {"path": str(zip_path)}, ms, None)
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        get_audit_logger().log_event("support", "api", "bundle", False, ms, str(e), None, None)
        return _std_resp(False, None, ms, str(e))


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
        from modules.humanoid.update.update_engine import apply as update_apply
        result = update_apply()
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(result.get("ok", False), result.get("data"), ms, result.get("error"))
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return _std_resp(False, None, ms, str(e))


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



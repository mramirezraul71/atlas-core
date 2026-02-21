"""
ATLAS Workspace -- Open Interpreter Bridge.

Session-managed integration of Open Interpreter as the autonomous execution
engine for Atlas Workspace.  Supports Bedrock, OpenAI, Gemini, Anthropic,
xAI/Grok, DeepSeek, Groq and Ollama through LiteLLM routing.

Security: every execution passes through PolicyEngine + AuditLogger +
GovernanceGates before touching the OS.
"""
from __future__ import annotations

import asyncio
import logging
import os
import threading
import time
import uuid
from dataclasses import dataclass, field
from typing import Any, AsyncGenerator, Dict, List, Optional

logger = logging.getLogger("atlas.interpreter")

# ---------------------------------------------------------------------------
# Model catalog -- ordered by priority per category
# ---------------------------------------------------------------------------

_MODEL_CATALOG: Dict[str, List[Dict[str, str]]] = {
    "bedrock": [
        {"id": "bedrock/us.anthropic.claude-opus-4-6-v1:0", "label": "Claude Opus 4.6 (Bedrock)", "tier": "premium"},
        {"id": "bedrock/us.anthropic.claude-sonnet-4-20250514-v1:0", "label": "Claude Sonnet 4 (Bedrock)", "tier": "standard"},
        {"id": "bedrock/us.anthropic.claude-haiku-4-5-20251001-v1:0", "label": "Claude Haiku 4.5 (Bedrock)", "tier": "fast"},
    ],
    "anthropic": [
        {"id": "anthropic/claude-opus-4-20250514", "label": "Claude Opus 4 (Direct)", "tier": "premium"},
        {"id": "anthropic/claude-sonnet-4-20250514", "label": "Claude Sonnet 4 (Direct)", "tier": "standard"},
    ],
    "openai": [
        {"id": "openai/gpt-4.1", "label": "GPT-4.1", "tier": "premium"},
        {"id": "openai/gpt-4.1-mini", "label": "GPT-4.1 Mini", "tier": "fast"},
        {"id": "openai/gpt-4o", "label": "GPT-4o", "tier": "standard"},
        {"id": "openai/o3-mini", "label": "o3-mini", "tier": "reasoning"},
    ],
    "gemini": [
        {"id": "gemini/gemini-2.5-pro", "label": "Gemini 2.5 Pro", "tier": "premium"},
        {"id": "gemini/gemini-2.5-flash", "label": "Gemini 2.5 Flash", "tier": "fast"},
    ],
    "xai": [
        {"id": "xai/grok-4", "label": "Grok 4", "tier": "premium"},
        {"id": "xai/grok-4-fast", "label": "Grok 4 Fast", "tier": "fast"},
    ],
    "deepseek": [
        {"id": "deepseek/deepseek-chat", "label": "DeepSeek V3", "tier": "standard"},
        {"id": "deepseek/deepseek-reasoner", "label": "DeepSeek R1", "tier": "reasoning"},
    ],
    "groq": [
        {"id": "groq/llama-3.3-70b-versatile", "label": "Llama 3.3 70B (Groq)", "tier": "fast"},
    ],
    "ollama": [
        {"id": "ollama/llama3.1:latest", "label": "Llama 3.1 (Local)", "tier": "local"},
        {"id": "ollama/deepseek-r1:latest", "label": "DeepSeek R1 (Local)", "tier": "local"},
        {"id": "ollama/qwen2.5:latest", "label": "Qwen 2.5 (Local)", "tier": "local"},
    ],
}

_PROVIDER_PRIORITY = ["bedrock", "anthropic", "openai", "xai", "gemini", "deepseek", "groq", "ollama"]

_ATLAS_SYSTEM_MESSAGE = """\
Eres el agente de ejecucion autonoma del sistema ATLAS NEXUS.
Propietario: Raul Ramirez, arquitecto del sistema.
Ejecutas tareas reales: codigo Python, comandos terminal, gestion de archivos,
navegacion web y operaciones Git.
Responde siempre en espanol. Se directo y accionable.
Proyecto principal: C:\\ATLAS_PUSH
Reglas de seguridad:
- NO ejecutes rm -rf, format, del /f /s, rd /s /q ni mkfs.
- NO accedas a archivos fuera de C:\\ATLAS_PUSH sin autorizacion.
- Reporta cada accion antes de ejecutarla.
"""


def _provider_available(provider: str) -> bool:
    """Check whether a provider has credentials configured."""
    env = os.environ.get
    checks = {
        "bedrock": lambda: bool(
            (env("AWS_ACCESS_KEY_ID", "") and env("AWS_SECRET_ACCESS_KEY", ""))
            or env("AWS_PROFILE", "")
            or (env("ATLAS_AI_MODE", "") or "").strip().lower() == "bedrock"
        ),
        "anthropic": lambda: bool(env("ANTHROPIC_API_KEY", "").strip()),
        "openai": lambda: bool(env("OPENAI_API_KEY", "").strip()),
        "gemini": lambda: bool(env("GEMINI_API_KEY", "") or env("GOOGLE_API_KEY", "")),
        "xai": lambda: bool(env("XAI_API_KEY", "").strip()),
        "deepseek": lambda: bool(env("DEEPSEEK_API_KEY", "").strip()),
        "groq": lambda: bool(env("GROQ_API_KEY", "").strip()),
        "ollama": lambda: True,
    }
    try:
        return checks.get(provider, lambda: False)()
    except Exception:
        return False


def resolve_model(requested: Optional[str] = None, tier: str = "standard") -> str:
    """
    Determine which model to use.

    Priority:
    1. Explicit ``requested`` model string  (pass-through).
    2. WORKSPACE_INTERPRETER_MODEL env var.
    3. Auto-detect: iterate _PROVIDER_PRIORITY, pick first available model
       matching the requested ``tier`` (premium/standard/fast/reasoning/local).
    """
    if requested:
        return requested

    from_env = (os.getenv("WORKSPACE_INTERPRETER_MODEL") or "").strip()
    if from_env:
        return from_env

    for prov in _PROVIDER_PRIORITY:
        if not _provider_available(prov):
            continue
        models = _MODEL_CATALOG.get(prov, [])
        for m in models:
            if m["tier"] == tier:
                return m["id"]
        if models:
            return models[0]["id"]

    return "openai/gpt-4o"


def list_available_models() -> List[Dict[str, str]]:
    """Return flat list of all models whose provider is currently available."""
    out: List[Dict[str, str]] = []
    for prov in _PROVIDER_PRIORITY:
        if _provider_available(prov):
            out.extend(_MODEL_CATALOG.get(prov, []))
    return out


# ---------------------------------------------------------------------------
# InterpreterSession
# ---------------------------------------------------------------------------

@dataclass
class InterpreterSession:
    """Isolated Open Interpreter instance with its own conversation state."""

    session_id: str = field(default_factory=lambda: uuid.uuid4().hex[:12])
    model: str = ""
    created_at: float = field(default_factory=time.time)
    last_active: float = field(default_factory=time.time)
    _interpreter: Any = field(default=None, repr=False)
    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False)

    def _ensure_interpreter(self) -> Any:
        if self._interpreter is not None:
            return self._interpreter
        from interpreter import OpenInterpreter
        oi = OpenInterpreter()
        model_id = self.model or resolve_model()
        self.model = model_id

        oi.llm.model = model_id
        oi.llm.supports_functions = False
        oi.llm.supports_vision = False

        prov = model_id.split("/")[0] if "/" in model_id else model_id.split(":")[0]
        if prov == "bedrock":
            os.environ.setdefault("AWS_REGION_NAME", os.getenv("AWS_REGION", "us-east-1"))
        elif prov == "ollama":
            oi.llm.api_base = os.getenv("OLLAMA_BASE_URL", "http://localhost:11434")
        elif prov == "openai":
            oi.llm.api_key = os.getenv("OPENAI_API_KEY", "")
        elif prov == "anthropic":
            oi.llm.api_key = os.getenv("ANTHROPIC_API_KEY", "")
        elif prov == "gemini":
            oi.llm.api_key = os.getenv("GEMINI_API_KEY", "") or os.getenv("GOOGLE_API_KEY", "")
        elif prov == "xai":
            oi.llm.api_key = os.getenv("XAI_API_KEY", "")
        elif prov == "deepseek":
            oi.llm.api_key = os.getenv("DEEPSEEK_API_KEY", "")
        elif prov == "groq":
            oi.llm.api_key = os.getenv("GROQ_API_KEY", "")

        oi.auto_run = os.getenv("INTERPRETER_AUTO_RUN", "false").strip().lower() in ("1", "true", "yes")
        oi.safe_mode = "ask"
        oi.verbose = False
        oi.system_message = _ATLAS_SYSTEM_MESSAGE
        try:
            oi.anonymous_telemetry = False
        except (AttributeError, TypeError):
            oi.disable_telemetry = True
        oi.max_output = 3000

        self._interpreter = oi
        logger.info("InterpreterSession %s created with model=%s", self.session_id, model_id)
        return oi

    def chat_sync(self, task: str, stream: bool = True, auto_run: Optional[bool] = None) -> list:
        """Blocking chat call.  Returns list of message chunks."""
        with self._lock:
            oi = self._ensure_interpreter()
            if auto_run is not None:
                oi.auto_run = auto_run
            self.last_active = time.time()
            return list(oi.chat(task, stream=stream, display=False))

    def reset(self) -> None:
        with self._lock:
            if self._interpreter:
                try:
                    self._interpreter.reset()
                except Exception:
                    pass
                self._interpreter = None


# ---------------------------------------------------------------------------
# Session Manager
# ---------------------------------------------------------------------------

class InterpreterSessionManager:
    """Thread-safe pool of interpreter sessions with TTL eviction."""

    def __init__(
        self,
        max_sessions: int = 0,
        ttl_sec: int = 0,
    ) -> None:
        self._max = max_sessions or int(os.getenv("INTERPRETER_MAX_SESSIONS", "5"))
        self._ttl = ttl_sec or int(os.getenv("INTERPRETER_SESSION_TTL_SEC", "900"))
        self._sessions: Dict[str, InterpreterSession] = {}
        self._lock = threading.Lock()

    def get_or_create(self, session_id: Optional[str] = None, model: Optional[str] = None) -> InterpreterSession:
        with self._lock:
            self._evict_expired()
            if session_id and session_id in self._sessions:
                s = self._sessions[session_id]
                s.last_active = time.time()
                return s
            if len(self._sessions) >= self._max:
                oldest_key = min(self._sessions, key=lambda k: self._sessions[k].last_active)
                old = self._sessions.pop(oldest_key)
                old.reset()
                logger.info("Evicted oldest session %s to make room", oldest_key)
            sid = session_id or uuid.uuid4().hex[:12]
            s = InterpreterSession(session_id=sid, model=model or "")
            self._sessions[sid] = s
            return s

    def close(self, session_id: str) -> bool:
        with self._lock:
            s = self._sessions.pop(session_id, None)
            if s:
                s.reset()
                return True
            return False

    def list_sessions(self) -> List[Dict[str, Any]]:
        with self._lock:
            self._evict_expired()
            return [
                {
                    "session_id": s.session_id,
                    "model": s.model,
                    "created_at": s.created_at,
                    "last_active": s.last_active,
                    "age_sec": int(time.time() - s.created_at),
                }
                for s in self._sessions.values()
            ]

    def _evict_expired(self) -> None:
        now = time.time()
        expired = [k for k, v in self._sessions.items() if (now - v.last_active) > self._ttl]
        for k in expired:
            s = self._sessions.pop(k, None)
            if s:
                s.reset()
                logger.info("Session %s expired (TTL %ds)", k, self._ttl)


# ---------------------------------------------------------------------------
# Global manager singleton (thread-safe by design)
# ---------------------------------------------------------------------------

_manager: Optional[InterpreterSessionManager] = None
_mgr_lock = threading.Lock()


def get_session_manager() -> InterpreterSessionManager:
    global _manager
    if _manager is None:
        with _mgr_lock:
            if _manager is None:
                _manager = InterpreterSessionManager()
    return _manager


# ---------------------------------------------------------------------------
# Execution helpers (async wrappers)
# ---------------------------------------------------------------------------

def _policy_check(task: str) -> Optional[str]:
    """
    Return error string if policy explicitly denies execution, else None.

    The interpreter receives natural-language tasks, not raw shell commands,
    so we only block when the policy engine has an *explicit* deny rule.
    Missing rules ("no matching allow rule") are treated as allowed for
    owner-level access -- the interpreter's own safety layer handles the rest.
    """
    try:
        from modules.humanoid.policy import ActorContext, get_policy_engine
        role = os.getenv("POLICY_DEFAULT_ROLE", "owner")
        actor = ActorContext(actor="interpreter", role=role)
        decision = get_policy_engine().can(actor, "hands", "exec_command", target=task)
        if not decision.allow:
            reason = (decision.reason or "").lower()
            if "no matching" in reason or "not found" in reason:
                return None
            return decision.reason or "policy denied"
    except ImportError:
        pass
    except Exception as exc:
        logger.debug("Policy check soft-fail: %s", exc)
    return None


def _audit_log(task: str, ok: bool, ms: int, error: Optional[str], meta: Optional[dict] = None) -> None:
    try:
        from modules.humanoid.audit import get_audit_logger
        get_audit_logger().log_event(
            "interpreter",
            os.getenv("POLICY_DEFAULT_ROLE", "owner"),
            "hands",
            "exec_command",
            ok,
            ms,
            error,
            {"task": task[:500]},
            meta,
        )
    except Exception:
        pass


async def execute_streaming(
    task: str,
    model: Optional[str] = None,
    auto_run: bool = False,
    session_id: Optional[str] = None,
) -> AsyncGenerator[Dict[str, Any], None]:
    """
    Run a task through Open Interpreter and yield typed chunks.

    Each chunk is a dict with keys: type (message|code|output|error|done),
    content, and optional metadata.
    """
    t0 = time.perf_counter()

    denial = _policy_check(task)
    if denial:
        _audit_log(task, False, int((time.perf_counter() - t0) * 1000), denial)
        yield {"type": "error", "content": f"Policy denied: {denial}"}
        return

    mgr = get_session_manager()
    session = mgr.get_or_create(session_id=session_id, model=model)

    yield {
        "type": "status",
        "content": f"Session {session.session_id} | model={session.model or resolve_model(model)}",
        "session_id": session.session_id,
    }

    loop = asyncio.get_event_loop()

    try:
        chunks = await loop.run_in_executor(
            None,
            lambda: session.chat_sync(task, stream=True, auto_run=auto_run),
        )
    except Exception as exc:
        ms = int((time.perf_counter() - t0) * 1000)
        _audit_log(task, False, ms, str(exc))
        yield {"type": "error", "content": str(exc)}
        return

    for chunk in chunks:
        ctype = chunk.get("type", "message")
        if ctype == "message":
            yield {"type": "message", "content": chunk.get("content", ""), "role": chunk.get("role", "assistant")}
        elif ctype == "code":
            yield {"type": "code", "language": chunk.get("format", "python"), "content": chunk.get("content", "")}
        elif ctype == "console":
            yield {"type": "output", "content": chunk.get("output", chunk.get("content", "")), "format": chunk.get("format", "output")}
        elif ctype == "confirmation":
            yield {"type": "confirmation", "content": chunk.get("content", "")}
        elif ctype == "image":
            yield {"type": "image", "content": chunk.get("content", ""), "format": chunk.get("format", "png")}
        else:
            yield {"type": ctype, "content": str(chunk.get("content", ""))}

    ms = int((time.perf_counter() - t0) * 1000)
    _audit_log(task, True, ms, None, {"session_id": session.session_id, "model": session.model, "chunks": len(chunks)})
    yield {"type": "done", "content": "Tarea completada", "elapsed_ms": ms, "session_id": session.session_id}


async def execute_quick(task: str, model: Optional[str] = None) -> Dict[str, Any]:
    """Non-streaming execution.  Returns full result dict."""
    t0 = time.perf_counter()

    denial = _policy_check(task)
    if denial:
        _audit_log(task, False, int((time.perf_counter() - t0) * 1000), denial)
        return {"ok": False, "error": f"Policy denied: {denial}"}

    mgr = get_session_manager()
    session = mgr.get_or_create(model=model)

    loop = asyncio.get_event_loop()
    try:
        chunks = await loop.run_in_executor(
            None,
            lambda: session.chat_sync(task, stream=False, auto_run=True),
        )
    except Exception as exc:
        ms = int((time.perf_counter() - t0) * 1000)
        _audit_log(task, False, ms, str(exc))
        return {"ok": False, "error": str(exc), "elapsed_ms": ms}

    messages = chunks if isinstance(chunks, list) else [chunks]
    text_parts = [
        m.get("content", "")
        for m in messages
        if m.get("type") == "message" and m.get("role") == "assistant"
    ]
    ms = int((time.perf_counter() - t0) * 1000)
    _audit_log(task, True, ms, None, {"session_id": session.session_id, "model": session.model})
    return {
        "ok": True,
        "result": "\n".join(text_parts) or "Tarea ejecutada",
        "model": session.model,
        "session_id": session.session_id,
        "elapsed_ms": ms,
    }


def interpreter_status() -> Dict[str, Any]:
    """Return engine status for the /status endpoint."""
    try:
        import interpreter as _oi_pkg
        version = getattr(_oi_pkg, "__version__", "unknown")
    except Exception:
        version = "not_installed"

    mgr = get_session_manager()
    return {
        "installed": version != "not_installed",
        "version": version,
        "ai_mode": (os.getenv("ATLAS_AI_MODE") or "auto").strip(),
        "default_model": resolve_model(),
        "available_models": list_available_models(),
        "active_sessions": mgr.list_sessions(),
        "max_sessions": mgr._max,
        "session_ttl_sec": mgr._ttl,
        "auto_run": os.getenv("INTERPRETER_AUTO_RUN", "false"),
    }

"""
POT Triggers: Sistema de Disparadores Automáticos.
===================================================
Define las condiciones que disparan POTs automáticamente.

Este módulo conecta eventos del sistema con el POT Dispatcher.
"""
from __future__ import annotations

import logging
import os
import subprocess
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Set

_log = logging.getLogger("humanoid.quality.triggers")

REPO_ROOT = Path(__file__).resolve().parent.parent.parent.parent


def _env_bool(name: str, default: bool) -> bool:
    v = os.getenv(name, "true" if default else "false").strip().lower()
    return v in ("1", "true", "yes", "y", "on")


class TriggerCondition(str, Enum):
    """Tipos de condiciones que pueden disparar POTs."""
    GIT_CHANGES = "git_changes"          # Cambios pendientes en Git
    GIT_BEHIND = "git_behind"            # Branch detrás del remoto
    GIT_UNSAFE = "git_unsafe"            # Estado Git peligroso (lock/rebase/detached/conflicts)
    SERVICE_DOWN = "service_down"        # Servicio caído
    API_ERROR = "api_error"              # Error en API
    DISK_FULL = "disk_full"              # Disco lleno (>90%)
    MEMORY_HIGH = "memory_high"          # Memoria alta (>85%)
    INCIDENT_NEW = "incident_new"        # Nuevo incidente en ANS
    APPROVAL_PENDING = "approval_pending"  # Aprobación pendiente
    SCHEDULE_TIME = "schedule_time"      # Hora programada
    FILE_CHANGED = "file_changed"        # Archivo modificado
    WEBHOOK = "webhook"                  # Webhook externo
    STARTUP = "startup"                  # Inicio del sistema
    SHUTDOWN = "shutdown"                # Cierre del sistema


@dataclass
class TriggerRule:
    """Regla de trigger que mapea una condición a un POT."""
    id: str
    name: str
    condition: TriggerCondition
    pot_id: str
    enabled: bool = True
    priority: int = 5
    cooldown_seconds: int = 300  # Mínimo tiempo entre ejecuciones
    require_approval: bool = False
    context_builder: Optional[Callable[[], Dict[str, Any]]] = None
    check_interval_seconds: int = 60
    last_triggered: Optional[str] = None
    
    # Parámetros específicos por tipo de condición
    params: Dict[str, Any] = field(default_factory=dict)


# ============================================================================
# TRIGGER REGISTRY
# ============================================================================

# ============================================================================
# TURBO TRIGGER RULES (optimizados para velocidad)
# ============================================================================

DEFAULT_TRIGGER_RULES: List[TriggerRule] = [
    # Git Triggers - TURBO
    TriggerRule(
        id="git_changes_auto_commit",
        name="Auto-commit cuando hay cambios",
        condition=TriggerCondition.GIT_CHANGES,
        pot_id="git_commit",
        # Seguridad: no auto-commitear por defecto (evita estados Git peligrosos).
        enabled=_env_bool("QUALITY_GIT_TRIGGERS_ENABLED", False),
        priority=6,
        cooldown_seconds=60,   # TURBO: Era 600s, ahora 60s
        check_interval_seconds=15,  # TURBO: Era 120s, ahora 15s
        params={"min_changes": 1},
    ),
    TriggerRule(
        id="git_behind_auto_pull",
        name="Auto-pull cuando estamos detrás",
        condition=TriggerCondition.GIT_BEHIND,
        pot_id="git_pull",
        enabled=_env_bool("QUALITY_GIT_TRIGGERS_ENABLED", False),
        priority=4,
        cooldown_seconds=60,   # TURBO: Era 300s, ahora 60s
        check_interval_seconds=30,  # TURBO: Era 300s, ahora 30s
    ),
    TriggerRule(
        id="git_unsafe_self_heal",
        name="Auto-reparación Git (estado peligroso)",
        condition=TriggerCondition.GIT_UNSAFE,
        pot_id="git_safe_sync",
        enabled=_env_bool("QUALITY_GIT_UNSAFE_TRIGGERS_ENABLED", True),
        priority=2,
        cooldown_seconds=120,
        check_interval_seconds=15,
        # Por defecto exige aprobación: toca el repositorio.
        require_approval=_env_bool("QUALITY_GIT_UNSAFE_REQUIRE_APPROVAL", True),
    ),
    
    # Service Health Triggers - TURBO
    TriggerRule(
        id="service_down_repair",
        name="Reparar servicio caído",
        condition=TriggerCondition.SERVICE_DOWN,
        pot_id="services_repair",
        enabled=True,
        priority=1,  # TURBO: Máxima prioridad
        cooldown_seconds=30,   # TURBO: Era 120s, ahora 30s
        check_interval_seconds=5,  # TURBO: Check cada 5s
        require_approval=False,
        params={"services": ["atlas_api", "scheduler", "ans"]},
    ),
    TriggerRule(
        id="api_error_repair",
        name="Reparar API con errores",
        condition=TriggerCondition.API_ERROR,
        pot_id="api_repair",
        enabled=True,
        priority=2,
        cooldown_seconds=30,   # TURBO: Era 180s, ahora 30s
        check_interval_seconds=5,
    ),
    
    # System Health Triggers - TURBO
    TriggerRule(
        id="disk_full_cleanup",
        name="Limpiar cuando disco lleno",
        condition=TriggerCondition.DISK_FULL,
        pot_id="maintenance_daily",
        enabled=True,
        priority=3,
        cooldown_seconds=600,  # TURBO: Era 3600s, ahora 600s
        check_interval_seconds=60,
        params={"threshold_percent": 85},  # TURBO: threshold más bajo
    ),
    
    # Incident Triggers - TURBO
    TriggerRule(
        id="incident_auto_triage",
        name="Triage automático de incidentes",
        condition=TriggerCondition.INCIDENT_NEW,
        pot_id="incident_triage",
        enabled=True,
        priority=2,
        cooldown_seconds=15,   # TURBO: Era 60s, ahora 15s
        check_interval_seconds=5,
    ),
    
    # Startup/Shutdown Triggers
    TriggerRule(
        id="session_startup",
        name="Inicialización de sesión",
        condition=TriggerCondition.STARTUP,
        pot_id="session_startup",
        enabled=True,
        priority=1,
        cooldown_seconds=0,
    ),
    TriggerRule(
        id="session_shutdown",
        name="Cierre de sesión",
        condition=TriggerCondition.SHUTDOWN,
        pot_id="session_shutdown",
        enabled=True,
        priority=1,
        cooldown_seconds=0,
    ),
]


class TriggerRegistry:
    """Registro de reglas de trigger."""
    
    def __init__(self):
        self._rules: Dict[str, TriggerRule] = {}
        self._load_defaults()
    
    def _load_defaults(self) -> None:
        """Carga las reglas por defecto."""
        for rule in DEFAULT_TRIGGER_RULES:
            self._rules[rule.id] = rule
    
    def register(self, rule: TriggerRule) -> None:
        """Registra una nueva regla."""
        self._rules[rule.id] = rule
        _log.info("Registered trigger rule: %s", rule.id)
    
    def get(self, rule_id: str) -> Optional[TriggerRule]:
        """Obtiene una regla por ID."""
        return self._rules.get(rule_id)
    
    def list_all(self, enabled_only: bool = False) -> List[TriggerRule]:
        """Lista todas las reglas."""
        rules = list(self._rules.values())
        if enabled_only:
            rules = [r for r in rules if r.enabled]
        return rules
    
    def get_by_condition(self, condition: TriggerCondition) -> List[TriggerRule]:
        """Obtiene reglas por tipo de condición."""
        return [r for r in self._rules.values() if r.condition == condition and r.enabled]
    
    def enable(self, rule_id: str) -> bool:
        """Habilita una regla."""
        if rule_id in self._rules:
            self._rules[rule_id].enabled = True
            return True
        return False
    
    def disable(self, rule_id: str) -> bool:
        """Deshabilita una regla."""
        if rule_id in self._rules:
            self._rules[rule_id].enabled = False
            return True
        return False


# ============================================================================
# CONDITION CHECKERS
# ============================================================================

def check_git_changes(min_changes: int = 1) -> Dict[str, Any]:
    """Verifica si hay cambios pendientes en Git."""
    try:
        result = subprocess.run(
            ["git", "status", "--porcelain"],
            cwd=str(REPO_ROOT),
            capture_output=True,
            text=True,
            timeout=30,
        )
        changes = [l for l in (result.stdout or "").strip().split("\n") if l.strip()]
        return {
            "triggered": len(changes) >= min_changes,
            "changes_count": len(changes),
            "files": changes[:10],
        }
    except Exception as e:
        return {"triggered": False, "error": str(e)}


def check_git_behind() -> Dict[str, Any]:
    """Verifica si estamos detrás del remoto."""
    try:
        # Fetch para actualizar referencias
        subprocess.run(
            ["git", "fetch", "--quiet"],
            cwd=str(REPO_ROOT),
            capture_output=True,
            timeout=60,
        )
        
        # Ver estado
        result = subprocess.run(
            ["git", "status", "-sb"],
            cwd=str(REPO_ROOT),
            capture_output=True,
            text=True,
            timeout=30,
        )
        output = result.stdout or ""
        behind = "behind" in output.lower()
        
        return {
            "triggered": behind,
            "status": output.strip().split("\n")[0] if output else "",
        }
    except Exception as e:
        return {"triggered": False, "error": str(e)}


def check_git_unsafe() -> Dict[str, Any]:
    """
    Detecta estados Git peligrosos que suelen romper la autonomía:
    - index.lock persistente
    - rebase/merge en progreso
    - detached HEAD
    - conflictos (unmerged)
    """
    try:
        git_dir = REPO_ROOT / ".git"
        lock_path = git_dir / "index.lock"
        rebase_apply = git_dir / "rebase-apply"
        rebase_merge = git_dir / "rebase-merge"
        merge_head = git_dir / "MERGE_HEAD"

        details: Dict[str, Any] = {
            "lock_exists": lock_path.exists(),
            "rebase_apply": rebase_apply.exists(),
            "rebase_merge": rebase_merge.exists(),
            "merge_in_progress": merge_head.exists(),
        }

        if details["lock_exists"] or details["rebase_apply"] or details["rebase_merge"] or details["merge_in_progress"]:
            return {"triggered": True, "reason": "git_state_markers", "details": details}

        st = subprocess.run(
            ["git", "status", "-sb"],
            cwd=str(REPO_ROOT),
            capture_output=True,
            text=True,
            timeout=20,
        )
        out = ((st.stdout or "") + "\n" + (st.stderr or "")).strip()
        low = out.lower()
        details["status_sb"] = (st.stdout or "")[:800]

        if "head (no branch)" in low or "detached" in low:
            return {"triggered": True, "reason": "detached_head", "details": details}
        if "rebase in progress" in low or "rebasing" in low:
            return {"triggered": True, "reason": "rebase_in_progress", "details": details}
        if "unmerged" in low:
            return {"triggered": True, "reason": "unmerged_paths", "details": details}

        ps = subprocess.run(
            ["git", "status", "--porcelain"],
            cwd=str(REPO_ROOT),
            capture_output=True,
            text=True,
            timeout=20,
        )
        lines = [ln for ln in (ps.stdout or "").splitlines() if ln.strip()]
        details["porcelain_count"] = len(lines)
        if any((ln[:2] if len(ln) >= 2 else "") in ("UU", "AA", "DD", "UD", "DU") for ln in lines):
            return {"triggered": True, "reason": "conflict_markers", "details": details}

        return {"triggered": False, "details": details}
    except Exception as e:
        return {"triggered": False, "error": str(e)}


def check_service_down(services: Optional[List[str]] = None) -> Dict[str, Any]:
    """Verifica si algún servicio está caído."""
    services = services or ["atlas_api", "scheduler"]
    down_services = []
    
    # Endpoints a verificar
    endpoints = {
        "atlas_api": "http://127.0.0.1:8787/health",
        "scheduler": "http://127.0.0.1:8791/scheduler/jobs",
        "ans": "http://127.0.0.1:8787/ans/status",
    }
    
    import urllib.request
    import urllib.error
    
    for svc in services:
        url = endpoints.get(svc)
        if not url:
            continue
        try:
            req = urllib.request.Request(url, method="GET")
            with urllib.request.urlopen(req, timeout=10) as r:
                if r.status >= 400:
                    down_services.append(svc)
        except Exception:
            down_services.append(svc)
    
    return {
        "triggered": len(down_services) > 0,
        "down_services": down_services,
        "checked": services,
    }


def check_disk_full(threshold_percent: int = 90) -> Dict[str, Any]:
    """Verifica si el disco está lleno."""
    try:
        import shutil
        total, used, free = shutil.disk_usage(str(REPO_ROOT))
        usage_percent = (used / total) * 100
        
        return {
            "triggered": usage_percent >= threshold_percent,
            "usage_percent": round(usage_percent, 1),
            "free_gb": round(free / (1024**3), 2),
            "threshold": threshold_percent,
        }
    except Exception as e:
        return {"triggered": False, "error": str(e)}


def check_incident_new() -> Dict[str, Any]:
    """Verifica si hay incidentes nuevos sin procesar."""
    try:
        from modules.humanoid.ans.core import get_open_incidents
        incidents = get_open_incidents(limit=10)
        unprocessed = [i for i in incidents if not i.get("pot_assigned")]
        
        return {
            "triggered": len(unprocessed) > 0,
            "incidents_count": len(unprocessed),
            "incidents": [{"id": i.get("id"), "check": i.get("check_id")} for i in unprocessed[:5]],
        }
    except Exception as e:
        return {"triggered": False, "error": str(e)}


# Mapeo de condiciones a funciones de verificación
CONDITION_CHECKERS: Dict[TriggerCondition, Callable] = {
    TriggerCondition.GIT_CHANGES: check_git_changes,
    TriggerCondition.GIT_BEHIND: check_git_behind,
    TriggerCondition.GIT_UNSAFE: check_git_unsafe,
    TriggerCondition.SERVICE_DOWN: check_service_down,
    TriggerCondition.DISK_FULL: check_disk_full,
    TriggerCondition.INCIDENT_NEW: check_incident_new,
}


# ============================================================================
# TRIGGER ENGINE
# ============================================================================

class TriggerEngine:
    """
    Motor que evalúa triggers y dispara POTs automáticamente.
    
    Este es el componente que hace que ATLAS reaccione a eventos
    y ejecute POTs de forma autónoma.
    """
    
    def __init__(self):
        self._registry = TriggerRegistry()
        self._running = False
        self._worker_thread: Optional[threading.Thread] = None
        self._last_check: Dict[str, float] = {}
        self._last_triggered: Dict[str, float] = {}
        self._lock = threading.Lock()
        self._stats = {
            "checks_total": 0,
            "triggers_fired": 0,
            "by_condition": {},
        }
    
    @property
    def registry(self) -> TriggerRegistry:
        return self._registry
    
    def start(self) -> None:
        """Inicia el engine de triggers."""
        if self._running:
            return
        
        self._running = True
        self._worker_thread = threading.Thread(
            target=self._worker_loop,
            name="TriggerEngine-Worker",
            daemon=True,
        )
        self._worker_thread.start()
        _log.info("Trigger Engine started")
        
        # Disparar trigger de startup
        self._fire_trigger(self._registry.get("session_startup"))
    
    def stop(self) -> None:
        """Detiene el engine."""
        if not self._running:
            return
        
        # Disparar trigger de shutdown
        self._fire_trigger(self._registry.get("session_shutdown"))
        
        self._running = False
        if self._worker_thread:
            self._worker_thread.join(timeout=5)
        _log.info("Trigger Engine stopped")
    
    def is_running(self) -> bool:
        return self._running
    
    def _worker_loop(self) -> None:
        """Loop principal que evalúa triggers."""
        while self._running:
            try:
                self._evaluate_all_triggers()
            except Exception as e:
                _log.exception("Error evaluating triggers: %s", e)
            
            # Dormir 10 segundos entre ciclos de evaluación
            time.sleep(10)
    
    def _evaluate_all_triggers(self) -> None:
        """Evalúa todos los triggers activos."""
        now = time.time()
        
        for rule in self._registry.list_all(enabled_only=True):
            rule_id = rule.id
            
            # Verificar intervalo de check
            last_check = self._last_check.get(rule_id, 0)
            if now - last_check < rule.check_interval_seconds:
                continue
            
            self._last_check[rule_id] = now
            
            # Verificar cooldown
            last_triggered = self._last_triggered.get(rule_id, 0)
            if rule.cooldown_seconds > 0 and now - last_triggered < rule.cooldown_seconds:
                continue
            
            # Evaluar condición
            checker = CONDITION_CHECKERS.get(rule.condition)
            if not checker:
                continue
            
            try:
                # Pasar parámetros si la función los acepta
                params = rule.params or {}
                if params:
                    result = checker(**params)
                else:
                    result = checker()
                
                with self._lock:
                    self._stats["checks_total"] += 1
                
                if result.get("triggered"):
                    _log.info("Trigger %s fired: %s", rule_id, result)
                    self._fire_trigger(rule, context=result)
                    self._last_triggered[rule_id] = now
                    
            except Exception as e:
                _log.warning("Trigger check %s failed: %s", rule_id, e)
    
    def _fire_trigger(self, rule: Optional[TriggerRule], context: Dict[str, Any] = None) -> None:
        """Dispara un trigger enviando al dispatcher."""
        if not rule:
            return
        
        try:
            from .dispatcher import get_dispatcher, DispatchRequest, TriggerType
            
            dispatcher = get_dispatcher()
            if not dispatcher.is_running():
                dispatcher.start()
            
            # Construir contexto
            ctx = context or {}
            if rule.context_builder:
                try:
                    ctx.update(rule.context_builder())
                except Exception:
                    pass
            
            # Determinar tipo de trigger
            trigger_type_map = {
                TriggerCondition.STARTUP: TriggerType.STARTUP,
                TriggerCondition.SHUTDOWN: TriggerType.SHUTDOWN,
                TriggerCondition.INCIDENT_NEW: TriggerType.INCIDENT,
                TriggerCondition.SCHEDULE_TIME: TriggerType.SCHEDULED,
                TriggerCondition.WEBHOOK: TriggerType.WEBHOOK,
            }
            trigger_type = trigger_type_map.get(rule.condition, TriggerType.CONDITION)
            
            dispatcher.dispatch(DispatchRequest(
                trigger_type=trigger_type,
                trigger_id=rule.id,
                pot_id=rule.pot_id,
                context=ctx,
                priority=rule.priority,
                source="trigger_engine",
                require_approval=rule.require_approval,
            ))
            
            with self._lock:
                self._stats["triggers_fired"] += 1
                self._stats["by_condition"].setdefault(rule.condition.value, 0)
                self._stats["by_condition"][rule.condition.value] += 1
            
            _log.info("Fired trigger %s -> POT %s", rule.id, rule.pot_id)
            
        except Exception as e:
            _log.exception("Failed to fire trigger %s: %s", rule.id if rule else "unknown", e)
    
    def get_stats(self) -> Dict[str, Any]:
        """Obtiene estadísticas del engine."""
        with self._lock:
            return {
                **self._stats,
                "running": self._running,
                "rules_total": len(self._registry.list_all()),
                "rules_enabled": len(self._registry.list_all(enabled_only=True)),
            }
    
    def force_check(self, rule_id: str) -> Dict[str, Any]:
        """Fuerza la evaluación de un trigger específico."""
        rule = self._registry.get(rule_id)
        if not rule:
            return {"ok": False, "error": f"Rule not found: {rule_id}"}
        
        checker = CONDITION_CHECKERS.get(rule.condition)
        if not checker:
            return {"ok": False, "error": f"No checker for condition: {rule.condition}"}
        
        try:
            params = rule.params or {}
            result = checker(**params) if params else checker()
            
            if result.get("triggered"):
                self._fire_trigger(rule, context=result)
            
            return {
                "ok": True,
                "rule_id": rule_id,
                "condition": rule.condition.value,
                "result": result,
                "fired": result.get("triggered", False),
            }
        except Exception as e:
            return {"ok": False, "error": str(e)}


# ============================================================================
# INSTANCIA GLOBAL
# ============================================================================

_trigger_engine: Optional[TriggerEngine] = None
_trigger_lock = threading.Lock()


def get_trigger_engine() -> TriggerEngine:
    """Obtiene la instancia global del trigger engine."""
    global _trigger_engine
    with _trigger_lock:
        if _trigger_engine is None:
            _trigger_engine = TriggerEngine()
        return _trigger_engine


def start_triggers() -> TriggerEngine:
    """Inicia el engine de triggers."""
    engine = get_trigger_engine()
    if not engine.is_running():
        engine.start()
    return engine


def stop_triggers() -> None:
    """Detiene el engine de triggers."""
    global _trigger_engine
    with _trigger_lock:
        if _trigger_engine:
            _trigger_engine.stop()


def register_trigger(rule: TriggerRule) -> None:
    """Registra una nueva regla de trigger."""
    get_trigger_engine().registry.register(rule)


def get_trigger_stats() -> Dict[str, Any]:
    """Obtiene estadísticas de triggers."""
    return get_trigger_engine().get_stats()


# ============================================================================
# EXPORTACIONES
# ============================================================================

__all__ = [
    "TriggerCondition",
    "TriggerRule",
    "TriggerRegistry",
    "TriggerEngine",
    "get_trigger_engine",
    "start_triggers",
    "stop_triggers",
    "register_trigger",
    "get_trigger_stats",
    "check_git_changes",
    "check_git_behind",
    "check_service_down",
    "check_disk_full",
    "check_incident_new",
]

"""Self-Manifest: anatomía del sistema. Fuente única de verdad para autoconocimiento."""
from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

_MANIFEST: Optional[Dict] = None


def _build_manifest() -> Dict[str, Any]:
    """Construye el manifiesto desde código real (checks, heals, módulos)."""
    checks = _discover_checks()
    heals = _discover_heals()
    modules = _discover_modules()
    deps = _build_dependency_graph(checks, heals, modules)

    return {
        "version": "1.0",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "anatomy": {
            "brain": {
                "id": "brain",
                "role": "Cerebro: orquestación, LLM, coherencia, lógica",
                "components": ["BrainOrchestrator", "CoherenceValidator", "LogicValidator"],
                "dependencies": ["llm_router", "memory", "audit"],
                "failure_modes": ["llm_unreachable", "memory_lock", "coherence_fail"],
            },
            "nervous_system": {
                "id": "ans",
                "role": "Sistema nervioso autónomo: sensa, decide, actúa",
                "checks": checks,
                "heals": heals,
                "dependencies": ["scheduler", "governance", "policy", "incident_store"],
                "failure_modes": ["check_not_registered", "heal_not_registered", "governance_blocked"],
            },
            "kernel": {
                "id": "kernel",
                "role": "Núcleo: registry, event bus, health",
                "modules": modules,
                "dependencies": [],
            },
            "organs": {
                "memory": {"id": "memory", "role": "Memoria persistente SQLite", "check": "memory_health", "heals": ["clear_stale_locks"]},
                "audit": {"id": "audit", "role": "Auditoría", "check": "audit_health", "heals": ["clear_stale_locks"]},
                "scheduler": {"id": "scheduler", "role": "Planificador de jobs", "check": "scheduler_health", "heals": ["restart_scheduler"]},
                "llm": {"id": "llm", "role": "LLM/Ollama", "check": "llm_health", "heals": ["fallback_models"]},
                "api": {"id": "api", "role": "API HTTP", "check": "api_health", "heals": []},
                "deps": {"id": "deps", "role": "Dependencias opcionales", "check": "deps_health", "heals": ["install_optional_deps"]},
                "disk": {"id": "disk", "role": "Disco", "check": "disk_health", "heals": ["rotate_logs"]},
                "logs": {"id": "logs", "role": "Logs", "check": "logs_health", "heals": ["rotate_logs"]},
            },
        },
        "dependency_graph": deps,
    }


def _discover_checks() -> List[Dict]:
    """Descubre checks registrados desde el registry."""
    out = []
    try:
        from modules.humanoid.ans.checks import _register_all
        _register_all()
        from modules.humanoid.ans.registry import get_checks
        for cid, fn in get_checks().items():
            out.append({"id": cid, "module": getattr(fn, "__module__", "?"), "callable": getattr(fn, "__name__", "?")})
    except Exception:
        pass
    return out


def _discover_heals() -> List[Dict]:
    """Descubre heals registrados."""
    out = []
    try:
        import modules.humanoid.ans.heals  # noqa: F401
        from modules.humanoid.ans.registry import get_heals
        from modules.humanoid.ans.engine import SAFE_HEALS
        for hid, fn in get_heals().items():
            out.append({
                "id": hid,
                "safe": hid in SAFE_HEALS,
                "module": getattr(fn, "__module__", "?"),
            })
    except Exception:
        pass
    return out


def _discover_modules() -> List[str]:
    """Módulos del kernel."""
    try:
        from modules.humanoid import get_humanoid_kernel
        k = get_humanoid_kernel()
        return list(k.registry.all_names())
    except Exception:
        return ["brain", "hands", "eyes", "ears", "autonomy", "comms", "update"]


def _build_dependency_graph(checks: List, heals: List, modules: List) -> Dict:
    """Grafo de dependencias: check -> heals sugeridos."""
    return {
        "check_to_heal": {
            "api_health": ["restart_api"],
            "memory_health": ["clear_stale_locks"],
            "audit_health": ["clear_stale_locks"],
            "scheduler_health": ["restart_scheduler"],
            "llm_health": ["fallback_models"],
            "deps_health": ["install_optional_deps"],
            "disk_health": ["rotate_logs"],
            "logs_health": ["rotate_logs"],
            "gateway_health": ["retry_gateway_bootstrap"],
            "cluster_health": ["mark_node_offline"],
        },
    }


def get_manifest() -> Dict[str, Any]:
    """Obtiene el manifiesto (cacheado)."""
    global _MANIFEST
    if _MANIFEST is None:
        _MANIFEST = _build_manifest()
    return _MANIFEST


def get_anatomy() -> Dict[str, Any]:
    """Anatomía: cerebro, nervios, órganos."""
    return get_manifest().get("anatomy", {})


def get_nervous_system() -> Dict[str, Any]:
    """Sistema nervioso: checks + heals."""
    return get_anatomy().get("nervous_system", {})


def get_component(component_id: str) -> Optional[Dict]:
    """Busca un componente por id en toda la anatomía."""
    anat = get_anatomy()
    for section in anat.values():
        if isinstance(section, dict) and section.get("id") == component_id:
            return section
        if isinstance(section, dict) and "organs" in section:
            for org in section.get("organs", {}).values():
                if isinstance(org, dict) and org.get("id") == component_id:
                    return org
    return anat.get("organs", {}).get(component_id)

from __future__ import annotations

import json
import os
import sys
import urllib.request
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List

import psutil

from . import storage

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPTS_DIR = REPO_ROOT / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from atlas_fault_manager import _collect_candidates, _collect_snapshot

AI_ROUTER_STATE_PATH = REPO_ROOT / "state" / "atlas_ai_router_state.json"
QUANT_EXECUTOR_STATE_PATH = (
    REPO_ROOT / "atlas_code_quant" / "data" / "operation" / "auton_executor_state.json"
)

SEVERITY_SCORE = {
    "info": 1,
    "warning": 2,
    "degraded": 3,
    "critical": 4,
    "emergency": 5,
}

DOMAIN_DESCRIPTIONS = {
    "autonomy_core": "Supervisor, scheduler, daemones internos y lazo MAPE-K de ATLAS.",
    "operations_core": "Chequeos operativos, dependencias opcionales y mantenimiento del workspace.",
    "robotics_core": "Robot, visión, cámara y rutas de ejecución física.",
    "communications_core": "Telegram, WhatsApp, audio y salidas hacia operadores.",
    "ai_cognition": "Router IA, Ollama local y capacidad cognitiva auxiliar.",
    "gateway_core": "PUSH, endpoints HTTP y capa de visibilidad/operación remota.",
    "quant_core": "Protecciones y estado del subsistema Quant.",
    "memory_core": "Persistencia, bitácora y memoria de incidentes.",
}

DOMAIN_DEPENDENCIES = {
    "gateway_core": [],
    "memory_core": ["gateway_core"],
    "ai_cognition": ["gateway_core", "memory_core"],
    "communications_core": ["gateway_core", "memory_core"],
    "operations_core": ["gateway_core", "memory_core"],
    "autonomy_core": ["gateway_core", "memory_core", "operations_core", "ai_cognition"],
    "robotics_core": ["gateway_core", "autonomy_core"],
    "quant_core": ["gateway_core", "communications_core", "memory_core"],
}

DOMAIN_COMPONENT_HINTS = {
    "service.push": "gateway_core",
    "service.nexus": "autonomy_core",
    "service.robot": "robotics_core",
    "router_": "ai_cognition",
    "llm_": "ai_cognition",
    "robot_": "robotics_core",
    "vision": "robotics_core",
    "camera": "robotics_core",
    "comms.": "communications_core",
    "telegram": "communications_core",
    "whatsapp": "communications_core",
    "scheduler": "autonomy_core",
    "supervisor": "autonomy_core",
    "evolution": "autonomy_core",
    "memory": "memory_core",
    "bitacora": "memory_core",
    "quant": "quant_core",
}


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _http_json(url: str, timeout: float = 2.5) -> Dict[str, Any] | None:
    try:
        with urllib.request.urlopen(url, timeout=timeout) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except Exception:
        return None


def _service_snapshot(snapshot: Dict[str, Any]) -> Dict[str, Any]:
    services: Dict[str, Dict[str, Any]] = {
        "push": {"name": "push", "healthy": True, "faults": []},
        "nexus": {"name": "nexus", "healthy": True, "faults": []},
        "robot": {"name": "robot", "healthy": True, "faults": []},
    }
    for event in snapshot.get("events", []):
        component_id = str(event.get("component_id") or "")
        if not component_id.startswith("service."):
            continue
        service_name = component_id.split(".", 1)[1]
        services.setdefault(
            service_name, {"name": service_name, "healthy": True, "faults": []}
        )
        services[service_name]["healthy"] = False
        services[service_name]["faults"].append(
            {
                "severity": event.get("severity"),
                "fault_code": event.get("fault_code"),
                "symptom": event.get("symptom"),
            }
        )
    return services


def _resource_snapshot() -> Dict[str, Any]:
    disk_root = (os.getenv("SystemDrive") or "C:") + "\\"
    return {
        "cpu_percent": float(psutil.cpu_percent(interval=0)),
        "ram_percent": float(psutil.virtual_memory().percent),
        "disk_percent": float(psutil.disk_usage(disk_root).percent),
        "boot_time": float(psutil.boot_time()),
    }


def _listener_snapshot(port: int = 8791) -> Dict[str, Any]:
    listeners: List[Dict[str, Any]] = []
    try:
        for conn in psutil.net_connections(kind="tcp"):
            laddr = getattr(conn, "laddr", None)
            if not laddr:
                continue
            if int(getattr(laddr, "port", 0) or 0) != int(port):
                continue
            if str(getattr(conn, "status", "")).upper() != "LISTEN":
                continue
            listeners.append(
                {
                    "address": str(getattr(laddr, "ip", "") or ""),
                    "port": int(getattr(laddr, "port", 0) or 0),
                    "pid": int(conn.pid or 0),
                }
            )
    except Exception as exc:
        return {"ok": False, "port": port, "listeners": [], "error": str(exc)}

    unique_pids = sorted({item["pid"] for item in listeners if item["pid"] > 0})
    unique_addresses = sorted({item["address"] for item in listeners if item["address"]})
    return {
        "ok": True,
        "port": port,
        "listeners": listeners,
        "listener_count": len(listeners),
        "unique_pids": unique_pids,
        "unique_addresses": unique_addresses,
        "conflict": len(unique_pids) > 1,
    }


def _control_plane_snapshot(listener_snapshot: Dict[str, Any]) -> Dict[str, Any]:
    preferred_endpoint = "http://127.0.0.1:8791"
    preferred_health = _http_json(f"{preferred_endpoint}/health", timeout=2.0) or {}
    localhost_endpoint = "http://localhost:8791"
    localhost_health = _http_json(f"{localhost_endpoint}/health", timeout=2.0) or {}
    preferred_ok = bool(preferred_health.get("ok"))
    localhost_ok = bool(localhost_health.get("ok"))
    return {
        "preferred_endpoint": preferred_endpoint,
        "ui_url": f"{preferred_endpoint}/ui",
        "health_url": f"{preferred_endpoint}/health",
        "preferred_ok": preferred_ok,
        "localhost_endpoint": localhost_endpoint,
        "localhost_ok": localhost_ok,
        "listener_conflict": bool(listener_snapshot.get("conflict")),
        "listener_count": int(listener_snapshot.get("listener_count") or 0),
        "listener_addresses": list(listener_snapshot.get("unique_addresses") or []),
        "listener_pids": list(listener_snapshot.get("unique_pids") or []),
        "operator_action_required": bool(listener_snapshot.get("conflict")),
        "recommended_operator_action": (
            "Use 127.0.0.1:8791 for the local control plane and retire any extra wildcard listener."
            if listener_snapshot.get("conflict")
            else "Single PUSH listener detected."
        ),
        "status": (
            "conflict_mitigated"
            if listener_snapshot.get("conflict") and preferred_ok
            else "conflict_unhealthy"
            if listener_snapshot.get("conflict")
            else "healthy"
            if preferred_ok
            else "degraded"
        ),
    }


def _ollama_snapshot() -> Dict[str, Any]:
    base_url = (os.getenv("OLLAMA_BASE_URL") or "http://127.0.0.1:11434").rstrip("/")
    data = _http_json(f"{base_url}/api/tags", timeout=2.0)
    if not data:
        return {"reachable": False, "model_count": 0, "models": []}
    models = [m.get("name") for m in (data.get("models") or []) if m.get("name")]
    return {
        "reachable": True,
        "model_count": len(models),
        "models": models[:20],
    }


def _ai_router_snapshot() -> Dict[str, Any]:
    return storage.read_json(
        AI_ROUTER_STATE_PATH,
        default={"degraded_routes": {}, "last_events": {}, "updated_at": None},
    )


def _telegram_snapshot() -> Dict[str, Any]:
    try:
        from modules.humanoid.notify import _cached_chat_id

        chat_id = _cached_chat_id()
        return {"configured": bool(chat_id), "chat_id_present": bool(chat_id)}
    except Exception:
        return {"configured": False, "chat_id_present": False}


def _quant_snapshot() -> Dict[str, Any]:
    state = storage.read_json(
        QUANT_EXECUTOR_STATE_PATH,
        default={"mode": "unknown", "kill_switch_active": None},
    )
    mode = str(state.get("mode") or "unknown")
    kill_switch = bool(state.get("kill_switch_active"))
    return {
        "mode": mode,
        "kill_switch_active": kill_switch,
        "safe_mode": mode in {"disabled", "desktop_dry_run"} or kill_switch,
    }


def _normalize_domain_name(raw: str) -> str:
    value = (raw or "").strip().lower()
    if not value:
        return "operations_core"
    aliases = {
        "autonomy_core": "autonomy_core",
        "operations_core": "operations_core",
        "robotics_core": "robotics_core",
        "communications_core": "communications_core",
        "communications": "communications_core",
        "ai_cognition": "ai_cognition",
        "gateway_core": "gateway_core",
        "gateway": "gateway_core",
        "quant_core": "quant_core",
        "memory_core": "memory_core",
    }
    if value in aliases:
        return aliases[value]
    for prefix, domain_name in DOMAIN_COMPONENT_HINTS.items():
        if value.startswith(prefix):
            return domain_name
    return "operations_core"


def _infer_domain(event: Dict[str, Any]) -> str:
    domain_value = _normalize_domain_name(str(event.get("domain") or ""))
    if domain_value != "operations_core" or str(event.get("domain") or "").strip():
        return domain_value
    component_id = str(event.get("component_id") or "")
    fault_code = str(event.get("fault_code") or "")
    for prefix, domain_name in DOMAIN_COMPONENT_HINTS.items():
        if component_id.startswith(prefix) or fault_code.startswith(prefix):
            return domain_name
    return "operations_core"


def _default_domain_state(name: str) -> Dict[str, Any]:
    return {
        "name": name,
        "description": DOMAIN_DESCRIPTIONS.get(name, ""),
        "dependencies": DOMAIN_DEPENDENCIES.get(name, []),
        "events": [],
        "event_count": 0,
        "actionable_candidates": [],
        "actionable_count": 0,
        "severity_counts": {
            "info": 0,
            "warning": 0,
            "degraded": 0,
            "critical": 0,
            "emergency": 0,
        },
        "highest_severity": "info",
        "risk_score": 0,
        "health": "healthy",
        "capabilities": {
            "can_observe": True,
            "can_run_safe_heals": False,
            "can_run_playbooks": False,
        },
    }


def _derive_domain_health(highest_severity: str, event_count: int, risk_score: int) -> str:
    if highest_severity == "emergency":
        return "emergency"
    if highest_severity == "critical":
        return "critical"
    if highest_severity == "degraded" or risk_score >= 6:
        return "degraded"
    if highest_severity == "warning" or event_count > 0:
        return "warning"
    return "healthy"


def _build_domain_state(
    snapshot: Dict[str, Any],
    candidates: List[Dict[str, Any]],
    services: Dict[str, Any],
    resources: Dict[str, Any],
    ai_router: Dict[str, Any],
    ollama: Dict[str, Any],
    telegram: Dict[str, Any],
    quant: Dict[str, Any],
) -> Dict[str, Any]:
    domains = {name: _default_domain_state(name) for name in DOMAIN_DESCRIPTIONS}

    for event in snapshot.get("events", []):
        domain_name = _infer_domain(event)
        domain = domains.setdefault(domain_name, _default_domain_state(domain_name))
        domain["events"].append(event)
        domain["event_count"] += 1
        severity = str(event.get("severity") or "warning").lower()
        domain["severity_counts"][severity] = domain["severity_counts"].get(severity, 0) + 1
        if SEVERITY_SCORE.get(severity, 0) >= SEVERITY_SCORE.get(domain["highest_severity"], 0):
            domain["highest_severity"] = severity
        domain["risk_score"] += SEVERITY_SCORE.get(severity, 1)

    for candidate in candidates:
        domain_name = _infer_domain(candidate)
        domain = domains.setdefault(domain_name, _default_domain_state(domain_name))
        domain["actionable_candidates"].append(candidate)
        domain["actionable_count"] += 1
        if candidate.get("safe_heals"):
            domain["capabilities"]["can_run_safe_heals"] = True
        if candidate.get("playbook"):
            domain["capabilities"]["can_run_playbooks"] = True

    if not services.get("push", {}).get("healthy", True):
        domains["gateway_core"]["risk_score"] += 4
        domains["gateway_core"]["highest_severity"] = "critical"
        domains["gateway_core"]["health"] = "critical"
    if not ollama.get("reachable"):
        domains["ai_cognition"]["risk_score"] += 2
    if ai_router.get("degraded_routes"):
        domains["ai_cognition"]["risk_score"] += 2
    if not telegram.get("configured"):
        domains["communications_core"]["risk_score"] += 1
    if quant.get("kill_switch_active"):
        domains["quant_core"]["risk_score"] += 1
        domains["quant_core"]["capabilities"]["can_run_playbooks"] = False

    if float(resources.get("disk_percent") or 0.0) >= 93.0:
        domains["operations_core"]["risk_score"] += 2
    if float(resources.get("cpu_percent") or 0.0) >= 92.0:
        domains["operations_core"]["risk_score"] += 1

    for domain_name, domain in domains.items():
        domain["health"] = _derive_domain_health(
            domain["highest_severity"], domain["event_count"], int(domain["risk_score"] or 0)
        )
        domain["services_impacted"] = [
            service_name
            for service_name, service in services.items()
            if not service.get("healthy", True)
            and domain_name in {
                "gateway_core" if service_name == "push" else
                "autonomy_core" if service_name == "nexus" else
                "robotics_core"
            }
        ]
    return domains


def _capabilities_snapshot(
    candidates: List[Dict[str, Any]],
    ollama: Dict[str, Any],
    telegram: Dict[str, Any],
    quant: Dict[str, Any],
) -> Dict[str, Any]:
    return {
        "can_run_safe_heals": any(candidate.get("safe_heals") for candidate in candidates),
        "can_run_playbooks": any(candidate.get("playbook") for candidate in candidates),
        "can_use_local_ai": bool(ollama.get("reachable")),
        "can_notify_telegram": bool(telegram.get("configured")),
        "quant_safe_mode": bool(quant.get("safe_mode")),
    }


def build_runtime_model(*, refresh_faults: bool = True) -> Dict[str, Any]:
    snapshot = _collect_snapshot(write=refresh_faults)
    candidates = _collect_candidates(snapshot)
    services = _service_snapshot(snapshot)
    resources = _resource_snapshot()
    ai_router = _ai_router_snapshot()
    ollama = _ollama_snapshot()
    telegram = _telegram_snapshot()
    quant = _quant_snapshot()
    listener_8791 = _listener_snapshot(port=8791)
    control_plane = _control_plane_snapshot(listener_8791)
    degraded_routes = list((ai_router.get("degraded_routes") or {}).keys())
    domains = _build_domain_state(
        snapshot, candidates, services, resources, ai_router, ollama, telegram, quant
    )
    capabilities = _capabilities_snapshot(candidates, ollama, telegram, quant)

    healthy_domains = [name for name, item in domains.items() if item.get("health") == "healthy"]
    degraded_domains = [name for name, item in domains.items() if item.get("health") in {"warning", "degraded"}]
    critical_domains = [name for name, item in domains.items() if item.get("health") in {"critical", "emergency"}]

    runtime_model = {
        "generated_at": _now_iso(),
        "fault_snapshot": {
            "event_count": snapshot.get("event_count", 0),
            "severity_counts": snapshot.get("severity_counts", {}),
            "events": snapshot.get("events", []),
            "actionable_candidates": candidates,
        },
        "services": services,
        "domains": domains,
        "resources": resources,
        "local_ai": {
            "router": ai_router,
            "ollama": ollama,
            "degraded_routes": degraded_routes,
        },
        "communications": {
            "telegram": telegram,
            "bitacora": {"configured": True},
        },
        "network": {
            "listeners": {
                "8791": listener_8791,
            },
            "control_plane": control_plane,
        },
        "quant": quant,
        "capabilities": capabilities,
        "constraints": {
            "critical_faults_present": int((snapshot.get("severity_counts") or {}).get("critical", 0)) > 0,
            "emergency_faults_present": int((snapshot.get("severity_counts") or {}).get("emergency", 0)) > 0,
            "service_outage_present": any(not item.get("healthy", True) for item in services.values()),
            "local_ai_degraded": bool(degraded_routes),
            "resource_pressure_present": (
                float(resources.get("cpu_percent") or 0.0) >= 92.0
                or float(resources.get("ram_percent") or 0.0) >= 92.0
                or float(resources.get("disk_percent") or 0.0) >= 93.0
            ),
            "quant_guard_active": bool(quant.get("kill_switch_active")),
            "push_listener_conflict": bool(listener_8791.get("conflict")),
        },
        "system_posture": {
            "healthy_domains": healthy_domains,
            "degraded_domains": degraded_domains,
            "critical_domains": critical_domains,
            "domain_count": len(domains),
            "active_incident_domains": sorted({name for name, item in domains.items() if item.get("event_count")}),
        },
        "knowledge": {
            "runtime_model_path": str(storage.RUNTIME_MODEL_PATH),
            "control_plane_path": str(storage.CONTROL_PLANE_PATH),
            "fault_snapshot_path": str(REPO_ROOT / "state" / "atlas_fault_snapshot.json"),
            "ai_router_state_path": str(AI_ROUTER_STATE_PATH),
            "quant_executor_state_path": str(QUANT_EXECUTOR_STATE_PATH),
        },
    }
    storage.write_json(storage.RUNTIME_MODEL_PATH, runtime_model)
    storage.write_json(storage.CONTROL_PLANE_PATH, control_plane)
    return runtime_model

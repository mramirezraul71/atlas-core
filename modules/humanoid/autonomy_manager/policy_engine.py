from __future__ import annotations

from datetime import datetime, timezone
from typing import Any, Dict, List

from . import storage


SAFE_MODES = {
    "nominal": "ATLAS puede diagnosticar y ejecutar remediación segura y acotada.",
    "degraded_local": "ATLAS opera con degradación localizada; solo acciones seguras y baja concurrencia.",
    "degraded_system": "ATLAS detecta degradación sistémica; remediación conservadora y sin agresividad.",
    "safe_hold": "ATLAS sostiene el sistema y bloquea acciones sensibles hasta recuperar visibilidad.",
    "manual_assist": "ATLAS preserva contexto y recomienda; ejecución autónoma muy limitada.",
    "emergency": "ATLAS protege el sistema y solo ejecuta interlocks de emergencia.",
}

DOMAIN_DEFAULT_POLICY = {
    "autonomy_core": "allow_safe",
    "operations_core": "allow_safe",
    "robotics_core": "cautious",
    "communications_core": "allow_safe",
    "ai_cognition": "advisory",
    "gateway_core": "protect_visibility",
    "quant_core": "protect_capital",
    "memory_core": "protect_evidence",
}


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _derive_global_mode(runtime_model: Dict[str, Any]) -> tuple[str, str]:
    counts = (runtime_model.get("fault_snapshot") or {}).get("severity_counts") or {}
    critical = int(counts.get("critical", 0) or 0)
    emergency = int(counts.get("emergency", 0) or 0)
    degraded = int(counts.get("degraded", 0) or 0)
    warnings = int(counts.get("warning", 0) or 0)
    constraints = runtime_model.get("constraints") or {}
    posture = runtime_model.get("system_posture") or {}
    critical_domains = posture.get("critical_domains") or []
    degraded_domains = posture.get("degraded_domains") or []

    if emergency > 0:
        return "emergency", "Se detectaron fallos de severidad emergency."
    if constraints.get("service_outage_present") and constraints.get("resource_pressure_present"):
        return "safe_hold", "Hay pérdida de servicio y presión de recursos; ATLAS prioriza sostener el sistema."
    if critical > 0 and len(critical_domains) >= 1:
        return "safe_hold", "Hay dominios críticos activos y ATLAS debe congelar acciones sensibles."
    if critical > 0 or constraints.get("service_outage_present") or len(degraded_domains) >= 3 or degraded >= 4:
        return "degraded_system", "La degradación afecta varios dominios o servicios esenciales."
    if constraints.get("local_ai_degraded") or constraints.get("resource_pressure_present") or degraded > 0 or warnings >= 3:
        return "degraded_local", "La degradación está contenida y ATLAS puede seguir corrigiendo de forma segura."
    return "nominal", "No hay señales relevantes que exijan modo seguro."


def _domain_policy(domain_name: str, state: Dict[str, Any], global_mode: str) -> Dict[str, Any]:
    health = str(state.get("health") or "healthy")
    default_policy = DOMAIN_DEFAULT_POLICY.get(domain_name, "allow_safe")
    allowed = {"observe": True, "ans_cycle": False, "playbook_safe": False, "playbook_disruptive": False}
    reason = "dominio sin restricciones relevantes"
    blocked_by: List[str] = []

    if global_mode == "emergency":
        allowed["observe"] = True
        reason = "modo emergency global"
        blocked_by.append("global_emergency")
    elif global_mode == "safe_hold":
        allowed["observe"] = True
        allowed["ans_cycle"] = domain_name in {"autonomy_core", "operations_core", "memory_core"}
        allowed["playbook_safe"] = domain_name in {"communications_core", "memory_core", "gateway_core"}
        reason = "safe hold global: solo acciones muy acotadas"
        blocked_by.append("global_safe_hold")
    else:
        allowed["observe"] = True
        allowed["ans_cycle"] = health in {"healthy", "warning", "degraded"}
        allowed["playbook_safe"] = health in {"warning", "degraded"} and default_policy != "advisory"
        allowed["playbook_disruptive"] = health == "degraded" and global_mode == "nominal"

    if domain_name == "quant_core":
        allowed["ans_cycle"] = False
        allowed["playbook_safe"] = False
        allowed["playbook_disruptive"] = False
        blocked_by.append("protect_capital")
        reason = "Quant queda bajo interlock de capital salvo validación específica."

    if domain_name == "robotics_core":
        allowed["playbook_disruptive"] = False
        if global_mode in {"degraded_system", "safe_hold", "emergency"}:
            allowed["playbook_safe"] = False
            blocked_by.append("protect_robot")
            reason = "Robótica queda contenida para evitar acciones físicas sin visibilidad total."

    if domain_name == "ai_cognition":
        allowed["ans_cycle"] = False
        allowed["playbook_safe"] = False
        allowed["playbook_disruptive"] = False
        reason = "IA local solo asesora; no ejecuta remediación."

    if domain_name == "memory_core":
        allowed["playbook_disruptive"] = False
        if health in {"critical", "emergency"}:
            allowed["ans_cycle"] = False
            allowed["playbook_safe"] = False
            blocked_by.append("protect_evidence")
            reason = "Memoria degradada: preservar evidencia y evitar escrituras invasivas."

    if health in {"critical", "emergency"} and domain_name not in {"communications_core", "memory_core", "gateway_core"}:
        allowed["playbook_disruptive"] = False
        if global_mode != "nominal":
            allowed["playbook_safe"] = False
        blocked_by.append("critical_domain")

    return {
        "domain": domain_name,
        "health": health,
        "strategy": default_policy,
        "allowed_actions": allowed,
        "blocked_by": blocked_by,
        "reason": reason,
    }


def evaluate_policy(runtime_model: Dict[str, Any]) -> Dict[str, Any]:
    mode, reason = _derive_global_mode(runtime_model)
    domains = runtime_model.get("domains") or {}
    domain_policies = {
        name: _domain_policy(name, state, mode) for name, state in domains.items()
    }
    constraints = runtime_model.get("constraints") or {}
    resources = runtime_model.get("resources") or {}

    interlock_reasons: List[str] = []
    if mode != "nominal":
        interlock_reasons.append(f"global_mode={mode}")
    if constraints.get("service_outage_present"):
        interlock_reasons.append("service_outage_present")
    if constraints.get("resource_pressure_present"):
        interlock_reasons.append(
            f"resource_pressure cpu={resources.get('cpu_percent')} ram={resources.get('ram_percent')} disk={resources.get('disk_percent')}"
        )
    if constraints.get("local_ai_degraded"):
        interlock_reasons.append("local_ai_degraded")
    if constraints.get("quant_guard_active"):
        interlock_reasons.append("quant_guard_active")
    if constraints.get("push_listener_conflict"):
        interlock_reasons.append("push_listener_conflict")

    allow_execution = mode not in {"manual_assist", "emergency"}
    allow_ans_cycle = mode in {"nominal", "degraded_local", "degraded_system", "safe_hold"}
    allow_playbooks = mode in {"nominal", "degraded_local", "degraded_system", "safe_hold"}
    allow_disruptive = mode == "nominal" and not constraints.get("resource_pressure_present")
    active_domains = set((runtime_model.get("system_posture") or {}).get("active_incident_domains") or [])
    mission_phases = [
        {
            "name": "recover_visibility",
            "goal": "Asegurar visibilidad y evidencia del sistema antes de remediar en profundidad.",
            "domains": [d for d in ["gateway_core", "memory_core", "communications_core"] if d in active_domains],
        },
        {
            "name": "stabilize_core",
            "goal": "Estabilizar scheduler, supervisor y núcleo autónomo.",
            "domains": [d for d in ["autonomy_core", "operations_core"] if d in active_domains],
        },
        {
            "name": "recover_embodiment",
            "goal": "Recuperar robótica y visión cuando el núcleo ya es observable.",
            "domains": [d for d in ["robotics_core"] if d in active_domains],
        },
        {
            "name": "preserve_capital",
            "goal": "Mantener Quant bajo interlocks y proteger capital.",
            "domains": [d for d in ["quant_core"] if d in active_domains],
        },
        {
            "name": "optimize_cognition",
            "goal": "Usar IA local solo como capa cognitiva auxiliar cuando el sistema esté estable.",
            "domains": [d for d in ["ai_cognition"] if d in active_domains],
        },
    ]
    mission_phases = [phase for phase in mission_phases if phase["domains"]]
    current_focus = mission_phases[0]["domains"][0] if mission_phases and mission_phases[0]["domains"] else ""

    policy = {
        "generated_at": _now_iso(),
        "mode": mode,
        "mode_description": SAFE_MODES[mode],
        "reason": reason,
        "execution": {
            "allow_execution": allow_execution,
            "allow_ans_cycle": allow_ans_cycle,
            "allow_playbooks": allow_playbooks,
            "allow_disruptive_playbooks": allow_disruptive,
            "max_actions": 1 if mode in {"safe_hold", "manual_assist", "emergency"} else 2 if mode == "degraded_system" else 3,
            "require_post_check": True,
            "require_telegram_for_critical": True,
        },
        "interlocks": {
            "block_quant_openings": True,
            "block_high_risk_actions": mode in {"safe_hold", "manual_assist", "emergency"},
            "require_local_evidence": True,
            "require_ollama_optional": True,
            "allow_local_ai_analysis": bool(
                ((runtime_model.get("capabilities") or {}).get("can_use_local_ai"))
                and mode not in {"emergency", "safe_hold"}
            ),
            "active": interlock_reasons,
        },
        "domain_policies": domain_policies,
        "mission": {
            "priority_order": [
                "gateway_core",
                "memory_core",
                "autonomy_core",
                "communications_core",
                "operations_core",
                "robotics_core",
                "quant_core",
                "ai_cognition",
            ],
            "target_posture": "recover_visibility_then_stabilize",
            "phases": mission_phases,
            "current_focus": current_focus,
        },
        "observability": {
            "emit_bitacora": True,
            "emit_telegram": True,
            "emit_dashboard": True,
        },
    }
    storage.write_json(storage.POLICY_STATE_PATH, policy)
    return policy

from __future__ import annotations

from typing import Any, Dict, List


DOMAIN_TEMPLATES: Dict[str, Dict[str, str]] = {
    "gateway_core": {
        "objective": "Recuperar visibilidad, conectividad y acceso a endpoints críticos.",
        "guardrail": "No ocultar pérdida de visibilidad detrás de remediaciones agresivas.",
        "preferred_action": "playbook_safe",
        "operator_note": "Gateway primero: sin visibilidad no se debe escalar remediación compleja.",
    },
    "memory_core": {
        "objective": "Preservar evidencia y consistencia de memoria/estado.",
        "guardrail": "Evitar escrituras invasivas si la evidencia puede corromperse.",
        "preferred_action": "playbook_safe",
        "operator_note": "La memoria debe conservar trazabilidad incluso en degradación.",
    },
    "autonomy_core": {
        "objective": "Estabilizar el ciclo autónomo, scheduler y supervisor.",
        "guardrail": "Aplicar acciones pequeñas y medibles antes de reintentos agresivos.",
        "preferred_action": "ans_cycle",
        "operator_note": "El núcleo autónomo debe recuperar gobernanza antes de ampliar alcance.",
    },
    "operations_core": {
        "objective": "Restablecer operaciones internas y jobs seguros del sistema.",
        "guardrail": "No encadenar demasiadas acciones si el sistema está bajo presión.",
        "preferred_action": "ans_cycle",
        "operator_note": "Las remediaciones operativas deben ser iterativas y con post-check.",
    },
    "communications_core": {
        "objective": "Mantener Telegram, bitácora y canales de alerta disponibles.",
        "guardrail": "La señalización no debe romperse mientras se corrige el resto.",
        "preferred_action": "playbook_safe",
        "operator_note": "Sin comunicaciones, ATLAS pierde capacidad de rendir cuentas.",
    },
    "robotics_core": {
        "objective": "Proteger embodiment, visión y capacidades físicas.",
        "guardrail": "Bloquear acciones físicas o disruptivas sin visibilidad completa.",
        "preferred_action": "observe",
        "operator_note": "Robótica se mantiene contenida salvo postura nominal y evidencia fuerte.",
    },
    "quant_core": {
        "objective": "Proteger capital y bloquear aperturas sensibles.",
        "guardrail": "Nunca ejecutar remediación que pueda abrir riesgo financiero nuevo.",
        "preferred_action": "observe",
        "operator_note": "Quant permanece bajo interlocks salvo validación explícita de capital y riesgo.",
    },
    "ai_cognition": {
        "objective": "Usar IA local como análisis auxiliar sin convertirla en fuente única de verdad.",
        "guardrail": "La IA no debe ejecutar remediación ni saltar interlocks.",
        "preferred_action": "observe",
        "operator_note": "La capa cognitiva es consultiva; el núcleo seguro sigue siendo determinista.",
    },
}


def _allowed_action_for_kind(kind: str) -> str:
    if kind == "playbook":
        return "playbook_safe"
    if kind == "ans_cycle":
        return "ans_cycle"
    return "observe"


def build_domain_strategy(
    domain_name: str,
    runtime_model: Dict[str, Any],
    policy: Dict[str, Any],
    action: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    action = dict(action or {})
    domain_state = ((runtime_model.get("domains") or {}).get(domain_name)) or {}
    domain_policy = ((policy.get("domain_policies") or {}).get(domain_name)) or {}
    allowed = domain_policy.get("allowed_actions") or {}
    template = DOMAIN_TEMPLATES.get(domain_name, {})

    requested_kind = str(action.get("kind") or "observe_only")
    allowed_key = _allowed_action_for_kind(requested_kind)
    blocked_by: List[str] = list(domain_policy.get("blocked_by") or [])
    prechecks = ["local_evidence_present", "policy_still_allows_action"]
    postchecks = ["refresh_fault_snapshot", "compare_domain_health"]
    health = str(domain_state.get("health") or "healthy")

    if requested_kind == "playbook" and str(action.get("execution_class") or "safe") == "disruptive":
        allowed_key = "playbook_disruptive"
        prechecks.append("disruptive_playbook_explicitly_allowed")

    if domain_name == "quant_core":
        prechecks.extend(["kill_switch_or_capital_guard_active", "no_new_risk_openings"])
    if domain_name == "robotics_core":
        prechecks.extend(["full_visibility_before_embodiment", "protect_robotics"])
    if domain_name == "memory_core":
        prechecks.append("preserve_evidence_before_write")
    if domain_name == "gateway_core":
        postchecks.append("verify_visibility_restored")
    if domain_name == "communications_core":
        postchecks.append("verify_telegram_or_bitacora_signal")

    can_execute = bool(allowed.get(allowed_key))
    if requested_kind == "observe_only":
        can_execute = True
    if not can_execute and allowed_key not in blocked_by:
        blocked_by.append(allowed_key)

    mode = "guided_remediation" if can_execute and requested_kind != "observe_only" else "observe_only"
    reason = str(domain_policy.get("reason") or template.get("operator_note") or "")
    if not can_execute and blocked_by:
        reason = f"{reason} Bloqueado por: {', '.join(blocked_by)}".strip()

    return {
        "domain": domain_name,
        "health": health,
        "objective": template.get("objective") or "Mantener estabilidad y trazabilidad.",
        "guardrail": template.get("guardrail") or "Aplicar solo acciones seguras y auditables.",
        "preferred_action": template.get("preferred_action") or "observe",
        "requested_kind": requested_kind,
        "mode": mode,
        "can_execute": can_execute,
        "blocked_by": blocked_by,
        "reason": reason,
        "prechecks": prechecks,
        "postchecks": postchecks,
        "operator_note": template.get("operator_note") or "",
    }

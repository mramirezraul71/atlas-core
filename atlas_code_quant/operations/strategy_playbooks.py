from __future__ import annotations

from typing import Any


def _family(strategy_type: str) -> str:
    if strategy_type in {"equity_long", "equity_short"}:
        return "accion_direccional"
    if strategy_type in {"long_call", "long_put"}:
        return "opcion_simple_direccional"
    if strategy_type in {"bull_call_debit_spread", "bear_put_debit_spread"}:
        return "spread_debito_direccional"
    if strategy_type in {"bull_put_credit_spread", "bear_call_credit_spread"}:
        return "spread_credito_definido"
    return "estructura_opciones"


def build_strategy_playbook(
    *,
    candidate: dict[str, Any],
    selected: dict[str, Any],
    size_plan: dict[str, Any],
    entry_plan: dict[str, Any],
    exit_plan: dict[str, Any],
    risk_profile: dict[str, Any],
    adaptive_context: dict[str, Any],
    chart_plan: dict[str, Any],
    camera_plan: dict[str, Any],
    account_scope: str,
) -> dict[str, Any]:
    strategy_type = str(selected.get("strategy_type") or "")
    family = _family(strategy_type)
    direction = str(candidate.get("direction") or "neutral")
    timeframe = str(candidate.get("timeframe") or "1h")
    adaptive_bias = float(adaptive_context.get("total_bias") or 0.0)
    camera_fit = float(camera_plan.get("visual_fit_pct") or 0.0)
    effective_risk_pct = float(size_plan.get("risk_budget_pct") or 0.0)
    suggested_size = int(size_plan.get("suggested_size") or 0)

    pre_checks = [
        "abrir grafico principal del setup",
        "abrir confirmacion superior y contexto diario",
        "verificar que la estructura sigue intacta en camara",
        "comprobar que el ticket coincide con la tesis del selector",
    ]
    if adaptive_bias <= -1.0:
        pre_checks.append("hacer doble validacion porque el aprendizaje reciente enfrió este setup")
    if camera_fit < 75.0:
        pre_checks.append("no enviar hasta mejorar enfoque visual de la zona de disparo")

    chart_mission = {
        "auto_open": bool(chart_plan.get("auto_open_supported", False)),
        "targets": chart_plan.get("targets") or [],
        "camera_checkpoints": camera_plan.get("checkpoints") or [],
    }

    capital_guardrails = {
        "account_scope": account_scope,
        "risk_budget_pct": round(effective_risk_pct, 3),
        "risk_budget_usd": round(float(size_plan.get("risk_budget_usd") or 0.0), 2),
        "suggested_size": suggested_size,
        "estimated_notional_usd": round(float(size_plan.get("estimated_notional_usd") or 0.0), 2),
        "capital_source": size_plan.get("capital_source") or "capital",
    }

    if family == "accion_direccional":
        manage_rules = [
            "no perseguir precio si rompe sin ti; esperar continuidad o retesteo limpio",
            "en +1R, evaluar parcial y proteger con trailing de estructura",
            "si la temporalidad superior falla, salir aunque el stop nominal no haya sido alcanzado",
        ]
        review_triggers = [
            "vela de rechazo fuerte contra direccion en timeframe principal",
            "perdida de confirmacion en temporalidad superior",
            "cambio visual de estructura detectado por camara",
        ]
    elif family == "opcion_simple_direccional":
        manage_rules = [
            "buscar desplazamiento rapido; si no aparece, reducir exposicion",
            "vigilar deterioro de prima y no dejar que theta domine la posicion",
            "tomar parcial entre 30% y 45% si el movimiento se extiende con claridad",
        ]
        review_triggers = [
            "prima perdiendo 20%-25% sin extension de precio",
            "volatilidad cayendo mientras el activo se lateraliza",
            "la lectura visual en grafico deja de sostener el setup inicial",
        ]
    elif family == "spread_debito_direccional":
        manage_rules = [
            "entrar solo si el riesgo definido mejora la relacion costo/precision",
            "tomar ganancias parciales entre 35% y 50% del valor maximo esperable",
            "si el activo lateraliza y theta empieza a erosionar la tesis, salir temprano",
        ]
        review_triggers = [
            "fallo de continuidad tras la entrada",
            "debilidad progresiva en la confirmacion superior",
            "beneficio parcial ya alcanzado con deterioro de momentum",
        ]
    else:
        manage_rules = [
            "vender prima solo con confirmacion superior firme y desplazamiento moderado",
            "evitar dejar que el lado corto entre en zona de amenaza sin reaccion",
            "recomprar entre 40% y 60% del credito si el contexto sigue ordenado",
        ]
        review_triggers = [
            "precio acercandose al lado corto",
            "probabilidad dinamica cayendo de forma sostenida",
            "aumento de presion visual o ruptura del rango previsto",
        ]

    autonomy_policy = {
        "stage": "paper_ready" if account_scope == "paper" else "live_blocked",
        "camera_required": True,
        "preview_required": True,
        "selector_mandatory": True,
        "autonomous_chart_opening": True,
        "can_submit_without_supervision": account_scope == "paper" and camera_fit >= 78.0 and adaptive_bias >= -1.0,
    }

    return {
        "family": family,
        "strategy_type": strategy_type,
        "direction": direction,
        "timeframe": timeframe,
        "entry_protocol": {
            "style": entry_plan.get("entry_style") or "",
            "trigger": entry_plan.get("trigger_rule") or "",
            "confirmation": entry_plan.get("confirmation_rule") or "",
            "chart_focus": entry_plan.get("chart_focus") or "",
        },
        "pre_ticket_checks": pre_checks,
        "manage_rules": manage_rules,
        "exit_rules": [
            exit_plan.get("primary_take_profit") or "",
            exit_plan.get("risk_invalidation") or "",
            exit_plan.get("time_stop") or "",
        ],
        "review_triggers": review_triggers,
        "capital_guardrails": capital_guardrails,
        "risk_summary": {
            "break_even_points": risk_profile.get("break_even_points") or [],
            "max_loss_usd": risk_profile.get("max_loss_usd"),
            "reward_to_risk": risk_profile.get("reward_to_risk"),
        },
        "chart_mission": chart_mission,
        "autonomy_policy": autonomy_policy,
        "adaptive_note": (adaptive_context.get("notes") or [None])[0],
    }

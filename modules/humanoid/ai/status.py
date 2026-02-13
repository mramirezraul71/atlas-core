"""AI layer status for GET /ai/status."""
from __future__ import annotations

from typing import Any, Dict, List


def _ollama_available() -> bool:
    try:
        from modules.humanoid.deploy.healthcheck import _check_llm_reachable
        return _check_llm_reachable().get("ok", False)
    except Exception:
        return False


def get_ai_status() -> Dict[str, Any]:
    """Proveedores, modelo por ruta, presupuesto, políticas."""
    from . import policies
    from . import budgets
    from . import telemetry
    from .registry import list_providers, get_model_specs

    ollama_ok = _ollama_available()
    providers = list_providers(ollama_ok)
    specs = get_model_specs(ollama_ok)
    route_to_model = {s.route: s.full_key for s in specs}

    return {
        "ok": True,
        "ollama_available": ollama_ok,
        "providers": [
            {"id": p.id, "name": p.name, "is_free": p.is_free, "available": p.available}
            for p in providers
        ],
        "route_to_model": route_to_model,
        "budget_daily_usd": budgets.budget_daily_usd(),
        "max_cost_per_task_usd": budgets.max_cost_per_task_usd(),
        "spent_today_usd": budgets.spent_today_usd(),
        "external_apis_allowed": policies.external_network_allowed(),
        "paid_api_allowed": policies.paid_api_allowed(),
        "telemetry": telemetry.aggregate_by_provider(),
    }

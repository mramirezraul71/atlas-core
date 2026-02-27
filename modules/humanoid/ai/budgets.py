"""Cost limits and daily budget (for paid APIs)."""
from __future__ import annotations

import os
from typing import Dict

def _env_float(name: str, default: float) -> float:
    try:
        return float((os.getenv(name) or "").strip() or default)
    except Exception:
        return default


BUDGET_STORE: Dict[str, float] = {}  # date_str -> spent_usd


def budget_daily_usd() -> float:
    return _env_float("AI_BUDGET_DAILY_USD", 2.0)


def max_cost_per_task_usd() -> float:
    return _env_float("AI_MAX_COST_PER_TASK_USD", 0.25)


def record_spend(date_str: str, amount_usd: float) -> None:
    BUDGET_STORE[date_str] = BUDGET_STORE.get(date_str, 0.0) + amount_usd


def spent_today_usd() -> float:
    import datetime
    today = datetime.date.today().isoformat()
    return BUDGET_STORE.get(today, 0.0)


def can_spend(estimate_usd: float) -> bool:
    if estimate_usd <= 0:
        return True
    if estimate_usd > max_cost_per_task_usd():
        return False
    return (spent_today_usd() + estimate_usd) <= budget_daily_usd()

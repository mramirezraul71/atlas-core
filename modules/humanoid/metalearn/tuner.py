"""Apply bounded tuning from learned rules. Policy remains authority."""
from __future__ import annotations

import os
from typing import Any, Dict, List, Optional

from . import db


def _allow_autotune() -> bool:
    return os.getenv("METALEARN_ALLOW_AUTOTUNE", "true").strip().lower() in ("1", "true", "yes")


def _risk_shift_max() -> float:
    return float(os.getenv("METALEARN_RISK_SHIFT_MAX", "0.15") or 0.15)


def _canary_max() -> float:
    return float(os.getenv("METALEARN_CANARY_PERCENTAGE_MAX", "0.5") or 0.5)


def _autorun_limit_max() -> int:
    return int(os.getenv("METALEARN_AUTORUN_LIMIT_MAX", "5") or 5)


def _min_samples() -> int:
    return int(os.getenv("METALEARN_MIN_SAMPLES", "10") or 10)


# In-memory current params (loaded from snapshot or applied from rules)
_current_params: Optional[Dict[str, Any]] = None


def get_current_params() -> Dict[str, Any]:
    """Return current tuned params (risk_overrides, router_hints, canary_pct, autorun_limit)."""
    global _current_params
    if _current_params is not None:
        return dict(_current_params)
    return {
        "risk_overrides": {},
        "router_hints": {},
        "canary_pct": None,
        "autorun_limit": None,
    }


def set_current_params(params: Dict[str, Any]) -> None:
    global _current_params
    _current_params = dict(params) if params else {}


def get_risk_overrides() -> Dict[str, str]:
    """
    Return action_type -> risk_level overrides for GA scorer.
    GA scorer should merge with default RISK_MAP; policy still final.
    """
    if not _allow_autotune():
        return {}
    p = get_current_params()
    return dict(p.get("risk_overrides") or {})


def get_router_hints() -> Dict[str, str]:
    """Return route/prompt_type -> preferred model_family hint for LLM router."""
    if not _allow_autotune():
        return {}
    p = get_current_params()
    return dict(p.get("router_hints") or {})


def apply_tuning(rules: List[Dict[str, Any]], snapshot_before: bool = True) -> Dict[str, Any]:
    """
    Apply bounded tuning from learned rules. Saves snapshot before if requested.
    Returns {ok, applied: {risk_overrides, router_hints, canary_pct, autorun_limit}, snapshot_id, error}.
    """
    try:
        if not _allow_autotune():
            return {"ok": False, "applied": {}, "snapshot_id": None, "error": "METALEARN_ALLOW_AUTOTUNE=false"}
        min_samp = _min_samples()
        risk_max = _risk_shift_max()
        canary_max = _canary_max()
        autorun_max = _autorun_limit_max()

        risk_overrides: Dict[str, str] = {}
        router_hints: Dict[str, str] = {}
        canary_pct: Optional[float] = None
        autorun_limit: Optional[int] = None

        risk_order = {"low": 0, "medium": 1, "high": 2, "critical": 3}
        order_to_risk = ["low", "medium", "high", "critical"]

        for r in rules:
            if r.get("sample_count", 0) < min_samp:
                continue
            cond = r.get("conditions") or {}
            action_type = cond.get("action_type")
            adjust = r.get("risk_adjust") or 0
            if action_type and adjust != 0:
                # Map current risk to index, apply bounded shift, map back
                current_risk = cond.get("risk_level", "medium")
                idx = risk_order.get(current_risk, 1)
                shift = max(-risk_max, min(risk_max, adjust))
                new_idx = max(0, min(3, int(idx + (1 if shift > 0 else -1))))
                risk_overrides[action_type] = order_to_risk[new_idx]
            if r.get("router_hint"):
                router_hints[action_type or "default"] = r["router_hint"]
            if r.get("canary_hint") is not None:
                canary_pct = min(canary_max, max(0, float(r["canary_hint"])))
            if autorun_limit is None and action_type:
                # Optional: suggest autorun_limit from safe action success rate
                if (r.get("success_rate") or 0) >= 0.9 and (r.get("approve_rate") or 0) >= 0.9:
                    autorun_limit = min(autorun_max, 3)

        params = {
            "risk_overrides": risk_overrides,
            "router_hints": router_hints,
            "canary_pct": canary_pct,
            "autorun_limit": autorun_limit,
        }
        snapshot_id = None
        if snapshot_before:
            from .rollback import save_snapshot
            snapshot_id = save_snapshot(params, comment="pre_tune")
        set_current_params(params)
        return {"ok": True, "applied": params, "snapshot_id": snapshot_id, "error": None}
    except Exception as e:
        return {"ok": False, "applied": {}, "snapshot_id": None, "error": str(e)}


def restore_params(params: Dict[str, Any]) -> None:
    """Restore params from snapshot (rollback)."""
    set_current_params(params)

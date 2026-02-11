"""Canary ramp-up: feature flags, X%% traffic to new version, comparative metrics."""
from __future__ import annotations

import os
import random
import time
from typing import Any, Dict, List, Optional

_CANARY_METRICS: List[Dict[str, Any]] = []
_CANARY_ERRORS: Dict[str, int] = {}
_CANARY_THRESHOLD = 0.15  # disable canary if error rate > 15%


def _env_bool(name: str, default: bool) -> bool:
    v = os.getenv(name, "true" if default else "false").strip().lower()
    return v in ("1", "true", "yes", "y", "on")


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.getenv(name, str(default)) or default)
    except (TypeError, ValueError):
        return default


def _env_list(name: str, default: List[str]) -> List[str]:
    raw = os.getenv(name, "")
    if not raw:
        return default
    return [x.strip() for x in raw.split(",") if x.strip()]


def is_canary_enabled() -> bool:
    if not _env_bool("CANARY_ENABLED", False):
        return False
    try:
        from .switcher import get_deploy_state
        state = get_deploy_state()
        if state.get("canary_disabled_fallback"):
            return False
    except Exception:
        pass
    return True


def get_canary_percentage() -> float:
    return max(0.0, min(1.0, _env_float("CANARY_PERCENTAGE", 0.2)))


def get_canary_features() -> List[str]:
    return _env_list("CANARY_FEATURES", ["vision", "web", "optimizer"])


def set_canary_disabled_fallback(disabled: bool) -> None:
    """Set canary_disabled_fallback in deploy_state (fallback when error rate > threshold)."""
    try:
        from .switcher import get_deploy_state, persist_state
        state = get_deploy_state()
        state["canary_disabled_fallback"] = disabled
        persist_state(state)
    except Exception:
        pass


def use_canary_version(feature: str) -> bool:
    """True if this call should use canary (new) version for feature. Thread-safe enough for single process."""
    if not is_canary_enabled():
        return False
    if feature not in get_canary_features():
        return False
    return random.random() < get_canary_percentage()


def record_canary_call(feature: str, used_canary: bool, ok: bool, latency_ms: float) -> None:
    """Record one call for comparative metrics."""
    _CANARY_METRICS.append({
        "ts": time.time(),
        "feature": feature,
        "canary": used_canary,
        "ok": ok,
        "latency_ms": latency_ms,
    })
    if not ok and used_canary:
        _CANARY_ERRORS[feature] = _CANARY_ERRORS.get(feature, 0) + 1
    # Keep last 500
    if len(_CANARY_METRICS) > 500:
        _CANARY_METRICS[:] = _CANARY_METRICS[-500:]


def set_canary_disabled_fallback(disabled: bool) -> None:
    """Set canary_disabled_fallback in deploy_state (fallback when error rate > threshold)."""
    try:
        from .switcher import get_deploy_state, persist_state
        state = get_deploy_state()
        state["canary_disabled_fallback"] = disabled
        persist_state(state)
    except Exception:
        pass


def get_canary_stats() -> Dict[str, Any]:
    """Aggregate canary vs stable: count, error_rate, avg_latency_ms."""
    canary_enabled = is_canary_enabled()
    pct = get_canary_percentage()
    features = get_canary_features()
    recent = [m for m in _CANARY_METRICS if m["ts"] > time.time() - 3600]
    canary_calls = [m for m in recent if m["canary"]]
    stable_calls = [m for m in recent if not m["canary"]]
    canary_errors = sum(1 for m in canary_calls if not m["ok"])
    canary_total = len(canary_calls)
    error_rate = (canary_errors / canary_total) if canary_total else 0.0
    return {
        "enabled": canary_enabled,
        "percentage": pct,
        "features": features,
        "canary_calls_1h": canary_total,
        "stable_calls_1h": len(stable_calls),
        "canary_error_rate": round(error_rate, 4),
        "canary_avg_latency_ms": round(sum(m["latency_ms"] for m in canary_calls) / canary_total, 2) if canary_total else None,
        "stable_avg_latency_ms": round(sum(m["latency_ms"] for m in stable_calls) / len(stable_calls), 2) if stable_calls else None,
        "disabled_due_to_errors": canary_enabled and error_rate > _CANARY_THRESHOLD,
    }


def should_disable_canary() -> bool:
    """True if canary error rate exceeds threshold (fallback)."""
    if not is_canary_enabled():
        return False
    stats = get_canary_stats()
    return bool(stats.get("canary_error_rate", 0) > _CANARY_THRESHOLD)

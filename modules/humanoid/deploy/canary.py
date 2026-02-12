"""Canary ramp-up: feature flags, X%% traffic to new version, comparative metrics."""
from __future__ import annotations

import os
import random
import time
from typing import Any, Dict, List, Optional

_CANARY_METRICS: List[Dict[str, Any]] = []
_CANARY_ERRORS: Dict[str, int] = {}
# Runtime overrides (POST /canary/config); None = use env
_runtime_enabled: Optional[bool] = None
_runtime_percentage: Optional[float] = None
_runtime_features: Optional[List[str]] = None


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


def _error_rate_threshold() -> float:
    return max(0.0, min(1.0, _env_float("CANARY_ERROR_RATE_THRESHOLD", 0.25)))


def _latency_threshold_ms() -> float:
    return max(0.0, _env_float("CANARY_LATENCY_THRESHOLD_MS", 4000))


def is_canary_enabled() -> bool:
    if _runtime_enabled is not None:
        if not _runtime_enabled:
            return False
    elif not _env_bool("CANARY_ENABLED", False):
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
    if _runtime_percentage is not None:
        return max(0.0, min(1.0, _runtime_percentage))
    return max(0.0, min(1.0, _env_float("CANARY_PERCENTAGE", 0.2)))


def get_canary_features() -> List[str]:
    if _runtime_features is not None:
        return list(_runtime_features)
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


def should_use_canary(feature_name: str, request_id: Optional[str] = None) -> bool:
    """Deterministic canary: request_id (or random) drives percentage. Use for feature gating (vision, web, optimizer)."""
    if not is_canary_enabled():
        return False
    if feature_name not in get_canary_features():
        return False
    pct = get_canary_percentage()
    if pct <= 0:
        return False
    if pct >= 1.0:
        return True
    if request_id is not None:
        h = hash(request_id) % 10000
        return (h / 10000.0) < pct
    return random.random() < pct


def use_canary_version(feature: str) -> bool:
    """True if this call should use canary (new) version for feature. Delegates to should_use_canary(feature, None)."""
    return should_use_canary(feature, None)


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
    try:
        from modules.humanoid.metrics import get_metrics_store
        store = get_metrics_store()
        store.inc("canary_requests")
        store.record_latency(f"canary_{feature}", latency_ms)
    except Exception:
        pass
    if len(_CANARY_METRICS) > 500:
        _CANARY_METRICS[:] = _CANARY_METRICS[-500:]

    # Auto-disable if thresholds exceeded
    try:
        _maybe_auto_disable_canary()
    except Exception:
        pass


def _maybe_auto_disable_canary() -> None:
    """If canary error rate or latency exceeds env thresholds, set canary_disabled_fallback and audit."""
    stats = get_canary_stats()
    err = stats.get("canary_error_rate") or 0
    lat = stats.get("canary_avg_latency_ms")
    err_ok = err <= _error_rate_threshold()
    lat_ok = lat is None or lat <= _latency_threshold_ms()
    if not err_ok or not lat_ok:
        set_canary_disabled_fallback(True)
        try:
            from modules.humanoid.audit import get_audit_logger
            get_audit_logger().log_event("deploy", "canary", "auto_disable", False, 0, "threshold exceeded", {"error_rate": err, "latency_ms": lat}, None)
        except Exception:
            pass


def set_canary_config(enabled: Optional[bool] = None, percentage: Optional[float] = None, features: Optional[List[str]] = None) -> None:
    """Runtime override for canary (POST /canary/config)."""
    global _runtime_enabled, _runtime_percentage, _runtime_features
    if enabled is not None:
        _runtime_enabled = bool(enabled)
    if percentage is not None:
        _runtime_percentage = max(0.0, min(1.0, float(percentage)))
    if features is not None:
        _runtime_features = list(features)


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
    canary_avg = round(sum(m["latency_ms"] for m in canary_calls) / canary_total, 2) if canary_total else None
    return {
        "enabled": canary_enabled,
        "percentage": pct,
        "features": features,
        "canary_calls_1h": canary_total,
        "stable_calls_1h": len(stable_calls),
        "canary_error_rate": round(error_rate, 4),
        "canary_avg_latency_ms": canary_avg,
        "stable_avg_latency_ms": round(sum(m["latency_ms"] for m in stable_calls) / len(stable_calls), 2) if stable_calls else None,
        "disabled_due_to_errors": canary_enabled and error_rate > _error_rate_threshold(),
        "latency_threshold_ms": _latency_threshold_ms(),
        "error_rate_threshold": _error_rate_threshold(),
    }


def should_disable_canary() -> bool:
    """True if canary error rate or latency exceeds threshold (fallback)."""
    if not is_canary_enabled():
        return False
    stats = get_canary_stats()
    return bool(
        (stats.get("canary_error_rate") or 0) > _error_rate_threshold()
        or ((stats.get("canary_avg_latency_ms") or 0) > _latency_threshold_ms())
    )


def get_canary_report(hours: float = 24.0) -> Dict[str, Any]:
    """Report: canary vs stable comparison over the last N hours (counts, error rate, avg latency)."""
    stats = get_canary_stats()
    cutoff = time.time() - (hours * 3600)
    recent = [m for m in _CANARY_METRICS if m["ts"] > cutoff]
    canary_calls = [m for m in recent if m["canary"]]
    stable_calls = [m for m in recent if not m["canary"]]
    canary_errors = sum(1 for m in canary_calls if not m["ok"])
    canary_n = len(canary_calls)
    stable_n = len(stable_calls)
    return {
        "window_hours": hours,
        "canary_calls": canary_n,
        "stable_calls": stable_n,
        "canary_errors": canary_errors,
        "canary_error_rate": round(canary_errors / canary_n, 4) if canary_n else 0.0,
        "canary_avg_latency_ms": round(sum(m["latency_ms"] for m in canary_calls) / canary_n, 2) if canary_n else None,
        "stable_avg_latency_ms": round(sum(m["latency_ms"] for m in stable_calls) / stable_n, 2) if stable_n else None,
        **{k: v for k, v in stats.items() if k in ("enabled", "percentage", "features", "disabled_due_to_errors")},
    }

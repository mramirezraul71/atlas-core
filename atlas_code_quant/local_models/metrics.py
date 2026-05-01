"""Prometheus metrics for local model usage and degradation."""
from __future__ import annotations

import re
from typing import Any

try:
    from prometheus_client import Counter, Histogram

    _PROM_OK = True
except Exception:  # pragma: no cover
    Counter = Histogram = None  # type: ignore[assignment,misc]
    _PROM_OK = False

_REQUESTS_TOTAL = None
_REQUEST_DURATION_MS = None


def _normalize_label(value: Any, *, default: str, max_len: int = 80) -> str:
    raw = str(value or "").strip()
    if not raw:
        return default
    raw = raw[:max_len]
    # Keep labels bounded and low-cardinality-ish.
    return re.sub(r"[^a-zA-Z0-9_.:-]", "_", raw)


def _normalize_fallback_reason(reason: Any) -> str:
    raw = str(reason or "").strip().lower()
    if not raw:
        return "none"
    # Collapse detailed exception suffixes into stable categories.
    base = raw.split(":", 1)[0]
    return _normalize_label(base, default="unknown_reason", max_len=60)


def _init_metrics() -> None:
    global _REQUESTS_TOTAL, _REQUEST_DURATION_MS
    if not _PROM_OK:
        return
    if _REQUESTS_TOTAL is None:
        _REQUESTS_TOTAL = Counter(
            "atlas_local_model_requests_total",
            "Local model calls by role/model/fallback/outcome.",
            [
                "provider_role",
                "requested_model",
                "used_fallback",
                "fallback_reason",
                "outcome",
            ],
        )
    if _REQUEST_DURATION_MS is None:
        _REQUEST_DURATION_MS = Histogram(
            "atlas_local_model_request_duration_ms",
            "Local model request duration in milliseconds.",
            ["provider_role", "requested_model", "used_fallback"],
            buckets=(5, 10, 25, 50, 100, 200, 500, 1000, 2500, 5000, 10000, float("inf")),
        )


def record_local_model_result(
    *,
    provider_role: str,
    requested_model: str,
    used_fallback: bool,
    fallback_reason: str | None,
    outcome: str,
    duration_ms: float | None = None,
) -> None:
    """Record aggregated local model metrics.

    This function is intentionally no-op when prometheus_client is unavailable.
    """
    _init_metrics()
    if not _PROM_OK:
        return
    role = _normalize_label(provider_role, default="unknown_role", max_len=40)
    model = _normalize_label(requested_model, default="unknown_model", max_len=80)
    reason = _normalize_fallback_reason(fallback_reason if used_fallback else "none")
    fallback_label = "true" if bool(used_fallback) else "false"
    outcome_label = _normalize_label(outcome, default="unknown", max_len=20)
    try:
        _REQUESTS_TOTAL.labels(
            provider_role=role,
            requested_model=model,
            used_fallback=fallback_label,
            fallback_reason=reason,
            outcome=outcome_label,
        ).inc()
        if duration_ms is not None:
            _REQUEST_DURATION_MS.labels(
                provider_role=role,
                requested_model=model,
                used_fallback=fallback_label,
            ).observe(max(0.0, float(duration_ms)))
    except Exception:
        return

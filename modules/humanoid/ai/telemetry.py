"""Metrics per provider/model: latency, error, success, cost_est."""
from __future__ import annotations

import time
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Dict, List

@dataclass
class RequestMetric:
    provider_id: str
    model_key: str
    latency_ms: float
    success: bool
    cost_estimate_usd: float = 0.0
    quality_flag: str = ""  # empty|empty_response|invalid_json|suspected_hallucination


_metrics: List[RequestMetric] = []
_max_metrics = 500


def record(provider_id: str, model_key: str, latency_ms: float, success: bool, cost_estimate_usd: float = 0.0, quality_flag: str = "") -> None:
    global _metrics
    _metrics.append(
        RequestMetric(
            provider_id=provider_id,
            model_key=model_key,
            latency_ms=latency_ms,
            success=success,
            cost_estimate_usd=cost_estimate_usd,
            quality_flag=quality_flag or "",
        )
    )
    if len(_metrics) > _max_metrics:
        _metrics[:] = _metrics[-_max_metrics:]


def aggregate_by_provider() -> Dict[str, dict]:
    by_provider: Dict[str, dict] = defaultdict(lambda: {"count": 0, "success": 0, "errors": 0, "latency_sum_ms": 0.0, "cost_sum_usd": 0.0})
    for m in _metrics:
        k = f"{m.provider_id}:{m.model_key}"
        by_provider[k]["count"] += 1
        if m.success:
            by_provider[k]["success"] += 1
        else:
            by_provider[k]["errors"] += 1
        by_provider[k]["latency_sum_ms"] += m.latency_ms
        by_provider[k]["cost_sum_usd"] += m.cost_estimate_usd
    out = {}
    for k, v in by_provider.items():
        n = v["count"]
        out[k] = {
            "requests": n,
            "success_rate": v["success"] / n if n else 0,
            "error_rate": v["errors"] / n if n else 0,
            "latency_avg_ms": v["latency_sum_ms"] / n if n else 0,
            "cost_estimate_usd": v["cost_sum_usd"],
        }
    return out


def quality_insufficient_count(provider_id: str, model_key: str) -> int:
    return sum(1 for m in _metrics if m.provider_id == provider_id and m.model_key == model_key and m.quality_flag)


def get_recent(n: int = 50) -> List[dict]:
    return [
        {
            "provider_id": m.provider_id,
            "model_key": m.model_key,
            "latency_ms": m.latency_ms,
            "success": m.success,
            "cost_estimate_usd": m.cost_estimate_usd,
            "quality_flag": m.quality_flag or None,
        }
        for m in _metrics[-n:]
    ]

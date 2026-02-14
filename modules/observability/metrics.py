"""
Métricas Prometheus para ATLAS: requests, latencia, salud autónoma.
"""
from __future__ import annotations

import time
from typing import Dict, Any

from prometheus_client import Counter, Histogram, Gauge, generate_latest, REGISTRY

# --- Métricas ---
request_count = Counter(
    "atlas_http_requests_total",
    "Total HTTP requests",
    ["method", "endpoint", "status"],
)

request_duration = Histogram(
    "atlas_http_request_duration_seconds",
    "HTTP request duration",
    ["method", "endpoint"],
)

active_requests = Gauge(
    "atlas_http_requests_active",
    "Number of active HTTP requests",
)

memory_usage = Gauge(
    "atlas_memory_usage_bytes",
    "Memory usage in bytes",
)

autonomous_health_score = Gauge(
    "atlas_autonomous_health_score",
    "Autonomous health score (0-100)",
)

meta_learning_tasks = Gauge(
    "atlas_meta_learning_tasks_total",
    "Number of meta-learning tasks",
)

# Resumen en memoria para /api/observability/metrics (sin depender de APIs internas de prometheus_client)
_summary: Dict[str, Any] = {
    "total_requests": 0,
    "active_requests": 0,
    "memory_mb": 0.0,
    "health_score": 0.0,
    "meta_learning_tasks": 0.0,
}


def record_request(method: str, endpoint: str, status: int, duration_sec: float) -> None:
    request_count.labels(method=method, endpoint=endpoint, status=str(status)).inc()
    request_duration.labels(method=method, endpoint=endpoint).observe(duration_sec)
    _summary["total_requests"] = _summary.get("total_requests", 0) + 1


def inc_active() -> None:
    active_requests.inc()
    _summary["active_requests"] = _summary.get("active_requests", 0) + 1


def dec_active() -> None:
    active_requests.dec()
    _summary["active_requests"] = max(0, _summary.get("active_requests", 0) - 1)


def update_system_metrics() -> None:
    """Actualiza métricas de sistema (memoria, etc.). Llamar periódicamente."""
    try:
        import psutil
        rss = psutil.Process().memory_info().rss
        memory_usage.set(rss)
        _summary["memory_mb"] = round(rss / 1024 / 1024, 2)
    except Exception:
        pass


def set_health_score(score: float) -> None:
    autonomous_health_score.set(score)
    _summary["health_score"] = score


def set_meta_learning_tasks(count: float) -> None:
    meta_learning_tasks.set(count)
    _summary["meta_learning_tasks"] = count


class MetricsCollector:
    @staticmethod
    def get_prometheus_text() -> str:
        return generate_latest(REGISTRY).decode("utf-8")

    @staticmethod
    def get_metrics_summary() -> Dict[str, Any]:
        update_system_metrics()
        return {
            "total_requests": _summary.get("total_requests", 0),
            "active_requests": _summary.get("active_requests", 0),
            "memory_mb": _summary.get("memory_mb", 0.0),
            "health_score": _summary.get("health_score", 0.0),
            "meta_learning_tasks": _summary.get("meta_learning_tasks", 0.0),
        }

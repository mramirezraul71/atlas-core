"""
RouteOptimizer - Optimiza routing de Neural Router basado en métricas.
Registra uso por modelo, analiza performance, sugiere modelo por tipo de query.
"""
from __future__ import annotations

import json
import logging
import urllib.request
import urllib.error
from typing import Any, Dict, List

logger = logging.getLogger(__name__)


class RouteOptimizer:
    """Optimiza routing de Neural Router basado en métricas."""

    def __init__(self, nexus_url: str = "http://localhost:8000"):
        self.nexus_url = nexus_url.rstrip("/")
        self.model_stats: Dict[str, Dict[str, float]] = {}

    def record_model_usage(self, model: str, success: bool, latency_ms: float) -> None:
        """Registrar uso de un modelo."""
        if model not in self.model_stats:
            self.model_stats[model] = {"success": 0, "failures": 0, "total_latency": 0.0, "count": 0}
        self.model_stats[model]["count"] += 1
        self.model_stats[model]["total_latency"] += latency_ms
        if success:
            self.model_stats[model]["success"] += 1
        else:
            self.model_stats[model]["failures"] += 1

    def analyze_model_performance(self, model_name: str, time_range: int = 7) -> Dict[str, Any] | None:
        """Analizar performance de un modelo (time_range en días, ignorado por ahora; stats en memoria)."""
        if model_name not in self.model_stats:
            return None
        stats = self.model_stats[model_name]
        total = stats["count"]
        return {
            "model": model_name,
            "total_requests": total,
            "success_rate": (stats["success"] / total * 100) if total > 0 else 0,
            "avg_latency_ms": (stats["total_latency"] / total) if total > 0 else 0,
            "failures": int(stats["failures"]),
        }

    def suggest_model_for_query(self, query_type: str) -> str:
        """Sugerir mejor modelo para un tipo de query."""
        try:
            req = urllib.request.Request(
                f"{self.nexus_url}/status",
                headers={"Accept": "application/json"},
                method="GET",
            )
            with urllib.request.urlopen(req, timeout=5) as r:
                data = json.loads(r.read().decode())
            available_models = data.get("neural_router", {}).get("models", [])
            if isinstance(available_models, dict):
                available_models = list(available_models.keys()) if available_models else []
        except Exception as e:
            logger.debug("RouteOptimizer: no NEXUS status: %s", e)
            available_models = []
        if not available_models:
            return "llama3.2"
        if query_type == "code":
            return "deepseek-coder" if "deepseek-coder" in available_models else available_models[0]
        if query_type == "reasoning":
            return "deepseek-r1" if "deepseek-r1" in available_models else available_models[0]
        return available_models[0]

    def optimize_routing_rules(self) -> Dict[str, Dict[str, Any]]:
        """Actualizar reglas de routing basado en performance."""
        optimizations: Dict[str, Dict[str, Any]] = {}
        for model_name in self.model_stats:
            perf = self.analyze_model_performance(model_name)
            if perf is None:
                continue
            if perf["success_rate"] < 80:
                optimizations[model_name] = {
                    "recommendation": "downgrade_priority",
                    "reason": f"Success rate bajo: {perf['success_rate']:.1f}%",
                }
            if perf["avg_latency_ms"] > 5000:
                optimizations[model_name] = {
                    "recommendation": "use_for_batch_only",
                    "reason": f"Latencia alta: {perf['avg_latency_ms']:.0f}ms",
                }
        return optimizations

    def get_routing_stats(self) -> dict[str, Any]:
        """Estadísticas de routing para dashboards."""
        return {
            "models": dict(self.model_stats),
            "optimizations": self.optimize_routing_rules(),
        }

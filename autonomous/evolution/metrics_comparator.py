"""
MetricsComparator - Compara métricas before/after actualización.
Decide si hacer rollback según umbrales de degradación.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)


def _load_config() -> dict:
    cfg_path = Path(__file__).resolve().parent.parent.parent / "config" / "autonomous.yaml"
    if not cfg_path.exists():
        return {}
    try:
        import yaml
        with open(cfg_path, encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    except Exception:
        return {}


@dataclass
class ComparisonReport:
    degraded: list[str]
    improved: list[str]
    stable: list[str]
    details: dict[str, Any] = field(default_factory=dict)


class MetricsComparator:
    """
    Captura baseline pre-update y post-update; compara response times, error rates, resource usage.
    """

    def __init__(self, config: dict | None = None):
        self._config = config or _load_config().get("evolution", {})
        self._rollback_cfg = self._config.get("auto_rollback", {})
        self._degradation_threshold_pct = float(self._rollback_cfg.get("degradation_threshold", 15))
        self._baseline: dict[str, float] = {}
        self._post: dict[str, float] = {}

    def capture_baseline(self) -> dict[str, float]:
        """Captura métricas actuales como baseline. Retorna dict de métricas."""
        try:
            from autonomous.health_monitor import HealthAggregator
            agg = HealthAggregator()
            report = agg.get_global_health()
            self._baseline = {
                "health_score": report.score,
                "system_score": report.system_score,
                "services_score": report.services_score,
            }
            if report.components:
                sys_c = report.components.get("system", {})
                if isinstance(sys_c, dict):
                    self._baseline["cpu_percent"] = sys_c.get("cpu_percent", 0)
                    self._baseline["ram_percent"] = sys_c.get("ram_percent", 0)
            return dict(self._baseline)
        except Exception as e:
            logger.warning("capture_baseline: %s", e)
            self._baseline = {}
            return {}

    def capture_post_update(self) -> dict[str, float]:
        """Captura métricas tras la actualización."""
        try:
            from autonomous.health_monitor import HealthAggregator
            agg = HealthAggregator()
            report = agg.get_global_health()
            self._post = {
                "health_score": report.score,
                "system_score": report.system_score,
                "services_score": report.services_score,
            }
            if report.components:
                sys_c = report.components.get("system", {})
                if isinstance(sys_c, dict):
                    self._post["cpu_percent"] = sys_c.get("cpu_percent", 0)
                    self._post["ram_percent"] = sys_c.get("ram_percent", 0)
            return dict(self._post)
        except Exception as e:
            logger.warning("capture_post_update: %s", e)
            self._post = {}
            return {}

    def compare_metrics(self) -> ComparisonReport:
        """Compara baseline vs post; clasifica en degraded, improved, stable."""
        degraded = []
        improved = []
        stable = []
        details = {"baseline": self._baseline, "post": self._post}

        for key in set(self._baseline) | set(self._post):
            b = self._baseline.get(key)
            p = self._post.get(key)
            if b is None or p is None:
                continue
            try:
                b, p = float(b), float(p)
            except (TypeError, ValueError):
                continue
            # Para score: mayor es mejor. Para cpu/ram: menor es mejor.
            higher_is_better = "score" in key.lower()
            if higher_is_better:
                if p < b * (1 - self._degradation_threshold_pct / 100):
                    degraded.append(f"{key}: {b} -> {p} (degraded)")
                elif p > b * (1 + self._degradation_threshold_pct / 100):
                    improved.append(f"{key}: {b} -> {p} (improved)")
                else:
                    stable.append(f"{key}: {b} -> {p}")
            else:
                if p > b * (1 + self._degradation_threshold_pct / 100):
                    degraded.append(f"{key}: {b} -> {p} (worse)")
                elif p < b * (1 - self._degradation_threshold_pct / 100):
                    improved.append(f"{key}: {b} -> {p} (better)")
                else:
                    stable.append(f"{key}: {b} -> {p}")

        return ComparisonReport(degraded=degraded, improved=improved, stable=stable, details=details)

    def should_rollback(self) -> bool:
        """True si hay degradación por encima del umbral."""
        report = self.compare_metrics()
        return len(report.degraded) > 0

    def generate_comparison_chart(self) -> dict[str, Any]:
        """Datos para visualización (series before/after)."""
        return {
            "baseline": self._baseline,
            "post": self._post,
            "report": {
                "degraded": len(self.compare_metrics().degraded),
                "improved": len(self.compare_metrics().improved),
                "stable": len(self.compare_metrics().stable),
            },
        }

"""
DashboardEngine - Genera datos para dashboards: Health, Services, Neural Router, Tools, Self-Healing, Evolution.
"""
from __future__ import annotations

import logging
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


class DashboardEngine:
    """
    generate_dashboard_data(dashboard_type) → datos para UI;
    create_chart(metric, chart_type) → config de gráfico.
    """

    def __init__(self, config: dict | None = None):
        self._config = config or _load_config()

    def generate_dashboard_data(self, dashboard_type: str) -> dict[str, Any]:
        """Datos listos para consumir por el frontend."""
        data = {"type": dashboard_type, "series": [], "gauges": [], "tables": []}
        try:
            if dashboard_type == "health_overview":
                from autonomous.health_monitor import HealthAggregator
                agg = HealthAggregator()
                report = agg.get_global_health()
                data["gauges"] = [{"name": "global_score", "value": report.score, "max": 100}]
                data["tables"] = [{"name": "components", "rows": [report.components]}]
                data["recommendations"] = report.recommendations
            elif dashboard_type == "services_status":
                from autonomous.health_monitor import ServiceHealth
                sh = ServiceHealth()
                statuses = sh.get_all_services_status()
                data["tables"] = [{"name": "services", "rows": [{"name": k, "online": v.online, "latency_ms": v.latency_ms} for k, v in statuses.items()]}]
            elif dashboard_type == "self_healing":
                from autonomous.self_healing import HealingOrchestrator
                ho = HealingOrchestrator()
                data["gauges"] = [{"name": "stats", "value": ho.get_healing_stats()}]
            elif dashboard_type == "evolution":
                from autonomous.evolution import EvolutionOrchestratorV2
                ev = EvolutionOrchestratorV2()
                data["gauges"] = [{"name": "status", "value": ev.get_status()}]
            else:
                data["error"] = f"Unknown dashboard type: {dashboard_type}"
        except Exception as e:
            logger.exception("generate_dashboard_data: %s", e)
            data["error"] = str(e)
        return data

    def create_chart(self, metric: str, chart_type: str = "line") -> dict[str, Any]:
        """Configuración de gráfico para una métrica."""
        return {"metric": metric, "chart_type": chart_type, "title": metric.replace("_", " ").title()}

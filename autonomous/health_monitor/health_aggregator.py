"""
HealthAggregator - Centraliza SystemMetrics, ServiceHealth y AnomalyDetector.
Calcula salud global, persiste en SQLite para trending, genera recomendaciones.
"""
from __future__ import annotations

import asyncio
import json
import logging
import sqlite3
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from .system_metrics import SystemMetrics
from .service_health import ServiceHealth, ServiceStatus
from .anomaly_detector import AnomalyDetector, AnomalyReport

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
class GlobalHealth:
    """Salud global del sistema."""
    score: float
    components: dict[str, Any]
    anomalies: list[AnomalyReport]
    recommendations: list[str]
    timestamp: float = field(default_factory=time.time)
    system_score: float = 0.0
    services_score: float = 0.0


class HealthAggregator:
    """
    Combina métricas de sistema, estado de servicios y anomalías.
    Opcionalmente persiste en SQLite para histórico.
    """

    def __init__(self, config: dict | None = None):
        self._config = config or _load_config().get("health_monitor", {})
        self._system = SystemMetrics(self._config)
        self._services = ServiceHealth(self._config)
        self._anomaly = AnomalyDetector(self._config)
        self._interval_sec = int(self._config.get("check_interval_seconds", 10))
        self._db_path: Path | None = None
        self._running = False
        self._task: asyncio.Task | None = None
        self._history: list[dict] = []
        self._max_history = 500

        # Persistencia: logs/autonomous_health.sqlite
        base = Path(__file__).resolve().parent.parent.parent
        self._db_path = base / "logs" / "autonomous_health.sqlite"
        self._db_path.parent.mkdir(parents=True, exist_ok=True)
        self._init_db()

    def _init_db(self) -> None:
        if not self._db_path:
            return
        try:
            with sqlite3.connect(str(self._db_path)) as conn:
                conn.execute("""
                    CREATE TABLE IF NOT EXISTS health_reports (
                        id INTEGER PRIMARY KEY AUTOINCREMENT,
                        ts REAL NOT NULL,
                        score REAL NOT NULL,
                        system_score REAL,
                        services_score REAL,
                        payload TEXT
                    )
                """)
                conn.execute("CREATE INDEX IF NOT EXISTS idx_health_ts ON health_reports(ts)")
        except Exception as e:
            logger.warning("Health DB init: %s", e)

    def get_global_health(self) -> GlobalHealth:
        """Calcula y devuelve el estado de salud global."""
        sys_metrics = self._system.get_current_metrics()
        sys_score = self._system.get_health_score()
        service_statuses = self._services.get_all_services_status()
        service_scores = [self._services.calculate_service_score(s) for s in service_statuses.values()]
        services_score = sum(service_scores) / len(service_scores) if service_scores else 0.0

        raw = {
            "cpu_percent": sys_metrics.cpu_percent,
            "ram_percent": sys_metrics.ram_percent,
            "disk_usage_percent": sys_metrics.disk_usage_percent,
        }
        self._history.append(raw)
        if len(self._history) > self._max_history:
            self._history.pop(0)

        if len(self._history) >= 20 and self._anomaly._enabled:
            if not self._anomaly._trained:
                self._anomaly.train(self._history[:-1])
            anomaly_report = self._anomaly.detect(raw)
        else:
            anomaly_report = AnomalyReport(
                detected=False,
                severity="leve",
                metrics_affected=[],
                score=0.0,
            )

        global_score = (sys_score * 0.5 + services_score * 0.5)
        if anomaly_report.detected:
            global_score = max(0, global_score - 10 * (3 if anomaly_report.severity == "severa" else 2 if anomaly_report.severity == "moderada" else 1))

        components = {
            "system": {
                "score": sys_score,
                "cpu_percent": sys_metrics.cpu_percent,
                "ram_percent": sys_metrics.ram_percent,
                "disk_percent": sys_metrics.disk_usage_percent,
            },
            "services": {
                name: {"online": s.online, "latency_ms": s.latency_ms, "score": self._services.calculate_service_score(s)}
                for name, s in service_statuses.items()
            },
        }

        recommendations = []
        if sys_metrics.cpu_percent >= 80:
            recommendations.append("CPU alto: reducir carga o escalar")
        if sys_metrics.ram_percent >= 85:
            recommendations.append("RAM alto: revisar fugas o aumentar memoria")
        if sys_metrics.disk_usage_percent >= 90:
            recommendations.append("Disco casi lleno: limpiar logs o ampliar almacenamiento")
        for name, s in service_statuses.items():
            if not s.online:
                recommendations.append(f"Servicio {name} caído: revisar proceso y puerto")
        bottlenecks = self._services.get_bottlenecks()
        for b in bottlenecks:
            if b not in recommendations:
                recommendations.append(b)

        return GlobalHealth(
            score=round(global_score, 1),
            components=components,
            anomalies=[anomaly_report] if anomaly_report.detected else [],
            recommendations=recommendations,
            system_score=sys_score,
            services_score=round(services_score, 1),
        )

    def save_to_db(self, health_report: GlobalHealth) -> None:
        """Persiste el reporte en SQLite para trending."""
        if not self._db_path:
            return
        try:
            payload = json.dumps({
                "components": health_report.components,
                "recommendations": health_report.recommendations,
                "anomalies": [{"severity": a.severity, "metrics_affected": a.metrics_affected} for a in health_report.anomalies],
            }, default=str)
            with sqlite3.connect(str(self._db_path)) as conn:
                conn.execute(
                    "INSERT INTO health_reports (ts, score, system_score, services_score, payload) VALUES (?,?,?,?,?)",
                    (health_report.timestamp, health_report.score, health_report.system_score, health_report.services_score, payload),
                )
        except Exception as e:
            logger.debug("Save health report: %s", e)

    def get_historical_trend(self, days: int = 7) -> list[dict]:
        """Devuelve reportes históricos (últimos N días)."""
        if not self._db_path or not self._db_path.exists():
            return []
        cutoff = time.time() - days * 86400
        try:
            with sqlite3.connect(str(self._db_path)) as conn:
                conn.row_factory = sqlite3.Row
                cur = conn.execute(
                    "SELECT ts, score, system_score, services_score, payload FROM health_reports WHERE ts >= ? ORDER BY ts",
                    (cutoff,),
                )
                rows = cur.fetchall()
                return [
                    {
                        "ts": r["ts"],
                        "score": r["score"],
                        "system_score": r["system_score"],
                        "services_score": r["services_score"],
                        "payload": json.loads(r["payload"]) if r["payload"] else {},
                    }
                    for r in rows
                ]
        except Exception as e:
            logger.debug("Historical trend: %s", e)
            return []

    async def _monitoring_loop(self) -> None:
        """Loop que ejecuta get_global_health y save cada interval_sec."""
        while self._running:
            try:
                report = self.get_global_health()
                self.save_to_db(report)
            except Exception as e:
                logger.exception("Health monitoring loop: %s", e)
            await asyncio.sleep(self._interval_sec)

    def start_monitoring(self) -> None:
        """Inicia el loop de monitoreo en background (asyncio)."""
        if self._running:
            return
        self._running = True
        try:
            loop = asyncio.get_running_loop()
            self._task = loop.create_task(self._monitoring_loop())
            logger.info("Health monitoring started (interval=%ss)", self._interval_sec)
        except RuntimeError:
            # No event loop (e.g. script directo)
            self._running = False
            logger.warning("Health monitoring: no asyncio loop; use get_global_health() manually")
        except Exception as e:
            self._running = False
            logger.exception("Health monitoring start: %s", e)

    def stop_monitoring(self) -> None:
        """Detiene el loop de monitoreo."""
        self._running = False
        if self._task and not self._task.done():
            self._task.cancel()
        self._task = None

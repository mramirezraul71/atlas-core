"""
MetricsAggregator - Centraliza métricas de Health, Healing, Evolution, Neural, Tools, ANS.
Estructura: timestamp, source, metric_name, value, tags. Persiste en SQLite.
"""
from __future__ import annotations

import json
import logging
import sqlite3
import time
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


class MetricsAggregator:
    """
    collect_metric(source, name, value, tags) registra;
    get_metrics(source, time_range, filters) consulta;
    aggregate(name, operation, window) para sum/avg/max/min.
    """

    def __init__(self, config: dict | None = None):
        self._config = config or _load_config().get("telemetry", {})
        self._metrics_cfg = self._config.get("metrics", {})
        self._retention_days = int(self._metrics_cfg.get("retention_days", 30))
        base = Path(__file__).resolve().parent.parent.parent
        db = self._config.get("timeseries_db", "logs/autonomous_metrics.sqlite")
        self._db_path = base / db if isinstance(db, str) else base / "logs" / "autonomous_metrics.sqlite"
        self._db_path.parent.mkdir(parents=True, exist_ok=True)
        self._init_db()

    def _init_db(self) -> None:
        try:
            with sqlite3.connect(str(self._db_path)) as conn:
                conn.execute("""
                    CREATE TABLE IF NOT EXISTS metrics (
                        id INTEGER PRIMARY KEY AUTOINCREMENT,
                        ts REAL NOT NULL,
                        source TEXT NOT NULL,
                        metric_name TEXT NOT NULL,
                        value REAL NOT NULL,
                        tags TEXT,
                        metadata TEXT
                    )
                """)
                conn.execute("CREATE INDEX IF NOT EXISTS idx_metrics_ts ON metrics(ts)")
                conn.execute("CREATE INDEX IF NOT EXISTS idx_metrics_source_name ON metrics(source, metric_name)")
        except Exception as e:
            logger.warning("MetricsAggregator init: %s", e)

    def collect_metric(
        self,
        source: str,
        metric_name: str,
        value: float,
        tags: dict[str, str] | None = None,
        metadata: dict | None = None,
    ) -> None:
        """Registra una métrica."""
        try:
            with sqlite3.connect(str(self._db_path)) as conn:
                conn.execute(
                    "INSERT INTO metrics (ts, source, metric_name, value, tags, metadata) VALUES (?,?,?,?,?,?)",
                    (
                        time.time(),
                        source[:64],
                        metric_name[:128],
                        float(value),
                        json.dumps(tags or {}),
                        json.dumps(metadata or {}, default=str),
                    ),
                )
        except Exception as e:
            logger.debug("collect_metric: %s", e)

    def get_metrics(
        self,
        source: str | None = None,
        time_range_sec: float | None = None,
        filters: dict[str, Any] | None = None,
        limit: int = 1000,
    ) -> list[dict]:
        """Consulta métricas. time_range_sec = últimos N segundos."""
        cutoff = time.time() - (time_range_sec or 86400)
        try:
            with sqlite3.connect(str(self._db_path)) as conn:
                conn.row_factory = sqlite3.Row
                if source:
                    cur = conn.execute(
                        "SELECT ts, source, metric_name, value, tags, metadata FROM metrics WHERE ts >= ? AND source = ? ORDER BY ts DESC LIMIT ?",
                        (cutoff, source, limit),
                    )
                else:
                    cur = conn.execute(
                        "SELECT ts, source, metric_name, value, tags, metadata FROM metrics WHERE ts >= ? ORDER BY ts DESC LIMIT ?",
                        (cutoff, limit),
                    )
                rows = cur.fetchall()
                return [
                    {
                        "ts": r["ts"],
                        "source": r["source"],
                        "metric_name": r["metric_name"],
                        "value": r["value"],
                        "tags": json.loads(r["tags"]) if r["tags"] else {},
                        "metadata": json.loads(r["metadata"]) if r["metadata"] else {},
                    }
                    for r in rows
                ]
        except Exception as e:
            logger.debug("get_metrics: %s", e)
            return []

    def aggregate(
        self,
        metric_name: str,
        operation: str,
        time_window_sec: float = 3600,
        source: str | None = None,
    ) -> float | None:
        """Agregación: sum, avg, max, min sobre ventana temporal."""
        cutoff = time.time() - time_window_sec
        try:
            with sqlite3.connect(str(self._db_path)) as conn:
                if source:
                    cur = conn.execute(
                        "SELECT %s(value) FROM metrics WHERE ts >= ? AND metric_name = ? AND source = ?" % operation,
                        (cutoff, metric_name, source),
                    )
                else:
                    cur = conn.execute(
                        "SELECT %s(value) FROM metrics WHERE ts >= ? AND metric_name = ?" % operation,
                        (cutoff, metric_name),
                    )
                row = cur.fetchone()
                return float(row[0]) if row and row[0] is not None else None
        except Exception as e:
            logger.debug("aggregate: %s", e)
            return None

    def export_to_timeseries_db(self) -> bool:
        """Persistencia ya en SQLite; aquí podría exportar a InfluxDB si se configura."""
        return True

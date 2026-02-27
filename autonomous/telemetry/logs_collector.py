"""
LogsCollector - Logging estructurado (JSON): level, service, module, message, context, trace_id.
Búsqueda y agregación de errores.
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


class LogsCollector:
    """
    log(level, module, message, context) escribe en SQLite y opcionalmente en handler estándar.
    search_logs(filters, time_range) y aggregate_errors(time_range).
    """

    def __init__(self, config: dict | None = None):
        self._config = config or _load_config().get("telemetry", {})
        self._logs_cfg = self._config.get("logs", {})
        self._retention_days = int(self._logs_cfg.get("retention_days", 7))
        base = Path(__file__).resolve().parent.parent.parent
        self._db_path = base / "logs" / "autonomous_logs.sqlite"
        self._db_path.parent.mkdir(parents=True, exist_ok=True)
        self._init_db()

    def _init_db(self) -> None:
        try:
            with sqlite3.connect(str(self._db_path)) as conn:
                conn.execute("""
                    CREATE TABLE IF NOT EXISTS structured_logs (
                        id INTEGER PRIMARY KEY AUTOINCREMENT,
                        ts REAL NOT NULL,
                        level TEXT NOT NULL,
                        service TEXT,
                        module TEXT,
                        message TEXT NOT NULL,
                        context TEXT,
                        trace_id TEXT,
                        error_stack TEXT
                    )
                """)
                conn.execute("CREATE INDEX IF NOT EXISTS idx_logs_ts ON structured_logs(ts)")
                conn.execute("CREATE INDEX IF NOT EXISTS idx_logs_level ON structured_logs(level)")
        except Exception as e:
            logger.warning("LogsCollector init: %s", e)

    def log(
        self,
        level: str,
        module: str,
        message: str,
        context: dict | None = None,
        trace_id: str | None = None,
        error_stack: str | None = None,
        service: str = "autonomous",
    ) -> None:
        """Registra log estructurado."""
        try:
            with sqlite3.connect(str(self._db_path)) as conn:
                conn.execute(
                    "INSERT INTO structured_logs (ts, level, service, module, message, context, trace_id, error_stack) VALUES (?,?,?,?,?,?,?,?)",
                    (
                        time.time(),
                        level[:16],
                        service[:32],
                        module[:64],
                        message[:2000],
                        json.dumps(context or {}, default=str)[:2000],
                        (trace_id or "")[:64],
                        (error_stack or "")[:4000],
                    ),
                )
        except Exception as e:
            logger.debug("LogsCollector log: %s", e)

    def search_logs(
        self,
        filters: dict[str, Any] | None = None,
        time_range_sec: float = 86400,
        limit: int = 500,
    ) -> list[dict]:
        """Busca logs por level, service, module."""
        cutoff = time.time() - time_range_sec
        filters = filters or {}
        try:
            with sqlite3.connect(str(self._db_path)) as conn:
                conn.row_factory = sqlite3.Row
                q = "SELECT ts, level, service, module, message, context, trace_id FROM structured_logs WHERE ts >= ?"
                params: list[Any] = [cutoff]
                if filters.get("level"):
                    q += " AND level = ?"
                    params.append(filters["level"])
                if filters.get("service"):
                    q += " AND service = ?"
                    params.append(filters["service"])
                if filters.get("module"):
                    q += " AND module LIKE ?"
                    params.append(f"%{filters['module']}%")
                q += " ORDER BY ts DESC LIMIT ?"
                params.append(limit)
                cur = conn.execute(q, params)
                rows = cur.fetchall()
                return [
                    {
                        "ts": r["ts"],
                        "level": r["level"],
                        "service": r["service"],
                        "module": r["module"],
                        "message": r["message"],
                        "context": json.loads(r["context"]) if r["context"] else {},
                        "trace_id": r["trace_id"],
                    }
                    for r in rows
                ]
        except Exception as e:
            logger.debug("search_logs: %s", e)
            return []

    def aggregate_errors(self, time_range_sec: float = 3600) -> list[dict]:
        """Agrupa errores similares (por message) con conteo."""
        cutoff = time.time() - time_range_sec
        try:
            with sqlite3.connect(str(self._db_path)) as conn:
                cur = conn.execute(
                    "SELECT message, COUNT(*) as cnt FROM structured_logs WHERE ts >= ? AND level IN ('ERROR','CRITICAL') GROUP BY message ORDER BY cnt DESC LIMIT 50",
                    (cutoff,),
                )
                return [{"message": r[0], "count": r[1]} for r in cur.fetchall()]
        except Exception as e:
            logger.debug("aggregate_errors: %s", e)
            return []

    def export_logs(self, fmt: str = "json") -> str:
        """Exporta últimos logs en JSON o CSV."""
        rows = self.search_logs(time_range_sec=86400 * 7, limit=5000)
        if fmt.lower() == "csv":
            if not rows:
                return "ts,level,service,module,message\n"
            head = ",".join(rows[0].keys())
            lines = [head] + [",".join(str(v).replace(",", ";") for v in r.values() for r in rows]
            return "\n".join(lines)
        return json.dumps(rows, default=str, indent=0)

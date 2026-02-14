"""
FeedbackLoop - Registro de feedback implícito y explícito; mejora continua.
"""
from __future__ import annotations

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


class FeedbackLoop:
    def __init__(self, config: dict | None = None):
        self._config = config or _load_config().get("learning", {})
        base = Path(__file__).resolve().parent.parent.parent
        db = self._config.get("feedback_db", "logs/autonomous_learning.sqlite")
        self._db_path = base / db if isinstance(db, str) else base / "logs" / "autonomous_learning.sqlite"
        self._db_path.parent.mkdir(parents=True, exist_ok=True)
        self._init_db()

    def _init_db(self) -> None:
        try:
            with sqlite3.connect(str(self._db_path)) as conn:
                conn.execute("""
                    CREATE TABLE IF NOT EXISTS feedback (
                        id INTEGER PRIMARY KEY AUTOINCREMENT,
                        ts REAL NOT NULL,
                        context TEXT,
                        feedback_type TEXT NOT NULL,
                        value REAL NOT NULL
                    )
                """)
        except Exception as e:
            logger.warning("FeedbackLoop init: %s", e)

    def record_feedback(self, context: str, feedback_type: str, value: float) -> None:
        """Registra feedback (p. ej. satisfaction 0-1, success 0/1)."""
        try:
            with sqlite3.connect(str(self._db_path)) as conn:
                conn.execute(
                    "INSERT INTO feedback (ts, context, feedback_type, value) VALUES (?,?,?,?)",
                    (time.time(), context[:500], feedback_type[:32], float(value)),
                )
        except Exception as e:
            logger.debug("record_feedback: %s", e)

    def analyze_feedback(self, time_range_sec: float = 86400 * 7) -> dict[str, Any]:
        """Agregado por feedback_type."""
        cutoff = time.time() - time_range_sec
        try:
            with sqlite3.connect(str(self._db_path)) as conn:
                cur = conn.execute(
                    "SELECT feedback_type, AVG(value), COUNT(*) FROM feedback WHERE ts >= ? GROUP BY feedback_type",
                    (cutoff,),
                )
                return {row[0]: {"avg": row[1], "count": row[2]} for row in cur.fetchall()}
        except Exception as e:
            logger.debug("analyze_feedback: %s", e)
            return {}

    def identify_improvement_areas(self) -> list[str]:
        """Áreas con feedback bajo."""
        agg = self.analyze_feedback()
        areas = []
        for ftype, v in agg.items():
            if v.get("avg", 1) < 0.5 and v.get("count", 0) >= 5:
                areas.append(ftype)
        return areas

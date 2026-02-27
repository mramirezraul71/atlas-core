"""
FailureMemory - Memoria de fallos en SQLite.
Registra error_signature, context, recovery_used, success; sugiere recuperación para errores similares.
"""
from __future__ import annotations

import hashlib
import json
import logging
import sqlite3
import time
from dataclasses import dataclass
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
class FailureRecord:
    error_signature: str
    timestamp: float
    context: str
    recovery_used: str
    success: bool
    notes: str


class FailureMemory:
    """
    Persiste fallos en SQLite; permite consultar similares y sugerir recuperación.
    """

    def __init__(self, config: dict | None = None):
        self._config = config or _load_config().get("self_healing", {})
        base = Path(__file__).resolve().parent.parent.parent
        db_path = self._config.get("failure_memory_db", "logs/autonomous_failure_memory.sqlite")
        self._db_path = base / db_path if isinstance(db_path, str) else base / "logs" / "autonomous_failure_memory.sqlite"
        self._db_path.parent.mkdir(parents=True, exist_ok=True)
        self._init_db()

    def _init_db(self) -> None:
        try:
            with sqlite3.connect(str(self._db_path)) as conn:
                conn.execute("""
                    CREATE TABLE IF NOT EXISTS failures (
                        id INTEGER PRIMARY KEY AUTOINCREMENT,
                        error_signature TEXT NOT NULL,
                        ts REAL NOT NULL,
                        context TEXT,
                        recovery_used TEXT,
                        success INTEGER NOT NULL,
                        notes TEXT
                    )
                """)
                conn.execute("CREATE INDEX IF NOT EXISTS idx_fail_sig ON failures(error_signature)")
                conn.execute("CREATE INDEX IF NOT EXISTS idx_fail_ts ON failures(ts)")
        except Exception as e:
            logger.warning("FailureMemory init: %s", e)

    def _signature(self, error: BaseException) -> str:
        msg = (getattr(error, "message", None) or str(error))[:200]
        return hashlib.sha256(f"{type(error).__name__}:{msg}".encode()).hexdigest()[:32]

    def record_failure(
        self,
        error: BaseException,
        context: dict[str, Any] | None,
        recovery: str,
        success: bool,
        notes: str = "",
    ) -> None:
        """Registra un fallo y la recuperación usada."""
        sig = self._signature(error)
        ctx_str = json.dumps(context or {}, default=str)[:1000]
        try:
            with sqlite3.connect(str(self._db_path)) as conn:
                conn.execute(
                    "INSERT INTO failures (error_signature, ts, context, recovery_used, success, notes) VALUES (?,?,?,?,?,?)",
                    (sig, time.time(), ctx_str, recovery, 1 if success else 0, notes[:500]),
                )
        except Exception as e:
            logger.debug("record_failure: %s", e)

    def get_similar_failures(self, error: BaseException, limit: int = 10) -> list[FailureRecord]:
        """Busca fallos con la misma firma o mensaje similar."""
        sig = self._signature(error)
        msg = str(error)[:100]
        try:
            with sqlite3.connect(str(self._db_path)) as conn:
                conn.row_factory = sqlite3.Row
                cur = conn.execute(
                    "SELECT error_signature, ts, context, recovery_used, success, notes FROM failures WHERE error_signature = ? OR context LIKE ? ORDER BY ts DESC LIMIT ?",
                    (sig, f"%{msg}%", limit),
                )
                rows = cur.fetchall()
                return [
                    FailureRecord(
                        error_signature=r["error_signature"],
                        timestamp=r["ts"],
                        context=r["context"] or "",
                        recovery_used=r["recovery_used"] or "",
                        success=bool(r["success"]),
                        notes=r["notes"] or "",
                    )
                    for r in rows
                ]
        except Exception as e:
            logger.debug("get_similar_failures: %s", e)
            return []

    def suggest_recovery(self, error: BaseException) -> str | None:
        """Sugiere la estrategia que más veces funcionó para este tipo de error."""
        similar = self.get_similar_failures(error, limit=20)
        success_recoveries = [r.recovery_used for r in similar if r.success and r.recovery_used]
        if not success_recoveries:
            return None
        from collections import Counter
        return Counter(success_recoveries).most_common(1)[0][0]

    def get_recurring_errors(self, days: int = 7) -> list[tuple[str, int]]:
        """Devuelve (error_signature, count) de errores que se repiten."""
        cutoff = time.time() - days * 86400
        try:
            with sqlite3.connect(str(self._db_path)) as conn:
                cur = conn.execute(
                    "SELECT error_signature, COUNT(*) as cnt FROM failures WHERE ts >= ? GROUP BY error_signature HAVING cnt >= 2 ORDER BY cnt DESC",
                    (cutoff,),
                )
                return list(cur.fetchall())
        except Exception as e:
            logger.debug("get_recurring_errors: %s", e)
            return []

    def generate_preventive_actions(self) -> list[str]:
        """Sugiere acciones preventivas basadas en errores recurrentes."""
        recurring = self.get_recurring_errors(7)
        actions = []
        for sig, count in recurring[:5]:
            actions.append(f"Error recurrente (x{count}): revisar causa raíz para firma {sig[:8]}...")
        return actions

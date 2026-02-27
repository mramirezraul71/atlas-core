"""
Lifelog — Registro continuo estructurado de la vida de ATLAS.

Inspirado en lifelogging robotico y RoboMemory (2025).
Captura todo evento significativo como tupla:
  (timestamp, perception, action, outcome, feedback, context)

Cada entrada es inmutable y se almacena en SQLite con indices
para consulta rapida por tipo, rango temporal, exito/fallo, etc.
"""
from __future__ import annotations

import json
import os
import sqlite3
import threading
import time
import uuid
from typing import Any, Dict, List, Optional, Tuple

_DB_PATH = os.path.join(
    os.environ.get("ATLAS_DATA_DIR", os.path.join(os.path.dirname(__file__), "..", "..", "..", "logs")),
    "lifelog.sqlite",
)
_lock = threading.Lock()


def _con() -> sqlite3.Connection:
    os.makedirs(os.path.dirname(_DB_PATH), exist_ok=True)
    c = sqlite3.connect(_DB_PATH, timeout=10)
    c.row_factory = sqlite3.Row
    c.execute("PRAGMA journal_mode=WAL")
    return c


def _ensure():
    with _con() as c:
        c.executescript("""
        CREATE TABLE IF NOT EXISTS lifelog (
            id TEXT PRIMARY KEY,
            timestamp_ts REAL NOT NULL,
            event_type TEXT NOT NULL,
            source TEXT NOT NULL,

            -- Percepcion: que vio/recibio ATLAS
            perception TEXT DEFAULT '',
            perception_data TEXT DEFAULT '{}',

            -- Accion: que hizo ATLAS
            action TEXT DEFAULT '',
            action_params TEXT DEFAULT '{}',

            -- Resultado: que paso
            outcome TEXT DEFAULT '',
            success INTEGER,
            reward REAL DEFAULT 0.0,
            duration_ms REAL DEFAULT 0.0,

            -- Retroalimentacion humana (si la hubo)
            human_feedback TEXT DEFAULT '',
            feedback_sentiment REAL,

            -- Contexto
            context TEXT DEFAULT '{}',
            tags TEXT DEFAULT '[]',
            importance REAL DEFAULT 0.5,

            -- Para agrupacion temporal
            session_id TEXT,
            episode_id TEXT,

            created_at TEXT DEFAULT (datetime('now'))
        );
        CREATE INDEX IF NOT EXISTS idx_lifelog_ts ON lifelog(timestamp_ts);
        CREATE INDEX IF NOT EXISTS idx_lifelog_type ON lifelog(event_type);
        CREATE INDEX IF NOT EXISTS idx_lifelog_source ON lifelog(source);
        CREATE INDEX IF NOT EXISTS idx_lifelog_session ON lifelog(session_id);
        CREATE INDEX IF NOT EXISTS idx_lifelog_episode ON lifelog(episode_id);
        CREATE INDEX IF NOT EXISTS idx_lifelog_success ON lifelog(success);

        CREATE TABLE IF NOT EXISTS lifelog_sessions (
            id TEXT PRIMARY KEY,
            start_ts REAL NOT NULL,
            end_ts REAL,
            event_count INTEGER DEFAULT 0,
            summary TEXT DEFAULT '',
            created_at TEXT DEFAULT (datetime('now'))
        );

        CREATE TABLE IF NOT EXISTS lifelog_fts (
            content TEXT
        );
        """)


_ensure()


class LifeLog:
    """Registro continuo de la vida operativa de ATLAS."""

    def __init__(self):
        self._current_session: Optional[str] = None
        self._session_count: int = 0
        self._start_session()

    def _start_session(self) -> str:
        sid = f"sess_{uuid.uuid4().hex[:8]}"
        self._current_session = sid
        self._session_count = 0
        with _lock:
            with _con() as c:
                c.execute("""
                    INSERT INTO lifelog_sessions (id, start_ts) VALUES (?, ?)
                """, (sid, time.time()))
        return sid

    def end_session(self, summary: str = ""):
        if self._current_session:
            with _lock:
                with _con() as c:
                    c.execute("""
                        UPDATE lifelog_sessions SET end_ts = ?, event_count = ?, summary = ?
                        WHERE id = ?
                    """, (time.time(), self._session_count, summary, self._current_session))
            self._current_session = None

    # ── Registro de eventos ────────────────────────────────

    def log(self, event_type: str, source: str,
            perception: str = "", perception_data: Dict = None,
            action: str = "", action_params: Dict = None,
            outcome: str = "", success: bool = None,
            reward: float = 0.0, duration_ms: float = 0.0,
            human_feedback: str = "", feedback_sentiment: float = None,
            context: Dict = None, tags: List[str] = None,
            importance: float = 0.5, episode_id: str = None) -> str:
        """Registra un evento completo en el lifelog."""
        lid = f"ll_{uuid.uuid4().hex[:10]}"
        now = time.time()

        with _lock:
            with _con() as c:
                c.execute("""
                    INSERT INTO lifelog (id, timestamp_ts, event_type, source,
                        perception, perception_data, action, action_params,
                        outcome, success, reward, duration_ms,
                        human_feedback, feedback_sentiment,
                        context, tags, importance, session_id, episode_id)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """, (lid, now, event_type, source,
                      perception, json.dumps(perception_data or {}),
                      action, json.dumps(action_params or {}),
                      outcome, int(success) if success is not None else None,
                      reward, duration_ms,
                      human_feedback, feedback_sentiment,
                      json.dumps(context or {}), json.dumps(tags or []),
                      importance, self._current_session, episode_id))

        self._session_count += 1
        return lid

    def log_perception(self, source: str, perception: str,
                       data: Dict = None, **kw) -> str:
        return self.log("perception", source, perception=perception,
                       perception_data=data, **kw)

    def log_action(self, source: str, action: str, params: Dict = None,
                   outcome: str = "", success: bool = None, **kw) -> str:
        return self.log("action", source, action=action,
                       action_params=params, outcome=outcome, success=success, **kw)

    def log_feedback(self, source: str, feedback: str,
                     sentiment: float = 0.0, **kw) -> str:
        return self.log("feedback", source, human_feedback=feedback,
                       feedback_sentiment=sentiment, **kw)

    def log_decision(self, source: str, perception: str, action: str,
                     outcome: str = "", success: bool = None,
                     reward: float = 0.0, **kw) -> str:
        return self.log("decision", source, perception=perception,
                       action=action, outcome=outcome, success=success,
                       reward=reward, importance=0.7, **kw)

    # ── Consultas ──────────────────────────────────────────

    def query(self, event_type: str = None, source: str = None,
              success: bool = None, min_importance: float = 0.0,
              since_ts: float = None, until_ts: float = None,
              episode_id: str = None, limit: int = 50,
              offset: int = 0) -> List[Dict]:
        clauses = ["1=1"]
        params: list = []
        if event_type:
            clauses.append("event_type = ?"); params.append(event_type)
        if source:
            clauses.append("source = ?"); params.append(source)
        if success is not None:
            clauses.append("success = ?"); params.append(int(success))
        if min_importance > 0:
            clauses.append("importance >= ?"); params.append(min_importance)
        if since_ts:
            clauses.append("timestamp_ts >= ?"); params.append(since_ts)
        if until_ts:
            clauses.append("timestamp_ts <= ?"); params.append(until_ts)
        if episode_id:
            clauses.append("episode_id = ?"); params.append(episode_id)

        params.extend([limit, offset])
        where = " AND ".join(clauses)

        with _con() as c:
            rows = c.execute(f"""
                SELECT * FROM lifelog WHERE {where}
                ORDER BY timestamp_ts DESC LIMIT ? OFFSET ?
            """, params).fetchall()
            return [dict(r) for r in rows]

    def get_timeline(self, hours: float = 24, limit: int = 100) -> List[Dict]:
        since = time.time() - hours * 3600
        return self.query(since_ts=since, limit=limit)

    def get_failures(self, limit: int = 30) -> List[Dict]:
        return self.query(success=False, limit=limit)

    def get_human_feedback_history(self, limit: int = 30) -> List[Dict]:
        with _con() as c:
            rows = c.execute("""
                SELECT * FROM lifelog WHERE human_feedback != '' AND human_feedback IS NOT NULL
                ORDER BY timestamp_ts DESC LIMIT ?
            """, (limit,)).fetchall()
            return [dict(r) for r in rows]

    def search(self, keyword: str, limit: int = 30) -> List[Dict]:
        pattern = f"%{keyword}%"
        with _con() as c:
            rows = c.execute("""
                SELECT * FROM lifelog
                WHERE perception LIKE ? OR action LIKE ? OR outcome LIKE ?
                    OR human_feedback LIKE ?
                ORDER BY timestamp_ts DESC LIMIT ?
            """, (pattern, pattern, pattern, pattern, limit)).fetchall()
            return [dict(r) for r in rows]

    # ── Estadisticas ───────────────────────────────────────

    def get_stats(self) -> Dict[str, Any]:
        with _con() as c:
            total = c.execute("SELECT COUNT(*) FROM lifelog").fetchone()[0]
            by_type = {}
            for r in c.execute("SELECT event_type, COUNT(*) as n FROM lifelog GROUP BY event_type").fetchall():
                by_type[r["event_type"]] = r["n"]
            by_source = {}
            for r in c.execute("SELECT source, COUNT(*) as n FROM lifelog GROUP BY source ORDER BY n DESC LIMIT 10").fetchall():
                by_source[r["source"]] = r["n"]
            success_count = c.execute("SELECT COUNT(*) FROM lifelog WHERE success = 1").fetchone()[0]
            fail_count = c.execute("SELECT COUNT(*) FROM lifelog WHERE success = 0").fetchone()[0]
            sessions = c.execute("SELECT COUNT(*) FROM lifelog_sessions").fetchone()[0]
            feedback_count = c.execute("SELECT COUNT(*) FROM lifelog WHERE human_feedback != ''").fetchone()[0]

        return {
            "total_entries": total,
            "by_type": by_type,
            "by_source": by_source,
            "success_count": success_count,
            "failure_count": fail_count,
            "success_rate": success_count / max(1, success_count + fail_count),
            "sessions": sessions,
            "current_session": self._current_session,
            "current_session_events": self._session_count,
            "human_feedback_entries": feedback_count,
        }

    def get_session_history(self, limit: int = 20) -> List[Dict]:
        with _con() as c:
            rows = c.execute("""
                SELECT * FROM lifelog_sessions ORDER BY start_ts DESC LIMIT ?
            """, (limit,)).fetchall()
            return [dict(r) for r in rows]


_instance: Optional[LifeLog] = None


def get_lifelog() -> LifeLog:
    global _instance
    if _instance is None:
        _instance = LifeLog()
    return _instance

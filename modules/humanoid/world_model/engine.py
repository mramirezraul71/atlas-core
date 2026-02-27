"""
WorldModel: Representacion interna actualizada del estado del mundo.

Mantiene un grafo espacial-temporal de entidades, sus estados y relaciones.
Se actualiza con cada percepcion y permite consultar/predecir estados.
"""
from __future__ import annotations

import json
import os
import sqlite3
import time
import threading
import uuid
from dataclasses import dataclass, field, asdict
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

_DB_PATH = os.path.join(
    os.environ.get("ATLAS_DATA_DIR", os.path.join(os.path.dirname(__file__), "..", "..", "..", "logs")),
    "world_model.sqlite",
)
_lock = threading.Lock()


def _con() -> sqlite3.Connection:
    os.makedirs(os.path.dirname(_DB_PATH), exist_ok=True)
    c = sqlite3.connect(_DB_PATH, timeout=10)
    c.row_factory = sqlite3.Row
    c.execute("PRAGMA journal_mode=WAL")
    c.execute("PRAGMA foreign_keys=ON")
    return c


def _ensure():
    with _con() as c:
        c.executescript("""
        CREATE TABLE IF NOT EXISTS entities (
            id TEXT PRIMARY KEY,
            name TEXT NOT NULL,
            entity_type TEXT NOT NULL,
            state TEXT DEFAULT '{}',
            properties TEXT DEFAULT '{}',
            last_seen_ts REAL,
            confidence REAL DEFAULT 1.0,
            created_ts TEXT DEFAULT (datetime('now')),
            updated_ts TEXT DEFAULT (datetime('now'))
        );
        CREATE INDEX IF NOT EXISTS idx_entities_type ON entities(entity_type);
        CREATE INDEX IF NOT EXISTS idx_entities_name ON entities(name);

        CREATE TABLE IF NOT EXISTS state_transitions (
            id TEXT PRIMARY KEY,
            entity_id TEXT NOT NULL,
            from_state TEXT,
            to_state TEXT,
            trigger_action TEXT,
            trigger_source TEXT,
            timestamp_ts REAL NOT NULL,
            metadata TEXT DEFAULT '{}',
            FOREIGN KEY (entity_id) REFERENCES entities(id)
        );
        CREATE INDEX IF NOT EXISTS idx_transitions_entity ON state_transitions(entity_id);
        CREATE INDEX IF NOT EXISTS idx_transitions_ts ON state_transitions(timestamp_ts);

        CREATE TABLE IF NOT EXISTS action_outcomes (
            id TEXT PRIMARY KEY,
            action_type TEXT NOT NULL,
            context TEXT DEFAULT '{}',
            predicted_outcome TEXT,
            actual_outcome TEXT,
            success INTEGER,
            reward REAL DEFAULT 0.0,
            timestamp_ts REAL NOT NULL,
            duration_ms REAL DEFAULT 0.0
        );
        CREATE INDEX IF NOT EXISTS idx_outcomes_action ON action_outcomes(action_type);

        CREATE TABLE IF NOT EXISTS world_snapshots (
            id TEXT PRIMARY KEY,
            snapshot TEXT NOT NULL,
            timestamp_ts REAL NOT NULL,
            trigger TEXT DEFAULT 'periodic'
        );
        CREATE INDEX IF NOT EXISTS idx_snapshots_ts ON world_snapshots(timestamp_ts);
        """)


_ensure()


@dataclass
class Entity:
    id: str
    name: str
    entity_type: str
    state: Dict[str, Any] = field(default_factory=dict)
    properties: Dict[str, Any] = field(default_factory=dict)
    confidence: float = 1.0
    last_seen_ts: float = 0.0


class WorldModel:
    """Modelo del mundo: entidades, estados, transiciones, predicciones."""

    def __init__(self):
        self._cache: Dict[str, Entity] = {}
        self._load_cache()

    def _load_cache(self):
        try:
            with _con() as c:
                rows = c.execute("SELECT * FROM entities ORDER BY updated_ts DESC LIMIT 500").fetchall()
                for r in rows:
                    self._cache[r["id"]] = Entity(
                        id=r["id"], name=r["name"], entity_type=r["entity_type"],
                        state=json.loads(r["state"] or "{}"),
                        properties=json.loads(r["properties"] or "{}"),
                        confidence=r["confidence"] or 1.0,
                        last_seen_ts=r["last_seen_ts"] or 0.0,
                    )
        except Exception:
            pass

    # ── Entidades ──────────────────────────────────────────

    def upsert_entity(self, name: str, entity_type: str,
                      state: Dict[str, Any] = None,
                      properties: Dict[str, Any] = None,
                      confidence: float = 1.0) -> Entity:
        now = time.time()
        eid = f"e_{entity_type}_{name}".replace(" ", "_").lower()

        prev = self._cache.get(eid)
        prev_state = prev.state.copy() if prev else {}

        ent = Entity(
            id=eid, name=name, entity_type=entity_type,
            state=state or (prev.state if prev else {}),
            properties={**(prev.properties if prev else {}), **(properties or {})},
            confidence=confidence, last_seen_ts=now,
        )
        self._cache[eid] = ent

        with _lock:
            with _con() as c:
                c.execute("""
                    INSERT INTO entities (id, name, entity_type, state, properties, last_seen_ts, confidence, updated_ts)
                    VALUES (?, ?, ?, ?, ?, ?, ?, datetime('now'))
                    ON CONFLICT(id) DO UPDATE SET
                        state=excluded.state, properties=excluded.properties,
                        last_seen_ts=excluded.last_seen_ts, confidence=excluded.confidence,
                        updated_ts=datetime('now')
                """, (eid, name, entity_type, json.dumps(state or {}),
                      json.dumps(ent.properties), now, confidence))

        if prev and prev_state != (state or {}):
            self._record_transition(eid, prev_state, state or {}, "update", "system")

        return ent

    def get_entity(self, entity_id: str) -> Optional[Entity]:
        return self._cache.get(entity_id)

    def query_entities(self, entity_type: str = None, state_filter: Dict[str, Any] = None) -> List[Entity]:
        results = list(self._cache.values())
        if entity_type:
            results = [e for e in results if e.entity_type == entity_type]
        if state_filter:
            results = [e for e in results if all(
                e.state.get(k) == v for k, v in state_filter.items()
            )]
        return results

    # ── Transiciones ───────────────────────────────────────

    def _record_transition(self, entity_id: str, from_state: Dict, to_state: Dict,
                           action: str, source: str):
        tid = f"tr_{uuid.uuid4().hex[:10]}"
        with _lock:
            with _con() as c:
                c.execute("""
                    INSERT INTO state_transitions (id, entity_id, from_state, to_state,
                        trigger_action, trigger_source, timestamp_ts, metadata)
                    VALUES (?, ?, ?, ?, ?, ?, ?, '{}')
                """, (tid, entity_id, json.dumps(from_state), json.dumps(to_state),
                      action, source, time.time()))

    def get_transitions(self, entity_id: str, limit: int = 20) -> List[Dict]:
        with _con() as c:
            rows = c.execute("""
                SELECT * FROM state_transitions WHERE entity_id = ?
                ORDER BY timestamp_ts DESC LIMIT ?
            """, (entity_id, limit)).fetchall()
            return [dict(r) for r in rows]

    # ── Outcomes (para prediccion) ─────────────────────────

    def record_outcome(self, action_type: str, context: Dict[str, Any],
                       predicted: str, actual: str, success: bool,
                       reward: float = 0.0, duration_ms: float = 0.0):
        oid = f"ao_{uuid.uuid4().hex[:10]}"
        with _lock:
            with _con() as c:
                c.execute("""
                    INSERT INTO action_outcomes (id, action_type, context, predicted_outcome,
                        actual_outcome, success, reward, timestamp_ts, duration_ms)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                """, (oid, action_type, json.dumps(context), predicted, actual,
                      int(success), reward, time.time(), duration_ms))

    def get_action_stats(self, action_type: str) -> Dict[str, Any]:
        with _con() as c:
            row = c.execute("""
                SELECT COUNT(*) as total,
                       SUM(success) as successes,
                       AVG(reward) as avg_reward,
                       AVG(duration_ms) as avg_duration
                FROM action_outcomes WHERE action_type = ?
            """, (action_type,)).fetchone()
            total = row["total"] or 0
            return {
                "action_type": action_type,
                "total": total,
                "successes": row["successes"] or 0,
                "success_rate": (row["successes"] or 0) / max(1, total),
                "avg_reward": row["avg_reward"] or 0.0,
                "avg_duration_ms": row["avg_duration"] or 0.0,
            }

    # ── Snapshots ──────────────────────────────────────────

    def take_snapshot(self, trigger: str = "periodic") -> str:
        snap = {
            "entities": {eid: asdict(e) for eid, e in self._cache.items()},
            "timestamp": time.time(),
            "entity_count": len(self._cache),
        }
        sid = f"snap_{uuid.uuid4().hex[:10]}"
        with _lock:
            with _con() as c:
                c.execute("""
                    INSERT INTO world_snapshots (id, snapshot, timestamp_ts, trigger)
                    VALUES (?, ?, ?, ?)
                """, (sid, json.dumps(snap), time.time(), trigger))
        return sid

    def get_latest_snapshot(self) -> Optional[Dict]:
        with _con() as c:
            row = c.execute("""
                SELECT * FROM world_snapshots ORDER BY timestamp_ts DESC LIMIT 1
            """).fetchone()
            if row:
                return {"id": row["id"], "snapshot": json.loads(row["snapshot"]),
                        "timestamp": row["timestamp_ts"], "trigger": row["trigger"]}
        return None

    # ── Resumen global ─────────────────────────────────────

    def get_state_summary(self) -> Dict[str, Any]:
        by_type: Dict[str, int] = {}
        for e in self._cache.values():
            by_type[e.entity_type] = by_type.get(e.entity_type, 0) + 1

        with _con() as c:
            transitions = c.execute("SELECT COUNT(*) FROM state_transitions").fetchone()[0]
            outcomes = c.execute("SELECT COUNT(*) FROM action_outcomes").fetchone()[0]
            snaps = c.execute("SELECT COUNT(*) FROM world_snapshots").fetchone()[0]

        return {
            "total_entities": len(self._cache),
            "entities_by_type": by_type,
            "total_transitions": transitions,
            "total_outcomes": outcomes,
            "total_snapshots": snaps,
        }


_instance: Optional[WorldModel] = None


def get_world_model() -> WorldModel:
    global _instance
    if _instance is None:
        _instance = WorldModel()
    return _instance

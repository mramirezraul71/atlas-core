"""
Memoria Autobiografica Artificial — Historia de vida de ATLAS.

Inspirado en Ella (2025) y Position Paper on Episodic Memory (2025).
Teje episodios individuales en una narrativa coherente de la vida del agente:
- Periodos (eras/epocas del sistema)
- Hitos (milestones: primer deploy, primera conversacion, etc.)
- Identidad (valores aprendidos, preferencias acumuladas, relaciones)
- Narrativa (resumen de experiencia de vida generado periodicamente)

Almacena en SQLite para persistencia total offline.
"""
from __future__ import annotations

import json
import os
import sqlite3
import threading
import time
import uuid
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Dict, List, Optional

_DB_PATH = os.path.join(
    os.environ.get("ATLAS_DATA_DIR", os.path.join(os.path.dirname(__file__), "..", "..", "..", "logs")),
    "autobiographical_memory.sqlite",
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
        CREATE TABLE IF NOT EXISTS life_periods (
            id TEXT PRIMARY KEY,
            name TEXT NOT NULL,
            description TEXT,
            start_ts REAL NOT NULL,
            end_ts REAL,
            metadata TEXT DEFAULT '{}',
            created_at TEXT DEFAULT (datetime('now'))
        );

        CREATE TABLE IF NOT EXISTS milestones (
            id TEXT PRIMARY KEY,
            title TEXT NOT NULL,
            description TEXT,
            category TEXT NOT NULL,
            importance REAL DEFAULT 0.5,
            timestamp_ts REAL NOT NULL,
            period_id TEXT,
            episode_ids TEXT DEFAULT '[]',
            metadata TEXT DEFAULT '{}',
            created_at TEXT DEFAULT (datetime('now')),
            FOREIGN KEY (period_id) REFERENCES life_periods(id)
        );
        CREATE INDEX IF NOT EXISTS idx_milestones_ts ON milestones(timestamp_ts);
        CREATE INDEX IF NOT EXISTS idx_milestones_cat ON milestones(category);

        CREATE TABLE IF NOT EXISTS identity_traits (
            id TEXT PRIMARY KEY,
            trait_type TEXT NOT NULL,
            key TEXT NOT NULL,
            value TEXT NOT NULL,
            confidence REAL DEFAULT 0.5,
            evidence_count INTEGER DEFAULT 1,
            first_observed_ts REAL,
            last_updated_ts REAL,
            UNIQUE(trait_type, key)
        );

        CREATE TABLE IF NOT EXISTS narratives (
            id TEXT PRIMARY KEY,
            period_id TEXT,
            scope TEXT NOT NULL,
            narrative TEXT NOT NULL,
            episode_count INTEGER DEFAULT 0,
            milestone_count INTEGER DEFAULT 0,
            generated_ts REAL NOT NULL,
            metadata TEXT DEFAULT '{}',
            FOREIGN KEY (period_id) REFERENCES life_periods(id)
        );
        CREATE INDEX IF NOT EXISTS idx_narratives_ts ON narratives(generated_ts);

        CREATE TABLE IF NOT EXISTS relationships (
            id TEXT PRIMARY KEY,
            entity_name TEXT NOT NULL,
            entity_type TEXT NOT NULL,
            sentiment REAL DEFAULT 0.0,
            interaction_count INTEGER DEFAULT 0,
            first_interaction_ts REAL,
            last_interaction_ts REAL,
            notes TEXT DEFAULT '[]',
            UNIQUE(entity_name, entity_type)
        );
        """)


_ensure()


class AutobiographicalMemory:
    """Memoria autobiografica: historia de vida, identidad y narrativa de ATLAS."""

    # ── Periodos de vida ───────────────────────────────────

    def start_period(self, name: str, description: str = "") -> str:
        pid = f"period_{uuid.uuid4().hex[:8]}"
        now = time.time()
        self._end_current_period()
        with _lock:
            with _con() as c:
                c.execute("""
                    INSERT INTO life_periods (id, name, description, start_ts)
                    VALUES (?, ?, ?, ?)
                """, (pid, name, description, now))
        return pid

    def _end_current_period(self):
        with _lock:
            with _con() as c:
                c.execute("""
                    UPDATE life_periods SET end_ts = ? WHERE end_ts IS NULL
                """, (time.time(),))

    def get_current_period(self) -> Optional[Dict]:
        with _con() as c:
            row = c.execute("""
                SELECT * FROM life_periods WHERE end_ts IS NULL
                ORDER BY start_ts DESC LIMIT 1
            """).fetchone()
            return dict(row) if row else None

    def get_all_periods(self) -> List[Dict]:
        with _con() as c:
            rows = c.execute("SELECT * FROM life_periods ORDER BY start_ts").fetchall()
            return [dict(r) for r in rows]

    # ── Hitos ──────────────────────────────────────────────

    def record_milestone(self, title: str, description: str, category: str,
                         importance: float = 0.7, episode_ids: List[str] = None,
                         metadata: Dict = None) -> str:
        mid = f"ms_{uuid.uuid4().hex[:8]}"
        now = time.time()
        period = self.get_current_period()
        with _lock:
            with _con() as c:
                c.execute("""
                    INSERT INTO milestones (id, title, description, category, importance,
                        timestamp_ts, period_id, episode_ids, metadata)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                """, (mid, title, description, category, importance, now,
                      period["id"] if period else None,
                      json.dumps(episode_ids or []),
                      json.dumps(metadata or {})))
        return mid

    def get_milestones(self, category: str = None, limit: int = 50) -> List[Dict]:
        with _con() as c:
            if category:
                rows = c.execute("""
                    SELECT * FROM milestones WHERE category = ?
                    ORDER BY timestamp_ts DESC LIMIT ?
                """, (category, limit)).fetchall()
            else:
                rows = c.execute("""
                    SELECT * FROM milestones ORDER BY importance DESC, timestamp_ts DESC LIMIT ?
                """, (limit,)).fetchall()
            return [dict(r) for r in rows]

    def get_milestone_categories(self) -> List[str]:
        with _con() as c:
            rows = c.execute("SELECT DISTINCT category FROM milestones ORDER BY category").fetchall()
            return [r["category"] for r in rows]

    # ── Identidad (traits, valores, preferencias) ──────────

    def update_trait(self, trait_type: str, key: str, value: str,
                     confidence: float = 0.5) -> None:
        now = time.time()
        with _lock:
            with _con() as c:
                existing = c.execute("""
                    SELECT * FROM identity_traits WHERE trait_type = ? AND key = ?
                """, (trait_type, key)).fetchone()
                if existing:
                    new_conf = min(1.0, existing["confidence"] + 0.05)
                    new_count = existing["evidence_count"] + 1
                    c.execute("""
                        UPDATE identity_traits SET value = ?, confidence = ?,
                            evidence_count = ?, last_updated_ts = ?
                        WHERE trait_type = ? AND key = ?
                    """, (value, new_conf, new_count, now, trait_type, key))
                else:
                    tid = f"trait_{uuid.uuid4().hex[:8]}"
                    c.execute("""
                        INSERT INTO identity_traits (id, trait_type, key, value,
                            confidence, evidence_count, first_observed_ts, last_updated_ts)
                        VALUES (?, ?, ?, ?, ?, 1, ?, ?)
                    """, (tid, trait_type, key, value, confidence, now, now))

    def get_identity(self) -> Dict[str, Any]:
        with _con() as c:
            rows = c.execute("""
                SELECT * FROM identity_traits ORDER BY confidence DESC
            """).fetchall()
        identity: Dict[str, Dict] = {}
        for r in rows:
            tt = r["trait_type"]
            if tt not in identity:
                identity[tt] = {}
            identity[tt][r["key"]] = {
                "value": r["value"],
                "confidence": r["confidence"],
                "evidence_count": r["evidence_count"],
            }
        return identity

    def get_traits(self, trait_type: str) -> List[Dict]:
        with _con() as c:
            rows = c.execute("""
                SELECT * FROM identity_traits WHERE trait_type = ?
                ORDER BY confidence DESC
            """, (trait_type,)).fetchall()
            return [dict(r) for r in rows]

    # ── Relaciones (con humanos, servicios, entidades) ─────

    def record_interaction(self, entity_name: str, entity_type: str = "human",
                           sentiment: float = 0.0, note: str = "") -> None:
        now = time.time()
        with _lock:
            with _con() as c:
                existing = c.execute("""
                    SELECT * FROM relationships WHERE entity_name = ? AND entity_type = ?
                """, (entity_name, entity_type)).fetchone()
                if existing:
                    notes = json.loads(existing["notes"] or "[]")
                    if note:
                        notes.append({"text": note, "ts": now})
                        if len(notes) > 50:
                            notes = notes[-50:]
                    avg_sent = (existing["sentiment"] * existing["interaction_count"] + sentiment) / (existing["interaction_count"] + 1)
                    c.execute("""
                        UPDATE relationships SET sentiment = ?, interaction_count = interaction_count + 1,
                            last_interaction_ts = ?, notes = ?
                        WHERE entity_name = ? AND entity_type = ?
                    """, (round(avg_sent, 3), now, json.dumps(notes), entity_name, entity_type))
                else:
                    rid = f"rel_{uuid.uuid4().hex[:8]}"
                    notes_list = [{"text": note, "ts": now}] if note else []
                    c.execute("""
                        INSERT INTO relationships (id, entity_name, entity_type, sentiment,
                            interaction_count, first_interaction_ts, last_interaction_ts, notes)
                        VALUES (?, ?, ?, ?, 1, ?, ?, ?)
                    """, (rid, entity_name, entity_type, sentiment, now, now, json.dumps(notes_list)))

    def get_relationships(self, entity_type: str = None) -> List[Dict]:
        with _con() as c:
            if entity_type:
                rows = c.execute("""
                    SELECT * FROM relationships WHERE entity_type = ?
                    ORDER BY interaction_count DESC
                """, (entity_type,)).fetchall()
            else:
                rows = c.execute("SELECT * FROM relationships ORDER BY interaction_count DESC").fetchall()
            return [dict(r) for r in rows]

    # ── Narrativas (resumen de vida generado) ──────────────

    def generate_narrative(self, scope: str = "full") -> str:
        milestones = self.get_milestones(limit=20)
        identity = self.get_identity()
        relationships = self.get_relationships()
        periods = self.get_all_periods()

        parts = []
        parts.append(f"ATLAS ha vivido {len(periods)} periodo(s) de vida.")

        if periods:
            for p in periods[-3:]:
                start = datetime.fromtimestamp(p["start_ts"]).strftime("%Y-%m-%d")
                end = datetime.fromtimestamp(p["end_ts"]).strftime("%Y-%m-%d") if p.get("end_ts") else "presente"
                parts.append(f"  Periodo '{p['name']}': {start} - {end}")

        if milestones:
            parts.append(f"\nHitos importantes ({len(milestones)}):")
            for m in milestones[:10]:
                parts.append(f"  [{m['category']}] {m['title']} (importancia: {m['importance']})")

        if identity:
            parts.append("\nIdentidad:")
            for trait_type, traits in identity.items():
                vals = ", ".join(f"{k}={v['value']}" for k, v in list(traits.items())[:5])
                parts.append(f"  {trait_type}: {vals}")

        if relationships:
            parts.append(f"\nRelaciones ({len(relationships)}):")
            for rel in relationships[:5]:
                parts.append(f"  {rel['entity_name']} ({rel['entity_type']}): "
                           f"sentiment={rel['sentiment']}, interacciones={rel['interaction_count']}")

        narrative = "\n".join(parts)

        nid = f"nar_{uuid.uuid4().hex[:8]}"
        with _lock:
            with _con() as c:
                period = self.get_current_period()
                c.execute("""
                    INSERT INTO narratives (id, period_id, scope, narrative,
                        milestone_count, generated_ts)
                    VALUES (?, ?, ?, ?, ?, ?)
                """, (nid, period["id"] if period else None, scope, narrative,
                      len(milestones), time.time()))

        return narrative

    def get_latest_narrative(self) -> Optional[str]:
        with _con() as c:
            row = c.execute("""
                SELECT narrative FROM narratives ORDER BY generated_ts DESC LIMIT 1
            """).fetchone()
            return row["narrative"] if row else None

    # ── Estadisticas ───────────────────────────────────────

    def get_stats(self) -> Dict[str, Any]:
        with _con() as c:
            periods = c.execute("SELECT COUNT(*) FROM life_periods").fetchone()[0]
            milestones = c.execute("SELECT COUNT(*) FROM milestones").fetchone()[0]
            traits = c.execute("SELECT COUNT(*) FROM identity_traits").fetchone()[0]
            narratives = c.execute("SELECT COUNT(*) FROM narratives").fetchone()[0]
            relationships = c.execute("SELECT COUNT(*) FROM relationships").fetchone()[0]
        return {
            "periods": periods, "milestones": milestones,
            "identity_traits": traits, "narratives": narratives,
            "relationships": relationships,
        }


_instance: Optional[AutobiographicalMemory] = None


def get_autobiographical_memory() -> AutobiographicalMemory:
    global _instance
    if _instance is None:
        _instance = AutobiographicalMemory()
    return _instance

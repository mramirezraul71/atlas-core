"""SQLite persistence for Nervous System signals and cycle snapshots index."""

from __future__ import annotations

import json
import os
import sqlite3
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional


def _db_path() -> Path:
    p = (os.getenv("NERVOUS_DB_PATH") or "").strip()
    if p:
        return Path(p).resolve()
    root = os.getenv("ATLAS_REPO_PATH") or os.getenv("ATLAS_PUSH_ROOT") or "C:\\ATLAS_PUSH"
    return (Path(root).resolve() / "logs" / "atlas_nervous.sqlite")


SCHEMA = """
CREATE TABLE IF NOT EXISTS signals (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  ts_utc TEXT NOT NULL,
  sensor_id TEXT NOT NULL,
  subsystem TEXT,
  severity TEXT NOT NULL,
  points INTEGER NOT NULL,
  fingerprint TEXT NOT NULL,
  message TEXT NOT NULL,
  details_json TEXT,
  status TEXT NOT NULL,
  incident_id TEXT,
  created_ts TEXT NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_signals_ts ON signals(ts_utc);
CREATE INDEX IF NOT EXISTS idx_signals_sensor ON signals(sensor_id);
CREATE INDEX IF NOT EXISTS idx_signals_fp ON signals(fingerprint);
CREATE INDEX IF NOT EXISTS idx_signals_status ON signals(status);
"""

_conn: Optional[sqlite3.Connection] = None


def _ensure() -> sqlite3.Connection:
    global _conn
    if _conn is not None:
        return _conn
    p = _db_path()
    p.parent.mkdir(parents=True, exist_ok=True)
    _conn = sqlite3.connect(str(p), check_same_thread=False)
    _conn.execute("PRAGMA journal_mode=WAL")
    for stmt in SCHEMA.split(";"):
        s = stmt.strip()
        if s:
            _conn.execute(s)
    _conn.commit()
    return _conn


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def insert_signal(
    *,
    sensor_id: str,
    subsystem: str,
    severity: str,
    points: int,
    fingerprint: str,
    message: str,
    details: Optional[Dict[str, Any]] = None,
    status: str = "open",
    incident_id: Optional[str] = None,
) -> int:
    conn = _ensure()
    ts = now_iso()
    conn.execute(
        """
        INSERT INTO signals (ts_utc, sensor_id, subsystem, severity, points, fingerprint, message, details_json, status, incident_id, created_ts)
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """,
        (
            ts,
            sensor_id,
            subsystem or "",
            severity,
            int(points),
            fingerprint,
            message[:500],
            json.dumps(details or {}, ensure_ascii=False)[:20000],
            status,
            incident_id,
            ts,
        ),
    )
    conn.commit()
    row = conn.execute("SELECT last_insert_rowid()").fetchone()
    return int(row[0] if row else 0)


def list_signals(status: Optional[str] = None, limit: int = 100) -> List[Dict[str, Any]]:
    conn = _ensure()
    if status:
        rows = conn.execute(
            """
            SELECT id, ts_utc, sensor_id, subsystem, severity, points, fingerprint, message, details_json, status, incident_id
            FROM signals WHERE status = ? ORDER BY id DESC LIMIT ?
            """,
            (status, int(limit)),
        ).fetchall()
    else:
        rows = conn.execute(
            """
            SELECT id, ts_utc, sensor_id, subsystem, severity, points, fingerprint, message, details_json, status, incident_id
            FROM signals ORDER BY id DESC LIMIT ?
            """,
            (int(limit),),
        ).fetchall()
    out: List[Dict[str, Any]] = []
    for r in rows:
        details = {}
        try:
            details = json.loads(r[8] or "{}") if r[8] else {}
        except Exception:
            details = {}
        out.append(
            {
                "id": r[0],
                "ts_utc": r[1],
                "sensor_id": r[2],
                "subsystem": r[3],
                "severity": r[4],
                "points": r[5],
                "fingerprint": r[6],
                "message": r[7],
                "details": details,
                "status": r[9],
                "incident_id": r[10],
            }
        )
    return out


def open_points(window_seconds: int = 300) -> Dict[str, Any]:
    """Returns points_total, count_open, by_severity for open signals in the last window.

    Nota: las señales de Nervio son eventos; por coherencia operativa el score debe reflejar el
    estado reciente, no castigar para siempre. Por eso se aplica una ventana temporal.
    """
    conn = _ensure()
    try:
        # SQLite can compare ISO strings lexicographically when same format. We just filter roughly by time in app.
        rows = conn.execute(
            """
            SELECT ts_utc, severity, points FROM signals WHERE status = 'open' ORDER BY id DESC LIMIT 1500
            """
        ).fetchall()
    except Exception:
        rows = []
    total = 0
    by: Dict[str, int] = {"low": 0, "med": 0, "high": 0, "critical": 0}
    count = 0
    # filter by window in Python (robusto para ISO con zona horaria)
    from datetime import datetime, timezone
    now = datetime.now(timezone.utc)
    for ts, sev, pts in rows:
        try:
            dt = datetime.fromisoformat(str(ts).replace("Z", "+00:00"))
            if (now - dt).total_seconds() > int(window_seconds):
                continue
        except Exception:
            # Si no se puede parsear timestamp, incluirlo (mejor señal que silencio)
            pass
        sev = (sev or "low").strip().lower()
        if sev not in by:
            by[sev] = 0
        by[sev] += int(pts or 0)
        total += int(pts or 0)
        count += 1
    return {"points_total": total, "by_severity": by, "count_open": count, "window_seconds": int(window_seconds)}


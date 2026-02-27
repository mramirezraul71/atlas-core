from __future__ import annotations

import json
import sqlite3
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


def _db_path() -> Path:
    return Path((__import__("os").environ.get("ATLAS_MTTR_DB_PATH") or r"C:\ATLAS_PUSH\logs\mttr.sqlite").strip())


SCHEMA = """
CREATE TABLE IF NOT EXISTS mttr_events (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  ts_start REAL NOT NULL,
  ts_end REAL,
  ok INTEGER,
  subsystem TEXT,
  signature TEXT,
  meta_json TEXT
);
CREATE INDEX IF NOT EXISTS idx_mttr_sub_ts ON mttr_events(subsystem, ts_start);
CREATE INDEX IF NOT EXISTS idx_mttr_sig_ts ON mttr_events(signature, ts_start);
"""


def _ensure() -> sqlite3.Connection:
    p = _db_path()
    p.parent.mkdir(parents=True, exist_ok=True)
    conn = sqlite3.connect(str(p))
    for stmt in [s.strip() for s in SCHEMA.split(";") if s.strip()]:
        conn.execute(stmt)
    conn.commit()
    return conn


def start(subsystem: str, signature: str, meta: Optional[Dict[str, Any]] = None) -> int:
    conn = _ensure()
    cur = conn.execute(
        "INSERT INTO mttr_events(ts_start, ts_end, ok, subsystem, signature, meta_json) VALUES (?,?,?,?,?,?)",
        (time.time(), None, None, (subsystem or "system")[:40], (signature or "")[:64], json.dumps(meta or {}, default=str)[:2000]),
    )
    conn.commit()
    return int(cur.lastrowid)


def resolve(event_id: int, ok: bool = True) -> None:
    if not event_id:
        return
    conn = _ensure()
    conn.execute("UPDATE mttr_events SET ts_end=?, ok=? WHERE id=?", (time.time(), 1 if ok else 0, int(event_id)))
    conn.commit()


def stats(hours: int = 24, limit: int = 10) -> Dict[str, Any]:
    cutoff = time.time() - max(1, int(hours)) * 3600
    conn = _ensure()
    rows = conn.execute(
        "SELECT subsystem, signature, ts_start, ts_end, ok FROM mttr_events WHERE ts_start >= ? AND ts_end IS NOT NULL",
        (cutoff,),
    ).fetchall()
    durs: List[Tuple[str, str, float, int]] = []
    for sub, sig, ts0, ts1, ok in rows:
        try:
            ms = max(0.0, (float(ts1) - float(ts0)) * 1000.0)
        except Exception:
            ms = 0.0
        durs.append((str(sub or "system"), str(sig or ""), ms, int(ok or 0)))

    def _p95(xs: List[float]) -> float:
        if not xs:
            return 0.0
        xs2 = sorted(xs)
        idx = int(round(0.95 * (len(xs2) - 1)))
        return float(xs2[max(0, min(idx, len(xs2) - 1))])

    all_ms = [x[2] for x in durs]
    ok_cnt = sum(1 for x in durs if x[3] == 1)
    fail_cnt = sum(1 for x in durs if x[3] == 0)
    by_sub: Dict[str, List[float]] = {}
    for sub, _sig, ms, _ok in durs:
        by_sub.setdefault(sub, []).append(ms)
    top = sorted(by_sub.items(), key=lambda kv: (sum(kv[1]) / max(1, len(kv[1]))), reverse=True)[: max(1, int(limit))]
    return {
        "hours": int(hours),
        "count": len(durs),
        "ok_count": ok_cnt,
        "fail_count": fail_cnt,
        "avg_ms": round(sum(all_ms) / max(1, len(all_ms)), 2) if all_ms else 0.0,
        "p95_ms": round(_p95(all_ms), 2) if all_ms else 0.0,
        "by_subsystem": [{"subsystem": sub, "avg_ms": round(sum(xs) / max(1, len(xs)), 2), "count": len(xs)} for sub, xs in top],
    }


def format_stats_human(hours: int = 24) -> str:
    s = stats(hours=hours, limit=8)
    if not s.get("count"):
        return "MTTR (recuperación): sin datos recientes."
    lines = [f"MTTR {s['hours']}h: avg={int(s['avg_ms'])}ms p95={int(s['p95_ms'])}ms eventos={s['count']}"]
    for item in s.get("by_subsystem") or []:
        lines.append(f"- {item['subsystem']}: avg={int(item['avg_ms'])}ms (n={item['count']})")
    return "\n".join(lines)


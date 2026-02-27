"""SQLite store for approval queue."""
from __future__ import annotations

import json
import os
import sqlite3
import threading
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

SCHEMA = """
CREATE TABLE IF NOT EXISTS approvals (
    id TEXT PRIMARY KEY,
    created_ts TEXT NOT NULL,
    action TEXT NOT NULL,
    payload_json TEXT,
    risk TEXT NOT NULL DEFAULT 'medium',
    status TEXT NOT NULL DEFAULT 'pending',
    job_id TEXT,
    run_id INTEGER,
    resolved_ts TEXT,
    resolved_by TEXT,
    requires_2fa INTEGER DEFAULT 0,
    expires_at TEXT,
    approval_signature TEXT,
    origin_node_id TEXT,
    chain_hash TEXT
);
CREATE INDEX IF NOT EXISTS idx_approvals_status ON approvals(status);
CREATE INDEX IF NOT EXISTS idx_approvals_created ON approvals(created_ts);
"""


def _db_path() -> str:
    p = os.getenv("ATLAS_APPROVALS_DB_PATH") or os.getenv("ATLAS_MEMORY_DB_PATH")
    if p:
        return str(Path(p).parent / "atlas_approvals.sqlite")
    return str(Path(os.getcwd()) / "logs" / "atlas_approvals.sqlite")


_LOCK = threading.Lock()
_conn: Optional[sqlite3.Connection] = None


def _ensure() -> sqlite3.Connection:
    """
    SQLite thread-safe:
    - check_same_thread=False para permitir uso desde distintas threads del servidor
    - lock global para serializar operaciones (evita corrupciones/locks raros)
    """
    global _conn
    with _LOCK:
        if _conn is not None:
            return _conn
        path = _db_path()
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        _conn = sqlite3.connect(path, check_same_thread=False)
        try:
            _conn.execute("PRAGMA journal_mode=WAL;")
        except Exception:
            pass
        _conn.executescript(SCHEMA)
        for col, typ in [
            ("requires_2fa", "INTEGER DEFAULT 0"),
            ("expires_at", "TEXT"),
            ("approval_signature", "TEXT"),
            ("origin_node_id", "TEXT"),
            ("chain_hash", "TEXT"),
            ("request_hash", "TEXT"),
            ("chain_prev_hash", "TEXT"),
            ("approved_via", "TEXT"),
            ("correlation_id", "TEXT"),
        ]:
            try:
                _conn.execute(f"ALTER TABLE approvals ADD COLUMN {col} {typ}")
            except sqlite3.OperationalError:
                pass
        _conn.commit()
        return _conn


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


def create(
    action: str,
    payload: Dict[str, Any],
    risk: str = "medium",
    job_id: Optional[str] = None,
    run_id: Optional[int] = None,
    requires_2fa: bool = False,
    origin_node_id: Optional[str] = None,
) -> Dict[str, Any]:
    from .ttl import expires_at_seconds
    from .chain import compute_request_hash, enabled as chain_enabled
    aid = str(uuid.uuid4())[:12]
    now = _now()
    expires_at = expires_at_seconds()
    request_hash = compute_request_hash(payload) if chain_enabled() else ""
    conn = _ensure()
    with _LOCK:
        conn.execute(
        """INSERT INTO approvals (id, created_ts, action, payload_json, risk, status, job_id, run_id, requires_2fa, expires_at, origin_node_id, request_hash)
           VALUES (?, ?, ?, ?, ?, 'pending', ?, ?, ?, ?, ?, ?)""",
        (aid, now, action, json.dumps(payload), risk, job_id, run_id, 1 if requires_2fa else 0, expires_at, origin_node_id, request_hash[:64] if request_hash else None),
        )
        conn.commit()
    return {
        "id": aid, "created_ts": now, "action": action, "payload": payload, "risk": risk, "status": "pending",
        "job_id": job_id, "run_id": run_id, "requires_2fa": requires_2fa, "expires_at": expires_at, "origin_node_id": origin_node_id,
    }


def list_items(
    status: Optional[str] = None,
    risk: Optional[str] = None,
    limit: int = 50,
) -> List[Dict[str, Any]]:
    conn = _ensure()
    cols = "id, created_ts, action, payload_json, risk, status, job_id, run_id, resolved_ts, resolved_by, requires_2fa, expires_at, approval_signature, origin_node_id, chain_hash"
    where, params = [], []
    if status:
        where.append("status = ?")
        params.append(status)
    if risk:
        where.append("risk = ?")
        params.append(risk.strip().lower())
    params.append(limit)
    sql = f"SELECT {cols} FROM approvals"
    if where:
        sql += " WHERE " + " AND ".join(where)
    sql += " ORDER BY created_ts DESC LIMIT ?"
    with _LOCK:
        rows = conn.execute(sql, params).fetchall()
    out = []
    for r in rows:
        d = {
            "id": r[0], "created_ts": r[1], "action": r[2],
            "payload": json.loads(r[3]) if r[3] else {},
            "risk": r[4], "status": r[5], "job_id": r[6], "run_id": r[7],
            "resolved_ts": r[8], "resolved_by": r[9],
        }
        d["requires_2fa"] = bool(r[10]) if len(r) > 10 else False
        d["expires_at"] = r[11] if len(r) > 11 else None
        d["approval_signature"] = r[12] if len(r) > 12 else None
        d["origin_node_id"] = r[13] if len(r) > 13 else None
        d["chain_hash"] = r[14] if len(r) > 14 else None
        try:
            d["expired"] = _is_expired(d.get("expires_at"))
        except Exception:
            d["expired"] = False
        out.append(d)
    return out


def get(aid: str) -> Optional[Dict[str, Any]]:
    conn = _ensure()
    cols = "id, created_ts, action, payload_json, risk, status, job_id, run_id, resolved_ts, resolved_by, requires_2fa, expires_at, approval_signature, origin_node_id, chain_hash"
    with _LOCK:
        row = conn.execute(f"SELECT {cols} FROM approvals WHERE id = ?", (aid,)).fetchone()
    if not row:
        return None
    d = {
        "id": row[0], "created_ts": row[1], "action": row[2],
        "payload": json.loads(row[3]) if row[3] else {},
        "risk": row[4], "status": row[5], "job_id": row[6], "run_id": row[7],
        "resolved_ts": row[8], "resolved_by": row[9],
    }
    if len(row) > 10:
        d["requires_2fa"] = bool(row[10])
        d["expires_at"] = row[11]
        d["approval_signature"] = row[12]
        d["origin_node_id"] = row[13]
        d["chain_hash"] = row[14]
    try:
        d["expired"] = _is_expired(d.get("expires_at"))
    except Exception:
        d["expired"] = False
    return d


def find_pending_equivalent(action: str, risk: str, request_hash: str) -> Optional[Dict[str, Any]]:
    """Return an active pending approval equivalent to the incoming request if any."""
    if not (action or "").strip() or not (request_hash or "").strip():
        return None
    conn = _ensure()
    cols = "id, created_ts, action, payload_json, risk, status, job_id, run_id, resolved_ts, resolved_by, requires_2fa, expires_at, approval_signature, origin_node_id, chain_hash"
    try:
        with _LOCK:
            row = conn.execute(
                f"SELECT {cols} FROM approvals WHERE status = 'pending' AND action = ? AND risk = ? AND request_hash = ? ORDER BY created_ts DESC LIMIT 1",
                ((action or "").strip(), (risk or "").strip().lower(), (request_hash or "").strip()[:64]),
            ).fetchone()
    except Exception:
        return None

    if not row:
        return None

    d = {
        "id": row[0], "created_ts": row[1], "action": row[2],
        "payload": json.loads(row[3]) if row[3] else {},
        "risk": row[4], "status": row[5], "job_id": row[6], "run_id": row[7],
        "resolved_ts": row[8], "resolved_by": row[9],
    }
    if len(row) > 10:
        d["requires_2fa"] = bool(row[10])
        d["expires_at"] = row[11]
        d["approval_signature"] = row[12]
        d["origin_node_id"] = row[13]
        d["chain_hash"] = row[14]
    try:
        d["expired"] = _is_expired(d.get("expires_at"))
    except Exception:
        d["expired"] = False
    return d


def _is_expired(expires_at: Optional[str]) -> bool:
    from .ttl import is_expired as ttl_expired
    return ttl_expired(expires_at)


def _compute_chain_hash(prev_hash: str, item: Dict[str, Any], resolved_ts: str, resolved_by: str) -> str:
    from .chain import compute_chain_hash, compute_request_hash
    req_hash = item.get("request_hash") or compute_request_hash(item.get("payload") or {})
    return compute_chain_hash(prev_hash, item.get("id", ""), "approved", req_hash, resolved_ts, resolved_by)


def _last_chain_hash(conn: sqlite3.Connection) -> str:
    row = conn.execute("SELECT chain_hash FROM approvals WHERE status='approved' AND chain_hash IS NOT NULL ORDER BY created_ts DESC LIMIT 1").fetchone()
    return row[0] if row and row[0] else "0"


def approve(aid: str, resolved_by: str = "api", signature: Optional[str] = None) -> bool:
    conn = _ensure()
    item = get(aid)
    if not item or item.get("status") != "pending":
        return False
    if _is_expired(item.get("expires_at")):
        return False
    now = _now()
    with _LOCK:
        prev = _last_chain_hash(conn)
        chain = _compute_chain_hash(prev, item, now, resolved_by)
        cur = conn.execute(
            "UPDATE approvals SET status = 'approved', resolved_ts = ?, resolved_by = ?, approval_signature = ?, chain_hash = ? WHERE id = ? AND status = 'pending'",
            (now, resolved_by, signature or "", chain, aid),
        )
        conn.commit()
        return cur.rowcount > 0


def reject(aid: str, resolved_by: str = "api") -> bool:
    conn = _ensure()
    item = get(aid)
    if not item or item.get("status") != "pending":
        return False
    now = _now()
    with _LOCK:
        cur = conn.execute("UPDATE approvals SET status = 'rejected', resolved_ts = ?, resolved_by = ? WHERE id = ? AND status = 'pending'", (now, resolved_by, aid))
        conn.commit()
        return cur.rowcount > 0


def expire_pending(limit: int = 500) -> Dict[str, Any]:
    """Mark expired pending approvals as status='expired' (autonomous reaper)."""
    from .ttl import is_expired

    conn = _ensure()
    lim = max(1, min(int(limit or 500), 5000))
    now = _now()
    expired_ids: List[str] = []

    with _LOCK:
        rows = conn.execute(
            "SELECT id, expires_at FROM approvals WHERE status = 'pending' ORDER BY created_ts ASC LIMIT ?",
            (lim,),
        ).fetchall()

        for row in rows:
            aid = row[0]
            expires_at = row[1] if len(row) > 1 else None
            if not is_expired(expires_at):
                continue
            conn.execute(
                "UPDATE approvals SET status = 'expired', resolved_ts = ?, resolved_by = ? WHERE id = ? AND status = 'pending'",
                (now, "system_ttl_reaper", aid),
            )
            expired_ids.append(aid)

        if expired_ids:
            conn.commit()

    return {
        "ok": True,
        "expired_count": len(expired_ids),
        "expired_ids": expired_ids[:50],
        "scanned": len(rows),
    }


def list_for_chain() -> List[Dict[str, Any]]:
    """All approvals ordered by created_ts for chain verification."""
    conn = _ensure()
    cols = "id, created_ts, action, payload_json, risk, status, job_id, run_id, resolved_ts, resolved_by, requires_2fa, expires_at, approval_signature, origin_node_id, chain_hash"
    with _LOCK:
        rows = conn.execute(f"SELECT {cols} FROM approvals ORDER BY created_ts ASC").fetchall()
    out = []
    for r in rows:
        d = {"id": r[0], "created_ts": r[1], "action": r[2], "payload": json.loads(r[3]) if r[3] else {}, "risk": r[4], "status": r[5], "job_id": r[6], "run_id": r[7], "resolved_ts": r[8], "resolved_by": r[9]}
        if len(r) > 14:
            d["chain_hash"] = r[14]
        out.append(d)
    return out


def verify_chain() -> Dict[str, Any]:
    """Recompute hashes; return {ok, valid, broken_at_id?, last_hash, count}."""
    from .chain import verify_chain as chain_verify, enabled
    items = list_for_chain()
    if not enabled():
        return {"ok": True, "valid": True, "broken_at_id": None, "last_hash": "0", "count": 0}
    return chain_verify(items)

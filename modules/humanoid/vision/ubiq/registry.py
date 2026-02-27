from __future__ import annotations

import os
import sqlite3
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional


_LOCK = threading.RLock()
_CONN: Optional[sqlite3.Connection] = None


def _db_path() -> Path:
    raw = (os.getenv("ATLAS_VISION_UBIQ_DB_PATH") or r"C:\ATLAS_PUSH\logs\vision_ubiq.db").strip()
    return Path(raw)


def ensure_db() -> Path:
    p = _db_path()
    p.parent.mkdir(parents=True, exist_ok=True)
    global _CONN
    with _LOCK:
        if _CONN is None:
            _CONN = sqlite3.connect(str(p), check_same_thread=False)
            _CONN.execute("PRAGMA journal_mode=WAL;")
            _CONN.execute("PRAGMA synchronous=NORMAL;")
            _CONN.execute(
                """
                CREATE TABLE IF NOT EXISTS cameras (
                  id TEXT PRIMARY KEY,
                  ip TEXT,
                  port INTEGER,
                  protocol TEXT,
                  url TEXT,
                  model TEXT,
                  source TEXT,
                  type TEXT,
                  onvif_xaddr TEXT,
                  meta_json TEXT,
                  added_ts REAL,
                  last_seen_ts REAL
                );
                """
            )
            _CONN.execute(
                """
                CREATE TABLE IF NOT EXISTS settings (
                  key TEXT PRIMARY KEY,
                  value TEXT,
                  updated_ts REAL
                );
                """
            )
            _CONN.commit()
    return p


def _conn() -> sqlite3.Connection:
    ensure_db()
    assert _CONN is not None
    return _CONN


def set_setting(key: str, value: str) -> None:
    k = (key or "").strip()
    if not k:
        return
    with _LOCK:
        c = _conn()
        c.execute(
            "INSERT INTO settings(key,value,updated_ts) VALUES(?,?,?) "
            "ON CONFLICT(key) DO UPDATE SET value=excluded.value, updated_ts=excluded.updated_ts",
            (k, str(value), time.time()),
        )
        c.commit()


def get_setting(key: str, default: str = "") -> str:
    k = (key or "").strip()
    if not k:
        return default
    with _LOCK:
        c = _conn()
        row = c.execute("SELECT value FROM settings WHERE key=?", (k,)).fetchone()
        if not row or row[0] is None:
            return default
        return str(row[0])


def list_cameras(limit: int = 200) -> List[Dict[str, Any]]:
    with _LOCK:
        c = _conn()
        rows = c.execute(
            "SELECT id,ip,port,protocol,url,model,source,type,onvif_xaddr,added_ts,last_seen_ts,meta_json "
            "FROM cameras ORDER BY last_seen_ts DESC LIMIT ?",
            (int(limit),),
        ).fetchall()
    out: List[Dict[str, Any]] = []
    for r in rows:
        out.append(
            {
                "id": r[0],
                "ip": r[1],
                "port": r[2],
                "protocol": r[3],
                "url": r[4],
                "model": r[5],
                "source": r[6],
                "type": r[7],
                "onvif_xaddr": r[8],
                "added_ts": r[9],
                "last_seen_ts": r[10],
                "meta_json": r[11] or "",
            }
        )
    return out


def get_camera(cam_id: str) -> Optional[Dict[str, Any]]:
    cid = (cam_id or "").strip()
    if not cid:
        return None
    with _LOCK:
        c = _conn()
        r = c.execute(
            "SELECT id,ip,port,protocol,url,model,source,type,onvif_xaddr,added_ts,last_seen_ts,meta_json "
            "FROM cameras WHERE id=?",
            (cid,),
        ).fetchone()
    if not r:
        return None
    return {
        "id": r[0],
        "ip": r[1],
        "port": r[2],
        "protocol": r[3],
        "url": r[4],
        "model": r[5],
        "source": r[6],
        "type": r[7],
        "onvif_xaddr": r[8],
        "added_ts": r[9],
        "last_seen_ts": r[10],
        "meta_json": r[11] or "",
    }


def upsert_camera(cam: Dict[str, Any]) -> bool:
    """
    Inserta/actualiza una cámara.
    Retorna True si es nueva (no existía antes), False si fue update.
    """
    import json

    cid = str(cam.get("id") or "").strip()
    if not cid:
        return False
    now = time.time()
    ip = str(cam.get("ip") or "").strip()
    port = int(cam.get("port") or 0) if str(cam.get("port") or "").strip() else 0
    protocol = str(cam.get("protocol") or "").strip()
    url = str(cam.get("url") or "").strip()
    model = str(cam.get("model") or "").strip()
    source = str(cam.get("source") or "network").strip()
    typ = str(cam.get("type") or "network").strip()
    onvif_xaddr = str(cam.get("onvif_xaddr") or "").strip()
    meta = cam.get("meta") or cam.get("meta_json") or {}
    meta_json = meta if isinstance(meta, str) else json.dumps(meta, ensure_ascii=False)
    with _LOCK:
        c = _conn()
        existed = bool(c.execute("SELECT 1 FROM cameras WHERE id=?", (cid,)).fetchone())
        c.execute(
            """
            INSERT INTO cameras(id,ip,port,protocol,url,model,source,type,onvif_xaddr,meta_json,added_ts,last_seen_ts)
            VALUES(?,?,?,?,?,?,?,?,?,?,?,?)
            ON CONFLICT(id) DO UPDATE SET
              ip=excluded.ip,
              port=excluded.port,
              protocol=excluded.protocol,
              url=excluded.url,
              model=excluded.model,
              source=excluded.source,
              type=excluded.type,
              onvif_xaddr=excluded.onvif_xaddr,
              meta_json=excluded.meta_json,
              last_seen_ts=excluded.last_seen_ts
            """,
            (cid, ip, port, protocol, url, model, source, typ, onvif_xaddr, meta_json, now, now),
        )
        c.commit()
    return not existed


from __future__ import annotations

import json
import sqlite3
import threading
import time
import uuid
from pathlib import Path
from typing import Any, Dict, List, Optional

import cv2


_DB_LOCK = threading.Lock()


def _db_path() -> Path:
    return Path(__file__).resolve().parent / "registry.db"


def _conn() -> sqlite3.Connection:
    c = sqlite3.connect(str(_db_path()), check_same_thread=False)
    c.row_factory = sqlite3.Row
    return c


def _init_db() -> None:
    with _DB_LOCK:
        c = _conn()
        cur = c.cursor()
        cur.execute(
            """
            CREATE TABLE IF NOT EXISTS cameras (
                id TEXT PRIMARY KEY,
                type TEXT NOT NULL,
                url TEXT,
                label TEXT,
                health TEXT DEFAULT 'unknown',
                priority INTEGER DEFAULT 99,
                connection_method TEXT,
                metadata TEXT,
                last_seen REAL,
                created_at REAL NOT NULL
            )
            """
        )
        cur.execute("CREATE INDEX IF NOT EXISTS idx_cameras_type ON cameras(type)")
        cur.execute("CREATE INDEX IF NOT EXISTS idx_cameras_health ON cameras(health)")
        cur.execute("CREATE INDEX IF NOT EXISTS idx_cameras_priority ON cameras(priority)")
        c.commit()
        c.close()


_init_db()


def register_camera(
    *,
    type: str,
    url: str,
    label: str,
    priority: int = 99,
    connection_method: Optional[str] = None,
    metadata: Optional[Dict[str, Any]] = None,
) -> str:
    now = time.time()
    cam_type = str(type or "unknown")
    cam_url = str(url or "")
    cam_label = str(label or "Camera")
    cam_meta = json.dumps(metadata or {}, ensure_ascii=False)
    with _DB_LOCK:
        c = _conn()
        cur = c.cursor()
        cur.execute("SELECT id FROM cameras WHERE type = ? AND url = ? LIMIT 1", (cam_type, cam_url))
        row = cur.fetchone()
        if row:
            cam_id = str(row["id"])
            cur.execute(
                """
                UPDATE cameras
                   SET label = ?, priority = ?, connection_method = ?, metadata = ?, last_seen = ?
                 WHERE id = ?
                """,
                (cam_label, int(priority), connection_method, cam_meta, now, cam_id),
            )
        else:
            cam_id = str(uuid.uuid4())
            cur.execute(
                """
                INSERT INTO cameras (id, type, url, label, health, priority, connection_method, metadata, last_seen, created_at)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    cam_id,
                    cam_type,
                    cam_url,
                    cam_label,
                    "unknown",
                    int(priority),
                    connection_method,
                    cam_meta,
                    now,
                    now,
                ),
            )
        c.commit()
        c.close()
    return cam_id


def list_cameras(filter_type: Optional[str] = None, only_online: bool = False) -> List[Dict[str, Any]]:
    query = "SELECT * FROM cameras WHERE 1=1"
    params: List[Any] = []
    if filter_type:
        query += " AND type = ?"
        params.append(filter_type)
    if only_online:
        query += " AND health = 'online'"
    query += " ORDER BY priority ASC, created_at ASC"
    with _DB_LOCK:
        c = _conn()
        cur = c.cursor()
        cur.execute(query, params)
        rows = cur.fetchall()
        c.close()

    out: List[Dict[str, Any]] = []
    for row in rows:
        item = dict(row)
        try:
            item["metadata"] = json.loads(item.get("metadata") or "{}")
        except Exception:
            item["metadata"] = {}
        out.append(item)
    return out


def get_camera_by_id(cam_id: str) -> Optional[Dict[str, Any]]:
    with _DB_LOCK:
        c = _conn()
        cur = c.cursor()
        cur.execute("SELECT * FROM cameras WHERE id = ?", (cam_id,))
        row = cur.fetchone()
        c.close()
    if not row:
        return None
    item = dict(row)
    try:
        item["metadata"] = json.loads(item.get("metadata") or "{}")
    except Exception:
        item["metadata"] = {}
    return item


def update_camera(cam_id: str, **fields) -> bool:
    if not fields:
        return False
    if "metadata" in fields and isinstance(fields["metadata"], dict):
        fields["metadata"] = json.dumps(fields["metadata"], ensure_ascii=False)
    keys = list(fields.keys())
    values = [fields[k] for k in keys]
    set_sql = ", ".join(f"{k} = ?" for k in keys)
    with _DB_LOCK:
        c = _conn()
        cur = c.cursor()
        cur.execute(f"UPDATE cameras SET {set_sql} WHERE id = ?", values + [cam_id])
        ok = cur.rowcount > 0
        c.commit()
        c.close()
    return ok


def delete_camera(cam_id: str) -> bool:
    with _DB_LOCK:
        c = _conn()
        cur = c.cursor()
        cur.execute("DELETE FROM cameras WHERE id = ?", (cam_id,))
        ok = cur.rowcount > 0
        c.commit()
        c.close()
    return ok


def health_check(cam_id: str, timeout_s: float = 2.5) -> Dict[str, Any]:
    cam = get_camera_by_id(cam_id)
    if not cam:
        return {"ok": False, "health": "offline", "error": "camera_not_found"}
    url = str(cam.get("url") or "")
    t0 = time.perf_counter()
    try:
        if url.startswith("cv2://"):
            idx = int(url.split("://", 1)[1])
            cap = cv2.VideoCapture(idx)
        elif url.startswith(("rtsp://", "http://", "https://")):
            cap = cv2.VideoCapture(url)
        else:
            cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            try:
                cap.release()
            except Exception:
                pass
            update_camera(cam_id, health="offline", last_seen=time.time())
            return {"ok": False, "health": "offline", "error": "cannot_open"}
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        start = time.perf_counter()
        ok, frame = cap.read()
        elapsed = (time.perf_counter() - start) + (time.perf_counter() - t0)
        try:
            cap.release()
        except Exception:
            pass
        if not ok or frame is None:
            update_camera(cam_id, health="degraded", last_seen=time.time())
            return {"ok": False, "health": "degraded", "latency_ms": round(elapsed * 1000, 2), "error": "no_frame"}
        update_camera(cam_id, health="online", last_seen=time.time())
        return {"ok": True, "health": "online", "latency_ms": round(elapsed * 1000, 2), "timeout_s": timeout_s}
    except Exception as e:
        update_camera(cam_id, health="offline", last_seen=time.time())
        return {"ok": False, "health": "offline", "error": str(e)}


def deduplicate_cameras() -> Dict[str, Any]:
    """
    Elimina duplicados por (type, url), conservando el más reciente por last_seen/created_at.
    """
    with _DB_LOCK:
        c = _conn()
        cur = c.cursor()
        cur.execute(
            """
            SELECT id, type, url, COALESCE(last_seen, 0) AS last_seen, COALESCE(created_at, 0) AS created_at
            FROM cameras
            ORDER BY type ASC, url ASC, last_seen DESC, created_at DESC
            """
        )
        rows = cur.fetchall()

        keep_ids = set()
        remove_ids: List[str] = []
        seen_keys = set()
        for row in rows:
            key = (str(row["type"] or ""), str(row["url"] or ""))
            rid = str(row["id"])
            if key not in seen_keys:
                seen_keys.add(key)
                keep_ids.add(rid)
            else:
                remove_ids.append(rid)

        removed = 0
        for rid in remove_ids:
            cur.execute("DELETE FROM cameras WHERE id = ?", (rid,))
            removed += int(cur.rowcount or 0)

        # Limpieza adicional de entradas vacías sin URL útil.
        cur.execute("DELETE FROM cameras WHERE TRIM(COALESCE(url, '')) = ''")
        removed += int(cur.rowcount or 0)

        cur.execute("SELECT COUNT(*) AS n FROM cameras")
        total = int(cur.fetchone()["n"])
        c.commit()
        c.close()
    return {"ok": True, "removed": removed, "total": total}


def get_active_camera_by_priority(prefer_online: bool = True) -> Optional[Dict[str, Any]]:
    cameras = list_cameras()
    if not cameras:
        return None
    if prefer_online:
        online = [c for c in cameras if str(c.get("health") or "").lower() == "online"]
        if online:
            cameras = online
    best = min(cameras, key=lambda c: int(c.get("priority", 99)))
    return best

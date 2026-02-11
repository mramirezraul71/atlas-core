"""Store: write plans, runs, artifacts, decisions, summaries into memory. Optional FTS index."""
from __future__ import annotations

import json
from typing import Any, Dict, List, Optional

from .db import _ensure, fts_available, new_id, now_iso


def _fts_insert(thread_id: Optional[str], task_id: Optional[str], kind: str, content: str) -> None:
    if not fts_available():
        return
    try:
        conn = _ensure()
        conn.execute(
            "INSERT INTO memory_fts (content, thread_id, task_id, kind) VALUES (?, ?, ?, ?)",
            (content[:10000], thread_id or "", task_id or "", kind),
        )
        conn.commit()
    except Exception:
        pass


def ensure_thread(thread_id: Optional[str] = None, title: str = "") -> str:
    if thread_id:
        return thread_id
    tid = new_id()
    now = now_iso()
    conn = _ensure()
    conn.execute("INSERT INTO threads (id, title, created_ts, updated_ts) VALUES (?, ?, ?, ?)", (tid, title or "default", now, now))
    conn.commit()
    return tid


def list_threads(limit: int = 50) -> List[Dict[str, Any]]:
    """List threads ordered by updated_ts desc. For GET /memory/thread/list."""
    conn = _ensure()
    rows = conn.execute(
        "SELECT id, title, created_ts, updated_ts FROM threads ORDER BY updated_ts DESC LIMIT ?",
        (limit,),
    ).fetchall()
    return [{"id": r[0], "title": r[1], "created_ts": r[2], "updated_ts": r[3]} for r in rows]


def memory_write(
    thread_id: Optional[str],
    kind: str,
    payload: Dict[str, Any],
    task_id: Optional[str] = None,
    run_id: Optional[int] = None,
) -> Dict[str, Any]:
    """
    Generic write: kind in (artifact, decision, summary).
    Returns {ok, id, kind, thread_id?, error}. Redacts secrets in content.
    """
    content = json.dumps(payload)[:10000]
    content = _redact_secrets(content)
    thread_id = ensure_thread(thread_id or None, payload.get("title", "")[:200])
    try:
        if kind == "artifact":
            path_or_uri = str(payload.get("path_or_uri", ""))[:2048]
            preview = str(payload.get("content_preview", content))[:2048]
            aid = store_artifact(task_id, run_id, payload.get("artifact_kind", "write"), path_or_uri, preview)
            return {"ok": True, "id": aid, "kind": "artifact", "thread_id": thread_id, "error": None}
        if kind == "decision":
            did = store_decision(thread_id, task_id, payload.get("decision_type", "api"), payload)
            return {"ok": True, "id": did, "kind": "decision", "thread_id": thread_id, "error": None}
        if kind == "summary":
            text = str(payload.get("content", content))[:50000]
            sid = add_summary(thread_id, text)
            return {"ok": True, "id": sid, "kind": "summary", "thread_id": thread_id, "error": None}
        return {"ok": False, "id": None, "error": f"unknown kind: {kind}"}
    except Exception as e:
        return {"ok": False, "id": None, "error": str(e)}


def _redact_secrets(text: str) -> str:
    """Mask token/key-like substrings. Minimal: env var names and 20-char hex."""
    import re
    for key in ("API_KEY", "TOKEN", "SECRET", "PASSWORD", "Bearer "):
        if key in text:
            text = re.sub(rf"({key}\s*[:=]\s*)[^\s,}}\]]+", r"\1***REDACTED***", text, flags=re.IGNORECASE)
    return text


def store_plan(goal: str, plan: Dict[str, Any], task_id: Optional[str] = None, thread_id: Optional[str] = None) -> str:
    task_id = task_id or new_id()
    thread_id = ensure_thread(thread_id, goal[:200])
    now = now_iso()
    conn = _ensure()
    conn.execute(
        "INSERT INTO tasks (id, thread_id, goal, plan_json, status, created_ts, updated_ts) VALUES (?, ?, ?, ?, ?, ?, ?)",
        (task_id, thread_id, goal, json.dumps(plan), "planned", now, now),
    )
    conn.commit()
    _fts_insert(thread_id, task_id, "plan", goal + " " + json.dumps(plan)[:2000])
    return task_id


def store_run(task_id: str, step_id: Optional[str], ok: bool, result: Optional[Dict] = None, error: Optional[str] = None, ms: int = 0) -> int:
    now = now_iso()
    conn = _ensure()
    conn.execute(
        "INSERT INTO runs (task_id, step_id, ok, result_json, error, ms, created_ts) VALUES (?, ?, ?, ?, ?, ?, ?)",
        (task_id, step_id or "", 1 if ok else 0, json.dumps(result) if result else None, error, ms, now),
    )
    conn.commit()
    row = conn.execute("SELECT last_insert_rowid()").fetchone()
    run_id = row[0] if row else 0
    _fts_insert(None, task_id, "run", (error or "") + (json.dumps(result) if result else "")[:2000])
    return run_id


def store_artifact(task_id: Optional[str], run_id: Optional[int], kind: str, path_or_uri: str, content_preview: str = "") -> int:
    now = now_iso()
    conn = _ensure()
    conn.execute(
        "INSERT INTO artifacts (task_id, run_id, kind, path_or_uri, content_preview, created_ts) VALUES (?, ?, ?, ?, ?, ?)",
        (task_id or "", run_id, kind, path_or_uri[:2048], content_preview[:2048], now),
    )
    conn.commit()
    row = conn.execute("SELECT last_insert_rowid()").fetchone()
    _fts_insert(None, task_id, "artifact", path_or_uri + " " + content_preview)
    return row[0] if row else 0


def store_decision(thread_id: Optional[str], task_id: Optional[str], decision_type: str, payload: Dict[str, Any]) -> int:
    now = now_iso()
    conn = _ensure()
    conn.execute(
        "INSERT INTO decisions (thread_id, task_id, decision_type, payload_json, created_ts) VALUES (?, ?, ?, ?, ?)",
        (thread_id or "", task_id or "", decision_type, json.dumps(payload), now),
    )
    conn.commit()
    row = conn.execute("SELECT last_insert_rowid()").fetchone()
    _fts_insert(thread_id, task_id, "decision", decision_type + " " + json.dumps(payload)[:1000])
    return row[0] if row else 0


def add_summary(thread_id: str, content: str) -> int:
    now = now_iso()
    conn = _ensure()
    conn.execute("INSERT INTO summaries (thread_id, content, created_ts) VALUES (?, ?, ?)", (thread_id, content[:50000], now))
    conn.commit()
    row = conn.execute("SELECT last_insert_rowid()").fetchone()
    _fts_insert(thread_id, None, "summary", content)
    return row[0] if row else 0

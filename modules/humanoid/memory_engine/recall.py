"""Recall: search by query (FTS or LIKE) and by thread."""
from __future__ import annotations

import json
from typing import Any, Dict, List, Optional

from .db import _ensure, fts_available


def recall_by_query(query: str, limit: int = 20) -> List[Dict[str, Any]]:
    """Search memory. FTS → LIKE → histórico simple por orden (fallback final)."""
    conn = _ensure()
    results: List[Dict[str, Any]] = []
    query_clean = (query or "").strip()
    if query_clean:
        if fts_available():
            try:
                rows = conn.execute(
                    "SELECT rowid, content, thread_id, task_id, kind FROM memory_fts WHERE memory_fts MATCH ? LIMIT ?",
                    (query_clean, limit),
                ).fetchall()
                for r in rows:
                    results.append({"rowid": r[0], "content": r[1][:500], "thread_id": r[2], "task_id": r[3], "kind": r[4]})
                if results:
                    return results
            except Exception:
                pass
        like = f"%{query_clean.replace('%', ' ').replace('_', ' ')}%"
        try:
            rows = conn.execute(
                "SELECT id, thread_id, goal, plan_json, status, created_ts FROM tasks WHERE goal LIKE ? OR plan_json LIKE ? ORDER BY created_ts DESC LIMIT ?",
                (like, like, limit),
            ).fetchall()
            for r in rows:
                results.append({
                    "task_id": r[0], "thread_id": r[1], "goal": r[2], "plan": json.loads(r[3]) if r[3] else {}, "status": r[4], "created_ts": r[5],
                })
            if results:
                return results
        except Exception:
            pass
    # Fallback final: histórico simple por orden (spec 1)
    try:
        rows = conn.execute(
            "SELECT id, thread_id, goal, plan_json, status, created_ts FROM tasks ORDER BY created_ts DESC LIMIT ?",
            (limit,),
        ).fetchall()
        for r in rows:
            results.append({
                "task_id": r[0], "thread_id": r[1], "goal": r[2], "plan": json.loads(r[3]) if r[3] else {}, "status": r[4], "created_ts": r[5],
            })
    except Exception:
        pass
    return results


def recall_by_thread(thread_id: str, limit: int = 50) -> Dict[str, Any]:
    """Get all tasks, runs, artifacts, summaries for a thread."""
    conn = _ensure()
    tasks = []
    for row in conn.execute(
        "SELECT id, goal, plan_json, status, created_ts FROM tasks WHERE thread_id = ? ORDER BY created_ts DESC LIMIT ?",
        (thread_id, limit),
    ).fetchall():
        tasks.append({"id": row[0], "goal": row[1], "plan": json.loads(row[2]) if row[2] else {}, "status": row[3], "created_ts": row[4]})
    runs = []
    for row in conn.execute(
        """SELECT r.id, r.task_id, r.step_id, r.ok, r.result_json, r.error, r.created_ts FROM runs r
           JOIN tasks t ON t.id = r.task_id WHERE t.thread_id = ? ORDER BY r.created_ts DESC LIMIT ?""",
        (thread_id, limit),
    ).fetchall():
        runs.append({"id": row[0], "task_id": row[1], "step_id": row[2], "ok": bool(row[3]), "result": json.loads(row[4]) if row[4] else None, "error": row[5], "created_ts": row[6]})
    artifacts = []
    for row in conn.execute(
        "SELECT a.id, a.task_id, a.kind, a.path_or_uri, a.created_ts FROM artifacts a JOIN tasks t ON t.id = a.task_id WHERE t.thread_id = ? ORDER BY a.created_ts DESC LIMIT ?",
        (thread_id, limit),
    ).fetchall():
        artifacts.append({"id": row[0], "task_id": row[1], "kind": row[2], "path_or_uri": row[3], "created_ts": row[4]})
    summaries = []
    for row in conn.execute("SELECT id, content, created_ts FROM summaries WHERE thread_id = ? ORDER BY created_ts DESC LIMIT ?", (thread_id, limit)).fetchall():
        summaries.append({"id": row[0], "content": row[1], "created_ts": row[2]})
    return {"thread_id": thread_id, "tasks": tasks, "runs": runs, "artifacts": artifacts, "summaries": summaries}


# --- RECORDATORIO: Embeddings pendiente ---
# TODO(memory): Implementar recall_by_similarity con vectores (Ollama/lib ligera).
# Schema: embedding BLOB o tabla embeddings(source_kind, source_id, vector).
# Mientras tanto: fallback a recall_by_query.


def recall_by_similarity(query: str, limit: int = 20) -> List[Dict[str, Any]]:
    """Búsqueda semántica por embedding. Pendiente: usar vectores cuando exista dependencia.
    Fallback actual: recall_by_query (FTS/LIKE)."""
    return recall_by_query(query, limit=limit)

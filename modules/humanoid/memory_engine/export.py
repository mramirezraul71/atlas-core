"""Export: snapshot and markdown export."""
from __future__ import annotations

import json
from datetime import datetime, timezone
from typing import Any, Dict, List

from .db import _ensure, now_iso


def export_markdown(thread_id: str = "", task_id: str = "", limit: int = 100) -> str:
    """Export memory as markdown. If thread_id/task_id empty, export recent."""
    conn = _ensure()
    lines = ["# ATLAS Memory Export", f"Generated: {now_iso()}", ""]
    if task_id:
        row = conn.execute("SELECT id, thread_id, goal, plan_json, status, created_ts FROM tasks WHERE id = ?", (task_id,)).fetchone()
        if row:
            lines.append(f"## Task {row[0]}")
            lines.append(f"- Goal: {row[2]}")
            lines.append(f"- Status: {row[4]}")
            lines.append(f"- Created: {row[5]}")
            if row[3]:
                lines.append("### Plan")
                lines.append("```json")
                lines.append(row[3][:3000])
                lines.append("```")
            for r in conn.execute("SELECT step_id, ok, error, result_json, created_ts FROM runs WHERE task_id = ? ORDER BY id", (task_id,)).fetchall():
                lines.append(f"- Run {r[0]}: ok={r[1]} error={r[2] or '-'} at {r[4]}")
            for a in conn.execute("SELECT kind, path_or_uri, created_ts FROM artifacts WHERE task_id = ?", (task_id,)).fetchall():
                lines.append(f"- Artifact [{a[0]}]: {a[1]} at {a[2]}")
    elif thread_id:
        from .recall import recall_by_thread
        data = recall_by_thread(thread_id, limit=limit)
        lines.append(f"## Thread {thread_id}")
        for t in data.get("tasks", [])[:20]:
            lines.append(f"- Task {t['id']}: {t['goal'][:80]} ({t['status']})")
        for s in data.get("summaries", [])[:5]:
            lines.append(f"### Summary\n{s['content'][:500]}")
    else:
        for row in conn.execute("SELECT id, thread_id, goal, status, created_ts FROM tasks ORDER BY created_ts DESC LIMIT ?", (limit,)).fetchall():
            lines.append(f"- Task {row[0]} (thread {row[1]}): {row[2][:60]} - {row[3]} at {row[4]}")
    return "\n".join(lines)


def snapshot() -> Dict[str, Any]:
    """Export full snapshot (threads, task count, run count, artifacts count) for backup."""
    conn = _ensure()
    threads = conn.execute("SELECT COUNT(*) FROM threads").fetchone()[0]
    tasks = conn.execute("SELECT COUNT(*) FROM tasks").fetchone()[0]
    runs = conn.execute("SELECT COUNT(*) FROM runs").fetchone()[0]
    artifacts = conn.execute("SELECT COUNT(*) FROM artifacts").fetchone()[0]
    return {
        "ok": True,
        "at": now_iso(),
        "counts": {"threads": threads, "tasks": tasks, "runs": runs, "artifacts": artifacts},
        "db_path": conn.execute("PRAGMA database_list").fetchone()[2] if conn else "",
    }

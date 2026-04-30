from __future__ import annotations

import json
import sqlite3
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List

REPO_ROOT = Path(__file__).resolve().parents[3]
STATE_DIR = REPO_ROOT / "state"
DB_PATH = REPO_ROOT / "data" / "autonomy_tasks.db"

RUNTIME_MODEL_PATH = STATE_DIR / "atlas_runtime_model.json"
POLICY_STATE_PATH = STATE_DIR / "atlas_autonomy_policy_state.json"
PLAN_PATH = STATE_DIR / "atlas_autonomy_plan_latest.json"
EXECUTION_PATH = STATE_DIR / "atlas_autonomy_execution_latest.json"
LATEST_PATH = STATE_DIR / "atlas_autonomy_manager_latest.json"
MEMORY_PATH = STATE_DIR / "atlas_autonomy_incident_memory.jsonl"
CONTROL_PLANE_PATH = STATE_DIR / "atlas_control_plane_state.json"


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_json(path: Path, default: Dict[str, Any] | None = None) -> Dict[str, Any]:
    if not path.exists():
        return dict(default or {})
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return dict(default or {})


def write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")


def append_memory(entry: Dict[str, Any]) -> None:
    MEMORY_PATH.parent.mkdir(parents=True, exist_ok=True)
    with MEMORY_PATH.open("a", encoding="utf-8") as fh:
        fh.write(json.dumps(entry, ensure_ascii=False) + "\n")


def read_recent_memory(limit: int = 40) -> List[Dict[str, Any]]:
    if not MEMORY_PATH.exists():
        return []
    items: List[Dict[str, Any]] = []
    try:
        with MEMORY_PATH.open("r", encoding="utf-8") as fh:
            for line in fh:
                text = (line or "").strip()
                if not text:
                    continue
                try:
                    items.append(json.loads(text))
                except Exception:
                    continue
    except Exception:
        return []
    if limit <= 0:
        return items
    return items[-limit:]


def _db() -> sqlite3.Connection:
    DB_PATH.parent.mkdir(parents=True, exist_ok=True)
    conn = sqlite3.connect(str(DB_PATH))
    conn.row_factory = sqlite3.Row
    conn.execute(
        """CREATE TABLE IF NOT EXISTS autonomy_tasks (
            id TEXT PRIMARY KEY,
            title TEXT,
            status TEXT DEFAULT 'pending',
            priority TEXT DEFAULT 'medium',
            source TEXT DEFAULT 'system',
            detail TEXT DEFAULT '',
            action_taken TEXT DEFAULT '',
            created_at TEXT DEFAULT (datetime('now')),
            updated_at TEXT DEFAULT (datetime('now'))
        )"""
    )
    conn.execute(
        """CREATE TABLE IF NOT EXISTS autonomy_timeline (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            ts TEXT DEFAULT (datetime('now')),
            event TEXT,
            kind TEXT DEFAULT 'info',
            result TEXT DEFAULT 'ok'
        )"""
    )
    conn.execute(
        """CREATE TABLE IF NOT EXISTS autonomy_manager_cycles (
            cycle_id TEXT PRIMARY KEY,
            ts TEXT DEFAULT (datetime('now')),
            trigger_mode TEXT,
            manager_status TEXT,
            policy_mode TEXT,
            incidents_total INTEGER DEFAULT 0,
            actions_planned INTEGER DEFAULT 0,
            actions_executed INTEGER DEFAULT 0,
            actions_succeeded INTEGER DEFAULT 0,
            actions_failed INTEGER DEFAULT 0,
            ai_used INTEGER DEFAULT 0,
            summary TEXT DEFAULT '',
            report_path TEXT DEFAULT ''
        )"""
    )
    conn.execute(
        """CREATE TABLE IF NOT EXISTS autonomy_manager_actions (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            cycle_id TEXT,
            action_id TEXT,
            action_index INTEGER DEFAULT 0,
            kind TEXT DEFAULT '',
            target TEXT DEFAULT '',
            risk TEXT DEFAULT '',
            status TEXT DEFAULT 'pending',
            started_at TEXT,
            finished_at TEXT,
            result_json TEXT DEFAULT ''
        )"""
    )
    conn.commit()
    return conn


def timeline(event: str, kind: str = "info", result: str = "ok") -> None:
    conn = _db()
    try:
        conn.execute(
            "INSERT INTO autonomy_timeline(event, kind, result) VALUES(?,?,?)",
            ((event or "")[:500], (kind or "info")[:40], (result or "ok")[:40]),
        )
        conn.commit()
    finally:
        conn.close()


def upsert_task(
    *,
    task_id: str,
    title: str,
    status: str,
    priority: str,
    detail: str,
    action_taken: str = "",
    source: str = "autonomy_manager",
) -> None:
    conn = _db()
    try:
        conn.execute(
            """INSERT INTO autonomy_tasks(id, title, status, priority, source, detail, action_taken, created_at, updated_at)
               VALUES(?,?,?,?,?,?,?, datetime('now'), datetime('now'))
               ON CONFLICT(id) DO UPDATE SET
                   title=excluded.title,
                   status=excluded.status,
                   priority=excluded.priority,
                   source=excluded.source,
                   detail=excluded.detail,
                   action_taken=excluded.action_taken,
                   updated_at=datetime('now')
            """,
            (
                task_id[:120],
                title[:180],
                status[:40],
                priority[:20],
                source[:60],
                detail[:1400],
                action_taken[:240],
            ),
        )
        conn.commit()
    finally:
        conn.close()


def record_cycle(report: Dict[str, Any]) -> None:
    conn = _db()
    try:
        conn.execute(
            """INSERT OR REPLACE INTO autonomy_manager_cycles(
                cycle_id, ts, trigger_mode, manager_status, policy_mode, incidents_total,
                actions_planned, actions_executed, actions_succeeded, actions_failed,
                ai_used, summary, report_path
            ) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?)""",
            (
                str(report.get("cycle_id") or "")[:80],
                str(report.get("generated_at") or now_iso()),
                str(report.get("trigger_mode") or "")[:40],
                str(report.get("manager_status") or "")[:40],
                str(((report.get("policy") or {}).get("mode")) or "")[:40],
                int(((report.get("before") or {}).get("event_count")) or 0),
                int(report.get("actions_planned") or 0),
                int(report.get("actions_executed") or 0),
                int(report.get("actions_succeeded") or 0),
                int(report.get("actions_failed") or 0),
                1 if ((report.get("plan") or {}).get("ai_analysis") or {}).get("used") else 0,
                str(report.get("summary") or "")[:1500],
                str(report.get("artifacts", {}).get("latest") or "")[:260],
            ),
        )
        conn.commit()
    finally:
        conn.close()


def record_action(cycle_id: str, action: Dict[str, Any], result: Dict[str, Any]) -> None:
    conn = _db()
    try:
        conn.execute(
            """INSERT INTO autonomy_manager_actions(
                cycle_id, action_id, action_index, kind, target, risk, status, started_at, finished_at, result_json
            ) VALUES(?,?,?,?,?,?,?,?,?,?)""",
            (
                cycle_id[:80],
                str(action.get("action_id") or "")[:80],
                int(action.get("index") or 0),
                str(action.get("kind") or "")[:80],
                str(action.get("target") or "")[:160],
                str(action.get("risk") or "")[:40],
                str(result.get("status") or "done")[:40],
                str(result.get("started_at") or "")[:40],
                str(result.get("finished_at") or "")[:40],
                json.dumps(result, ensure_ascii=False)[:4000],
            ),
        )
        conn.commit()
    finally:
        conn.close()


def list_recent_cycles(limit: int = 20) -> List[Dict[str, Any]]:
    conn = _db()
    try:
        rows = conn.execute(
            """SELECT cycle_id, ts, trigger_mode, manager_status, policy_mode,
                      incidents_total, actions_planned, actions_executed,
                      actions_succeeded, actions_failed, ai_used, summary
               FROM autonomy_manager_cycles
               ORDER BY ts DESC
               LIMIT ?""",
            (max(1, min(int(limit or 20), 100)),),
        ).fetchall()
        return [dict(row) for row in rows]
    finally:
        conn.close()


def list_recent_actions(limit: int = 30) -> List[Dict[str, Any]]:
    conn = _db()
    try:
        rows = conn.execute(
            """SELECT cycle_id, action_id, action_index, kind, target, risk, status, started_at, finished_at, result_json
               FROM autonomy_manager_actions
               ORDER BY id DESC
               LIMIT ?""",
            (max(1, min(int(limit or 30), 200)),),
        ).fetchall()
        items: List[Dict[str, Any]] = []
        for row in rows:
            item = dict(row)
            try:
                item["result"] = json.loads(item.pop("result_json") or "{}")
            except Exception:
                item["result"] = {}
            items.append(item)
        return items
    finally:
        conn.close()

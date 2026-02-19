"""
GoalContext: Contexto persistente de ejecucion por objetivo.

Cada goal tiene su propio contexto completo (plan, paso actual, resultados
intermedios) persistido en SQLite para sobrevivir reinicios y permitir
cambio de foco sin perdida de informacion.
"""
from __future__ import annotations

import json
import logging
import os
import sqlite3
import time
import uuid
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional

_log = logging.getLogger("humanoid.cortex.frontal.goal_context")

_DB_PATH = os.getenv("CGE_DB_PATH", os.path.join("logs", "atlas_cge.sqlite"))


class CGEGoalStatus(Enum):
    PENDING = "pending"
    PLANNING = "planning"
    ACTIVE = "active"
    PAUSED = "paused"
    WAITING_RESOURCE = "waiting_resource"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


class CGEGoalPriority(Enum):
    LOW = 0
    NORMAL = 1
    HIGH = 2
    URGENT = 3
    CRITICAL = 4


@dataclass
class GoalContext:
    """Contexto completo de ejecucion de un goal concurrente."""
    goal_id: str
    goal_type: str
    description: str
    priority: CGEGoalPriority = CGEGoalPriority.NORMAL
    status: CGEGoalStatus = CGEGoalStatus.PENDING

    plan_json: Optional[str] = None
    step_index: int = 0
    progress: float = 0.0
    context_data: Dict[str, Any] = field(default_factory=dict)
    resources_needed: List[str] = field(default_factory=list)
    resources_held: List[str] = field(default_factory=list)

    parent_goal_id: Optional[str] = None
    source: str = "user"
    error: Optional[str] = None

    created_ts: float = field(default_factory=time.time)
    updated_ts: float = field(default_factory=time.time)
    deadline_ts: Optional[float] = None

    def is_terminal(self) -> bool:
        return self.status in (
            CGEGoalStatus.COMPLETED,
            CGEGoalStatus.FAILED,
            CGEGoalStatus.CANCELLED,
        )

    def is_runnable(self) -> bool:
        return self.status in (CGEGoalStatus.ACTIVE, CGEGoalStatus.PLANNING)

    def elapsed_s(self) -> float:
        return time.time() - self.created_ts

    def is_overdue(self) -> bool:
        if self.deadline_ts and time.time() > self.deadline_ts:
            return True
        return False

    def to_dict(self) -> Dict[str, Any]:
        return {
            "goal_id": self.goal_id,
            "goal_type": self.goal_type,
            "description": self.description,
            "priority": self.priority.name,
            "status": self.status.value,
            "step_index": self.step_index,
            "progress": self.progress,
            "resources_needed": self.resources_needed,
            "resources_held": self.resources_held,
            "parent_goal_id": self.parent_goal_id,
            "source": self.source,
            "error": self.error,
            "created_ts": self.created_ts,
            "updated_ts": self.updated_ts,
            "deadline_ts": self.deadline_ts,
            "elapsed_s": round(self.elapsed_s(), 2),
        }


def new_goal_id() -> str:
    return f"cge_{uuid.uuid4().hex[:12]}"


# ---------------------------------------------------------------------------
# SQLite persistence
# ---------------------------------------------------------------------------

_SCHEMA = """
CREATE TABLE IF NOT EXISTS goal_contexts (
    goal_id TEXT PRIMARY KEY,
    goal_type TEXT NOT NULL,
    description TEXT NOT NULL,
    priority INTEGER DEFAULT 1,
    status TEXT DEFAULT 'pending',
    plan_json TEXT,
    step_index INTEGER DEFAULT 0,
    progress REAL DEFAULT 0.0,
    context_json TEXT,
    resources_needed TEXT,
    resources_held TEXT,
    parent_goal_id TEXT,
    source TEXT DEFAULT 'user',
    error TEXT,
    created_ts REAL,
    updated_ts REAL,
    deadline_ts REAL
);

CREATE TABLE IF NOT EXISTS goal_exec_log (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    goal_id TEXT NOT NULL,
    step_index INTEGER,
    action TEXT,
    result_json TEXT,
    ok INTEGER DEFAULT 0,
    ms INTEGER DEFAULT 0,
    ts REAL DEFAULT (strftime('%s','now')),
    FOREIGN KEY (goal_id) REFERENCES goal_contexts(goal_id)
);

CREATE TABLE IF NOT EXISTS resource_ledger (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    resource_type TEXT NOT NULL,
    goal_id TEXT NOT NULL,
    priority INTEGER DEFAULT 1,
    allocated_ts REAL DEFAULT (strftime('%s','now')),
    released_ts REAL
);
"""


class GoalContextDB:
    """Capa de persistencia SQLite para GoalContext."""

    def __init__(self, db_path: Optional[str] = None):
        self._path = db_path or _DB_PATH
        os.makedirs(os.path.dirname(self._path) or ".", exist_ok=True)
        self._init_db()

    def _conn(self) -> sqlite3.Connection:
        conn = sqlite3.connect(self._path, timeout=5)
        conn.execute("PRAGMA journal_mode=WAL")
        conn.execute("PRAGMA busy_timeout=3000")
        conn.row_factory = sqlite3.Row
        return conn

    def _init_db(self) -> None:
        try:
            with self._conn() as conn:
                conn.executescript(_SCHEMA)
        except Exception as exc:
            _log.error("CGE DB init failed: %s", exc)

    # -- CRUD ---------------------------------------------------------------

    def save(self, ctx: GoalContext) -> None:
        ctx.updated_ts = time.time()
        sql = """
        INSERT OR REPLACE INTO goal_contexts
            (goal_id, goal_type, description, priority, status,
             plan_json, step_index, progress, context_json,
             resources_needed, resources_held, parent_goal_id,
             source, error, created_ts, updated_ts, deadline_ts)
        VALUES (?,?,?,?,?, ?,?,?,?, ?,?,?, ?,?,?,?,?)
        """
        vals = (
            ctx.goal_id, ctx.goal_type, ctx.description,
            ctx.priority.value, ctx.status.value,
            ctx.plan_json, ctx.step_index, ctx.progress,
            json.dumps(ctx.context_data, default=str),
            json.dumps(ctx.resources_needed),
            json.dumps(ctx.resources_held),
            ctx.parent_goal_id, ctx.source, ctx.error,
            ctx.created_ts, ctx.updated_ts, ctx.deadline_ts,
        )
        try:
            with self._conn() as conn:
                conn.execute(sql, vals)
        except Exception as exc:
            _log.error("save goal_context %s failed: %s", ctx.goal_id, exc)

    def load(self, goal_id: str) -> Optional[GoalContext]:
        sql = "SELECT * FROM goal_contexts WHERE goal_id = ?"
        try:
            with self._conn() as conn:
                row = conn.execute(sql, (goal_id,)).fetchone()
            if not row:
                return None
            return self._row_to_ctx(row)
        except Exception as exc:
            _log.error("load goal_context %s failed: %s", goal_id, exc)
            return None

    def list_active(self) -> List[GoalContext]:
        sql = "SELECT * FROM goal_contexts WHERE status NOT IN ('completed','failed','cancelled') ORDER BY priority DESC, created_ts ASC"
        try:
            with self._conn() as conn:
                rows = conn.execute(sql).fetchall()
            return [self._row_to_ctx(r) for r in rows]
        except Exception as exc:
            _log.error("list_active failed: %s", exc)
            return []

    def list_all(self, limit: int = 50) -> List[GoalContext]:
        sql = "SELECT * FROM goal_contexts ORDER BY updated_ts DESC LIMIT ?"
        try:
            with self._conn() as conn:
                rows = conn.execute(sql, (limit,)).fetchall()
            return [self._row_to_ctx(r) for r in rows]
        except Exception as exc:
            _log.error("list_all failed: %s", exc)
            return []

    def update_status(self, goal_id: str, status: CGEGoalStatus,
                      error: Optional[str] = None) -> None:
        sql = "UPDATE goal_contexts SET status=?, error=?, updated_ts=? WHERE goal_id=?"
        try:
            with self._conn() as conn:
                conn.execute(sql, (status.value, error, time.time(), goal_id))
        except Exception as exc:
            _log.error("update_status %s failed: %s", goal_id, exc)

    def update_progress(self, goal_id: str, step_index: int,
                        progress: float, plan_json: Optional[str] = None) -> None:
        if plan_json is not None:
            sql = "UPDATE goal_contexts SET step_index=?, progress=?, plan_json=?, updated_ts=? WHERE goal_id=?"
            params = (step_index, progress, plan_json, time.time(), goal_id)
        else:
            sql = "UPDATE goal_contexts SET step_index=?, progress=?, updated_ts=? WHERE goal_id=?"
            params = (step_index, progress, time.time(), goal_id)
        try:
            with self._conn() as conn:
                conn.execute(sql, params)
        except Exception as exc:
            _log.error("update_progress %s failed: %s", goal_id, exc)

    # -- Exec log -----------------------------------------------------------

    def log_step(self, goal_id: str, step_index: int, action: str,
                 result: Any, ok: bool, ms: int) -> None:
        sql = """INSERT INTO goal_exec_log (goal_id, step_index, action, result_json, ok, ms, ts)
                 VALUES (?,?,?,?,?,?,?)"""
        try:
            with self._conn() as conn:
                conn.execute(sql, (
                    goal_id, step_index, action,
                    json.dumps(result, default=str) if result else None,
                    1 if ok else 0, ms, time.time(),
                ))
        except Exception as exc:
            _log.error("log_step %s failed: %s", goal_id, exc)

    def get_exec_log(self, goal_id: str, limit: int = 100) -> List[Dict[str, Any]]:
        sql = "SELECT * FROM goal_exec_log WHERE goal_id=? ORDER BY ts DESC LIMIT ?"
        try:
            with self._conn() as conn:
                rows = conn.execute(sql, (goal_id, limit)).fetchall()
            return [dict(r) for r in rows]
        except Exception:
            return []

    # -- Resource ledger ----------------------------------------------------

    def log_resource_alloc(self, resource_type: str, goal_id: str,
                           priority: int) -> None:
        sql = "INSERT INTO resource_ledger (resource_type, goal_id, priority, allocated_ts) VALUES (?,?,?,?)"
        try:
            with self._conn() as conn:
                conn.execute(sql, (resource_type, goal_id, priority, time.time()))
        except Exception:
            pass

    def log_resource_release(self, resource_type: str, goal_id: str) -> None:
        sql = """UPDATE resource_ledger SET released_ts=?
                 WHERE resource_type=? AND goal_id=? AND released_ts IS NULL"""
        try:
            with self._conn() as conn:
                conn.execute(sql, (time.time(), resource_type, goal_id))
        except Exception:
            pass

    # -- Cleanup ------------------------------------------------------------

    def cleanup(self, max_age_hours: int = 72) -> int:
        cutoff = time.time() - (max_age_hours * 3600)
        sql = "DELETE FROM goal_contexts WHERE status IN ('completed','failed','cancelled') AND updated_ts < ?"
        try:
            with self._conn() as conn:
                cur = conn.execute(sql, (cutoff,))
                n = cur.rowcount
                conn.execute("DELETE FROM goal_exec_log WHERE goal_id NOT IN (SELECT goal_id FROM goal_contexts)")
                conn.execute("DELETE FROM resource_ledger WHERE goal_id NOT IN (SELECT goal_id FROM goal_contexts)")
                return n
        except Exception as exc:
            _log.error("cleanup failed: %s", exc)
            return 0

    # -- Helpers ------------------------------------------------------------

    @staticmethod
    def _row_to_ctx(row: sqlite3.Row) -> GoalContext:
        def _json_list(val: Any) -> list:
            if not val:
                return []
            try:
                return json.loads(val)
            except Exception:
                return []

        def _json_dict(val: Any) -> dict:
            if not val:
                return {}
            try:
                return json.loads(val)
            except Exception:
                return {}

        return GoalContext(
            goal_id=row["goal_id"],
            goal_type=row["goal_type"],
            description=row["description"],
            priority=CGEGoalPriority(row["priority"]),
            status=CGEGoalStatus(row["status"]),
            plan_json=row["plan_json"],
            step_index=row["step_index"] or 0,
            progress=row["progress"] or 0.0,
            context_data=_json_dict(row["context_json"]),
            resources_needed=_json_list(row["resources_needed"]),
            resources_held=_json_list(row["resources_held"]),
            parent_goal_id=row["parent_goal_id"],
            source=row["source"] or "user",
            error=row["error"],
            created_ts=row["created_ts"] or time.time(),
            updated_ts=row["updated_ts"] or time.time(),
            deadline_ts=row["deadline_ts"],
        )

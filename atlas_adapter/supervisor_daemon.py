from __future__ import annotations

import asyncio
import json
import os
import re
import sqlite3
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional


def _repo_root() -> Path:
    return Path(__file__).resolve().parent.parent


_AUTO_DB = (_repo_root() / "data" / "autonomy_tasks.db").resolve()
_AUTO_DB.parent.mkdir(parents=True, exist_ok=True)
_LOCK_PATH = (_repo_root() / "logs" / "supervisor_daemon.lock").resolve()


def _auto_db() -> sqlite3.Connection:
    conn = sqlite3.connect(str(_AUTO_DB))
    conn.row_factory = sqlite3.Row
    conn.execute(
        """CREATE TABLE IF NOT EXISTS autonomy_tasks (
            id TEXT PRIMARY KEY, title TEXT, status TEXT DEFAULT 'pending',
            priority TEXT DEFAULT 'medium', source TEXT DEFAULT 'system',
            detail TEXT DEFAULT '', action_taken TEXT DEFAULT '',
            created_at TEXT DEFAULT (datetime('now')), updated_at TEXT DEFAULT (datetime('now'))
        )"""
    )
    conn.execute(
        """CREATE TABLE IF NOT EXISTS autonomy_timeline (
            id INTEGER PRIMARY KEY AUTOINCREMENT, ts TEXT DEFAULT (datetime('now')),
            event TEXT, kind TEXT DEFAULT 'info', result TEXT DEFAULT 'ok'
        )"""
    )
    conn.commit()
    return conn


def _task_priority_for_severity(sev: str) -> str:
    s = (sev or "").strip().lower()
    if s == "critical":
        return "critical"
    if s == "warning":
        return "high"
    return "medium"


def _bitacora_log(message: str, ok: bool, source: str = "supervisor") -> None:
    try:
        from modules.humanoid.ans.evolution_bitacora import \
            append_evolution_log

        append_evolution_log(message=message, ok=ok, source=source)
    except Exception:
        pass


def _normalize_text_for_signature(text: str) -> str:
    """Reduce ruido dinámico para evitar spam por cambios triviales."""
    t = (text or "").strip().lower()
    if not t:
        return ""
    t = re.sub(r"0x[0-9a-f]+", "<hex>", t)
    t = re.sub(r"\b\d+\b", "<n>", t)
    t = re.sub(r"\s+", " ", t)
    return t[:240]


def _insert_directive_task(title: str, detail: str, severity: str) -> str:
    tid = f"sup_{int(time.time() * 1000)}_{uuid.uuid4().hex[:8]}"
    prio = _task_priority_for_severity(severity)
    c = _auto_db()
    try:
        c.execute(
            """INSERT OR REPLACE INTO autonomy_tasks
               (id,title,status,priority,source,detail,action_taken,updated_at)
               VALUES(?,?,?,?,?,?,?,datetime('now'))""",
            (
                tid,
                title[:180],
                "pending",
                prio,
                "supervisor",
                detail[:1200],
                "",
            ),
        )
        c.commit()
    finally:
        c.close()
    return tid


def list_supervisor_directives(
    status: Optional[str] = None, limit: int = 50
) -> List[Dict[str, Any]]:
    lim = max(1, min(200, int(limit or 50)))
    st = (status or "").strip().lower()
    c = _auto_db()
    try:
        if st:
            rows = c.execute(
                """SELECT * FROM autonomy_tasks
                   WHERE source='supervisor' AND lower(status)=?
                   ORDER BY CASE priority WHEN 'critical' THEN 0 WHEN 'high' THEN 1 WHEN 'medium' THEN 2 ELSE 3 END,
                            created_at DESC
                   LIMIT ?""",
                (st, lim),
            ).fetchall()
        else:
            rows = c.execute(
                """SELECT * FROM autonomy_tasks
                   WHERE source='supervisor'
                   ORDER BY CASE priority WHEN 'critical' THEN 0 WHEN 'high' THEN 1 WHEN 'medium' THEN 2 ELSE 3 END,
                            created_at DESC
                   LIMIT ?""",
                (lim,),
            ).fetchall()
        return [dict(r) for r in (rows or [])]
    finally:
        c.close()


@dataclass
class SupervisorDaemonStatus:
    enabled: bool
    running: bool
    interval_sec: int
    last_tick_ts: float
    last_severity: str
    last_signature: str
    last_notice_ts: float
    last_task_id: str


class SupervisorDaemon:
    def __init__(self) -> None:
        self.enabled: bool = os.getenv(
            "SUPERVISOR_DAEMON_ENABLED", "true"
        ).strip().lower() in (
            "1",
            "true",
            "yes",
            "y",
            "on",
        )
        self.interval_sec: int = max(
            10, int(os.getenv("SUPERVISOR_DAEMON_INTERVAL_SEC", "30") or "30")
        )
        self.notice_every_sec: int = max(
            30, int(os.getenv("SUPERVISOR_DAEMON_NOTICE_EVERY_SEC", "600") or "600")
        )
        self.min_change_notice_sec: int = max(
            30,
            int(
                os.getenv(
                    "SUPERVISOR_DAEMON_MIN_CHANGE_NOTICE_SEC", "120"
                )
                or "120"
            ),
        )
        self.llm_enabled: bool = os.getenv(
            "SUPERVISOR_DAEMON_LLM_ENABLED", "true"
        ).strip().lower() in (
            "1",
            "true",
            "yes",
            "y",
            "on",
        )
        self.llm_min_interval_sec: int = max(
            60, int(os.getenv("SUPERVISOR_DAEMON_LLM_MIN_INTERVAL_SEC", "900") or "900")
        )
        self.telegram_enabled: bool = os.getenv(
            "SUPERVISOR_DAEMON_TELEGRAM", "true"
        ).strip().lower() in (
            "1",
            "true",
            "yes",
            "y",
            "on",
        )
        self.min_task_interval_sec: int = max(
            60,
            int(
                os.getenv(
                    "SUPERVISOR_DAEMON_MIN_TASK_INTERVAL_SEC", "900"
                )
                or "900"
            ),
        )
        self.min_telegram_interval_sec: int = max(
            60,
            int(
                os.getenv(
                    "SUPERVISOR_DAEMON_MIN_TELEGRAM_INTERVAL_SEC", "900"
                )
                or "900"
            ),
        )

        self._task: Optional[asyncio.Task] = None
        self._stop: Optional[asyncio.Event] = None
        self._running: bool = False
        self._lock_handle = None

        self._last_tick_ts: float = 0.0
        self._last_severity: str = "unknown"
        self._last_signature: str = ""
        self._last_notice_ts: float = 0.0
        self._last_task_id: str = ""
        self._last_llm_ts: float = 0.0
        self._last_task_ts: float = 0.0
        self._last_task_signature: str = ""
        self._last_telegram_ts: float = 0.0
        self._last_telegram_signature: str = ""

        self._sup = None

    def _acquire_singleton_lock(self) -> bool:
        """Evita múltiples daemons en procesos distintos (causa de spam)."""
        _LOCK_PATH.parent.mkdir(parents=True, exist_ok=True)
        fh = open(_LOCK_PATH, "a+", encoding="utf-8")
        try:
            fh.seek(0)
            if os.name == "nt":
                import msvcrt

                msvcrt.locking(fh.fileno(), msvcrt.LK_NBLCK, 1)
            else:
                import fcntl

                fcntl.flock(fh.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            fh.seek(0)
            fh.truncate()
            fh.write(str(os.getpid()))
            fh.flush()
            self._lock_handle = fh
            return True
        except Exception:
            try:
                fh.close()
            except Exception:
                pass
            return False

    def _release_singleton_lock(self) -> None:
        fh = self._lock_handle
        self._lock_handle = None
        if not fh:
            return
        try:
            fh.seek(0)
            if os.name == "nt":
                import msvcrt

                msvcrt.locking(fh.fileno(), msvcrt.LK_UNLCK, 1)
            else:
                import fcntl

                fcntl.flock(fh.fileno(), fcntl.LOCK_UN)
        except Exception:
            pass
        finally:
            try:
                fh.close()
            except Exception:
                pass

    def status(self) -> Dict[str, Any]:
        st = SupervisorDaemonStatus(
            enabled=bool(self.enabled),
            running=bool(self._running),
            interval_sec=int(self.interval_sec),
            last_tick_ts=float(self._last_tick_ts),
            last_severity=str(self._last_severity),
            last_signature=str(self._last_signature),
            last_notice_ts=float(self._last_notice_ts),
            last_task_id=str(self._last_task_id),
        )
        return {"ok": True, "data": st.__dict__}

    async def start(self) -> None:
        if self._running:
            return
        if not self._acquire_singleton_lock():
            _bitacora_log(
                "[SUPERVISOR] Daemon omitido: otro proceso ya mantiene el lock.",
                ok=True,
                source="supervisor",
            )
            return
        self._stop = asyncio.Event()
        self._task = asyncio.create_task(self._run(), name="atlas-supervisor-daemon")
        self._running = True
        _bitacora_log("[SUPERVISOR] Daemon iniciado.", ok=True, source="supervisor")

    async def stop(self) -> None:
        self.enabled = False
        if self._stop:
            self._stop.set()
        if self._task:
            try:
                await asyncio.wait_for(self._task, timeout=3)
            except Exception:
                pass
        self._running = False
        self._release_singleton_lock()
        _bitacora_log("[SUPERVISOR] Daemon detenido.", ok=True, source="supervisor")

    async def _run(self) -> None:
        assert self._stop is not None
        while not self._stop.is_set():
            try:
                if self.enabled:
                    await self._tick()
            except Exception:
                # Never crash the API because of the supervisor daemon.
                pass
            try:
                await asyncio.wait_for(
                    self._stop.wait(), timeout=float(self.interval_sec)
                )
            except asyncio.TimeoutError:
                pass

    async def _tick(self) -> None:
        now = time.time()
        self._last_tick_ts = now

        if self._sup is None:
            from atlas_adapter.llm.supervisor import \
                Supervisor as LlmSupervisor

            self._sup = LlmSupervisor()

        sup = self._sup

        snap = await asyncio.to_thread(sup._gather_system_snapshot)  # type: ignore[attr-defined]
        diag = await asyncio.to_thread(sup._diagnose, snap)  # type: ignore[attr-defined]

        severity = str(diag.get("severity") or "unknown").strip().lower()
        issues = diag.get("issues") or []
        warnings = diag.get("warnings") or []

        sig_obj = {"severity": severity, "issues": issues[:8], "warnings": warnings[:8]}
        signature = json.dumps(sig_obj, ensure_ascii=False, sort_keys=True)

        changed = signature != self._last_signature
        elapsed_notice = now - self._last_notice_ts
        due_heartbeat = elapsed_notice >= float(self.notice_every_sec)
        can_emit_change = elapsed_notice >= float(self.min_change_notice_sec)

        if (changed and can_emit_change) or due_heartbeat:
            self._last_signature = signature
            self._last_severity = severity
            self._last_notice_ts = now

            if severity == "healthy":
                _bitacora_log(
                    "[SUPERVISOR] OK: sistema estable (sin hallazgos críticos).",
                    ok=True,
                    source="supervisor",
                )
                return

            top = []
            for i in issues[:3] if isinstance(issues, list) else []:
                top.append(str(i)[:220])
            for w in warnings[:2] if isinstance(warnings, list) else []:
                top.append(str(w)[:220])
            top_txt = " | ".join(top) if top else "Hallazgos detectados"
            ok_flag = severity not in ("critical",)
            _bitacora_log(
                f"[SUPERVISOR] {severity.upper()}: {top_txt}",
                ok=ok_flag,
                source="supervisor",
            )

            # Create directive task for the Agent/Owner to action (internal queue)
            detail = "DIAGNÓSTICO (snapshot+reglas):\n"
            if isinstance(issues, list) and issues:
                detail += (
                    "\n".join(f"- ISSUE: {str(x)[:260]}" for x in issues[:8]) + "\n"
                )
            if isinstance(warnings, list) and warnings:
                detail += (
                    "\n".join(f"- WARN: {str(x)[:260]}" for x in warnings[:8]) + "\n"
                )
            detail += "\nSUGERENCIA DE PROMPT PARA EL AGENTE:\n"
            detail += "Investiga la causa raíz de los ISSUES/WARN anteriores con evidencia. Propón acciones concretas y seguras.\n"

            # Optionally enrich task with LLM recommendations (rate-limited)
            if self.llm_enabled and (now - self._last_llm_ts) >= float(
                self.llm_min_interval_sec
            ):
                self._last_llm_ts = now
                try:
                    obj = f"Monitoreo automático: resolver severidad {severity}"
                    # Inject resident policy (best-effort) so daemon advice follows Owner governance.
                    pol = ""
                    try:
                        from atlas_adapter.supervisor_policy import \
                            get_supervisor_policy

                        pj = get_supervisor_policy()
                        if isinstance(pj, dict) and pj.get("ok"):
                            pol = str(pj.get("policy") or "").strip()
                    except Exception:
                        pol = ""
                    ctx = {"issues": issues[:8], "modules": []}
                    if pol:
                        ctx["_resident_policy"] = pol
                    llm_res = await asyncio.to_thread(sup.advise, obj, ctx)
                    analysis = ""
                    actions = []
                    if isinstance(llm_res, dict) and llm_res.get("ok"):
                        analysis = str(llm_res.get("analysis") or "")
                        actions = llm_res.get("actions") or []
                    if analysis:
                        detail += (
                            "\nRECOMENDACIÓN SUPERVISOR (LLM):\n"
                            + analysis[:1200]
                            + "\n"
                        )
                    if isinstance(actions, list) and actions:
                        detail += "\nACCIONES EXTRAÍDAS:\n"
                        for a in actions[:5]:
                            desc = str((a or {}).get("description") or "")[:180]
                            risk = str((a or {}).get("risk") or "low")
                            exe = str((a or {}).get("execute") or "")[:220]
                            if desc:
                                detail += f"- ACTION: {desc} | RISK: {risk} | EXECUTE: {exe}\n"
                except Exception:
                    pass

            task_sig = f"{severity}|{_normalize_text_for_signature(top_txt)}"
            can_emit_task = (
                (now - self._last_task_ts) >= float(self.min_task_interval_sec)
            )
            if can_emit_task and task_sig != self._last_task_signature:
                title = f"Supervisor: {severity} ({len(issues) if isinstance(issues, list) else 0} issues)"
                self._last_task_id = _insert_directive_task(
                    title=title, detail=detail, severity=severity
                )
                self._last_task_ts = now
                self._last_task_signature = task_sig

            # Optional Telegram for critical only (best-effort)
            if self.telegram_enabled and severity == "critical":
                tg_sig = f"{severity}|{_normalize_text_for_signature(top_txt)}"
                can_emit_tg = (
                    (now - self._last_telegram_ts)
                    >= float(self.min_telegram_interval_sec)
                )
                if can_emit_tg and tg_sig != self._last_telegram_signature:
                    try:
                        from modules.humanoid.notify import send_telegram

                        await send_telegram(f"ATLAS SUPERVISOR CRITICAL\n{top_txt}")
                        self._last_telegram_ts = now
                        self._last_telegram_signature = tg_sig
                    except Exception:
                        pass


_SINGLETON: Optional[SupervisorDaemon] = None


def get_supervisor_daemon() -> SupervisorDaemon:
    global _SINGLETON
    if _SINGLETON is None:
        _SINGLETON = SupervisorDaemon()
    return _SINGLETON


async def start_supervisor_daemon() -> None:
    d = get_supervisor_daemon()
    if not d.enabled:
        return
    await d.start()


async def stop_supervisor_daemon() -> None:
    d = get_supervisor_daemon()
    await d.stop()

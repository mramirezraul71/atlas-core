from __future__ import annotations

import os
import threading
from dataclasses import asdict, dataclass
from datetime import datetime, timedelta, timezone
from typing import Any, Dict, Optional

from . import storage
from .manager import run_cycle


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


@dataclass
class AutonomyManagerDaemonStatus:
    enabled: bool
    running: bool
    interval_sec: int
    next_run_at: str
    last_cycle_id: str
    last_run_at: str
    last_status: str
    last_error: str
    cycles_run: int
    current_phase: str
    current_action: str


class AutonomyManagerDaemon:
    def __init__(self) -> None:
        self.enabled = (
            os.getenv("ATLAS_AUTONOMY_MANAGER_ENABLED", "true").strip().lower()
            in {"1", "true", "yes", "y", "on"}
        )
        self.interval_sec = max(
            30,
            int(os.getenv("ATLAS_AUTONOMY_MANAGER_INTERVAL_SEC", "180") or "180"),
        )
        self.use_ai = (
            os.getenv("ATLAS_AUTONOMY_MANAGER_AI_ENABLED", "true").strip().lower()
            in {"1", "true", "yes", "y", "on"}
        )
        self.emit = True
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self.next_run_at = ""
        self.last_cycle_id = ""
        self.last_run_at = ""
        self.last_status = "idle"
        self.last_error = ""
        self.cycles_run = 0
        self.current_phase = "idle"
        self.current_action = ""

    def status(self) -> Dict[str, Any]:
        return {
            "ok": True,
            "data": asdict(
                AutonomyManagerDaemonStatus(
                    enabled=self.enabled,
                    running=self._thread is not None and self._thread.is_alive(),
                    interval_sec=self.interval_sec,
                    next_run_at=self.next_run_at,
                    last_cycle_id=self.last_cycle_id,
                    last_run_at=self.last_run_at,
                    last_status=self.last_status,
                    last_error=self.last_error,
                    cycles_run=self.cycles_run,
                    current_phase=self.current_phase,
                    current_action=self.current_action,
                )
            ),
            "latest": storage.read_json(storage.LATEST_PATH, default={}),
        }

    def _loop(self) -> None:
        storage.timeline("Autonomy Manager daemon started", kind="info", result="ok")
        run_immediately = True
        while not self._stop.is_set():
            if run_immediately:
                run_immediately = False
            else:
                self.next_run_at = (
                    datetime.now(timezone.utc) + timedelta(seconds=self.interval_sec)
                ).isoformat()
                self.current_phase = "sleeping"
                self.current_action = f"next cycle in {self.interval_sec}s"
                if self._stop.wait(self.interval_sec):
                    break
            with self._lock:
                try:
                    self.current_phase = "running"
                    self.current_action = "building runtime model"
                    self.last_status = "running"
                    self.last_error = ""
                    report = run_cycle(
                        trigger_mode="daemon", emit=self.emit, use_ai=self.use_ai
                    )
                    self.last_cycle_id = str(report.get("cycle_id") or "")
                    self.last_run_at = str(report.get("generated_at") or _now_iso())
                    self.last_status = str(report.get("manager_status") or "unknown")
                    self.last_error = ""
                    self.cycles_run += 1
                    self.current_phase = "idle"
                    self.current_action = ""
                except Exception as exc:
                    self.last_run_at = _now_iso()
                    self.last_status = "failed"
                    self.last_error = str(exc)
                    self.current_phase = "failed"
                    self.current_action = str(exc)
                    storage.timeline(
                        f"Autonomy Manager daemon error: {exc}",
                        kind="error",
                        result="failed",
                    )
            self.next_run_at = (
                datetime.now(timezone.utc) + timedelta(seconds=self.interval_sec)
            ).isoformat()
        self.current_phase = "stopped"
        self.current_action = ""
        self.next_run_at = ""
        storage.timeline("Autonomy Manager daemon stopped", kind="info", result="ok")

    def start(self) -> Dict[str, Any]:
        if not self.enabled:
            return {"ok": False, "error": "daemon_disabled"}
        if self._thread is not None and self._thread.is_alive():
            return self.status()
        self._stop.clear()
        self.current_phase = "starting"
        self.current_action = "initial cycle pending"
        self.last_status = "starting"
        self.last_error = ""
        self._thread = threading.Thread(
            target=self._loop, name="atlas-autonomy-manager", daemon=True
        )
        self._thread.start()
        return self.status()

    def stop(self) -> Dict[str, Any]:
        self._stop.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=3)
        return self.status()


_DAEMON: Optional[AutonomyManagerDaemon] = None


def get_autonomy_manager_daemon() -> AutonomyManagerDaemon:
    global _DAEMON
    if _DAEMON is None:
        _DAEMON = AutonomyManagerDaemon()
    return _DAEMON

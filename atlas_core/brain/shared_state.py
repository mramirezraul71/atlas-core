"""Thread/process-safe shared JSON state access with atomic writes."""
from __future__ import annotations

import json
import logging
import os
import tempfile
import threading
import time
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Iterator

from .audit_log import AuditLog

logger = logging.getLogger("atlas.brain.shared_state")


class SharedStateStore:
    def __init__(self, path: Path, audit_log: AuditLog | None = None) -> None:
        self.path = path
        self.lock_path = Path(str(path) + ".lock")
        self._thread_lock = threading.RLock()
        self._audit = audit_log

    @contextmanager
    def _file_lock(self, timeout_sec: float = 2.0) -> Iterator[None]:
        """Best-effort Windows file lock with safe fallback."""
        self.lock_path.parent.mkdir(parents=True, exist_ok=True)
        started = time.time()
        fh = None
        have_lock = False
        while time.time() - started < timeout_sec:
            try:
                fh = self.lock_path.open("a+b")
                try:
                    import msvcrt  # type: ignore

                    msvcrt.locking(fh.fileno(), msvcrt.LK_NBLCK, 1)
                    have_lock = True
                    break
                except Exception:
                    # If lock primitive unavailable, fallback to thread lock only
                    have_lock = False
                    break
            except Exception:
                time.sleep(0.05)
        try:
            yield
        finally:
            if fh is not None:
                try:
                    if have_lock:
                        import msvcrt  # type: ignore

                        fh.seek(0)
                        msvcrt.locking(fh.fileno(), msvcrt.LK_UNLCK, 1)
                except Exception:
                    pass
                try:
                    fh.close()
                except Exception:
                    pass

    def _read_raw(self) -> dict[str, Any]:
        if not self.path.exists():
            return {}
        try:
            payload = json.loads(self.path.read_text(encoding="utf-8"))
            return payload if isinstance(payload, dict) else {}
        except json.JSONDecodeError:
            ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            corrupt = self.path.with_suffix(self.path.suffix + f".corrupt-{ts}")
            try:
                os.replace(self.path, corrupt)
                logger.warning("shared_state corrupt JSON moved to %s", corrupt)
                if self._audit:
                    self._audit.write("shared_state_corrupt_recovered", path=str(self.path), moved_to=str(corrupt))
            except Exception as exc:
                logger.warning("shared_state corrupt move failed: %s", exc)
            return {}
        except Exception as exc:
            logger.warning("shared_state read error: %s", exc)
            return {}

    def read_state(self) -> dict[str, Any]:
        with self._thread_lock:
            data = self._read_raw()
            return dict(data)

    def ensure_defaults(self, defaults: dict[str, Any]) -> dict[str, Any]:
        def _mut(d: dict[str, Any]) -> dict[str, Any]:
            changed = False
            for k, v in defaults.items():
                if k not in d:
                    d[k] = v
                    changed = True
            if changed and self._audit:
                self._audit.write("shared_state_defaults_applied", path=str(self.path), keys=sorted(defaults.keys()))
            return d

        return self.write_state(_mut)

    def atomic_write(self, data: dict[str, Any]) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd, tmp_name = tempfile.mkstemp(prefix=self.path.name + ".", suffix=".tmp", dir=str(self.path.parent))
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as fh:
                json.dump(data, fh, indent=2, ensure_ascii=True)
                fh.flush()
                os.fsync(fh.fileno())
            os.replace(tmp_name, self.path)
        finally:
            try:
                if os.path.exists(tmp_name):
                    os.unlink(tmp_name)
            except Exception:
                pass

    def write_state(self, mutator: Callable[[dict[str, Any]], dict[str, Any] | None]) -> dict[str, Any]:
        with self._thread_lock:
            with self._file_lock():
                current = self._read_raw()
                candidate = mutator(dict(current))
                next_state = candidate if isinstance(candidate, dict) else current
                self.atomic_write(next_state)
                if self._audit:
                    self._audit.write("shared_state_write", path=str(self.path), keys=sorted(next_state.keys()))
                return dict(next_state)


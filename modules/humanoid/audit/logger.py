"""Audit logger: log_event(...). Respects AUDIT_ENABLED."""
from __future__ import annotations

import logging
import os
from typing import Any, Dict, List, Optional

from .db import AuditDB

_log = logging.getLogger("humanoid.audit")

_audit_db: Optional[AuditDB] = None


def _audit_enabled() -> bool:
    v = os.getenv("AUDIT_ENABLED", "true").strip().lower()
    return v in ("1", "true", "yes", "y", "on")


def get_audit_logger() -> AuditLogger:
    """Singleton AuditLogger (DB from AUDIT_DB_PATH if set)."""
    global _audit_db
    if _audit_db is None and os.getenv("AUDIT_DB_PATH"):
        _audit_db = AuditDB()
    return AuditLogger(db=_audit_db)


class AuditLogger:
    """Logs humanoid events for audit. log_event(actor, role, module, action, ok, ms, ...)."""

    def __init__(self, db: Optional[AuditDB] = None, log_to_std: bool = True) -> None:
        self._db = db
        self._log_to_std = log_to_std
        self._enabled = _audit_enabled()

    def log_event(
        self,
        actor: str,
        role: str,
        module: str,
        action: str,
        ok: bool,
        ms: int = 0,
        error: Optional[str] = None,
        payload: Optional[Dict[str, Any]] = None,
        result: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Record one audit event (no-op if AUDIT_ENABLED=false)."""
        if not self._enabled:
            return
        if self._log_to_std:
            _log.info("[audit] %s %s %s %s ok=%s ms=%s", actor, role, module, action, ok, ms)
        if self._db is not None:
            try:
                self._db.log_event(actor, role, module, action, ok, ms, error, payload, result)
            except Exception as e:
                _log.warning("AuditDB log_event failed: %s", e)

    def log(self, scope: str, action: str, payload: Dict[str, Any], result: str = "ok") -> None:
        """Legacy: log(scope, action, payload, result). Maps to log_event with actor=api, role=owner."""
        self.log_event("api", "owner", scope, action, result == "ok", 0, None, payload, {"result": result})

    def tail(self, n: int = 50, module: Optional[str] = None) -> List[Dict[str, Any]]:
        """Last n audit entries (delegates to DB if present)."""
        if self._db is None:
            return []
        return self._db.tail(n=n, module=module)

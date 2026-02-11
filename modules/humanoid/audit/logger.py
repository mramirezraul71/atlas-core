"""Audit logger: in-memory or delegated to AuditDB. Respects AUDIT_ENABLED."""
from __future__ import annotations

import logging
import os
from typing import Any, Dict, Optional

from .db import AuditDB

_log = logging.getLogger("humanoid.audit")


def _audit_enabled() -> bool:
    v = os.getenv("AUDIT_ENABLED", "true").strip().lower()
    return v in ("1", "true", "yes", "y", "on")


class AuditLogger:
    """Logs humanoid actions for audit. Can use AuditDB and/or Python logging."""

    def __init__(self, db: AuditDB | None = None, log_to_std: bool = True) -> None:
        self._db = db
        self._log_to_std = log_to_std
        self._enabled = _audit_enabled()

    def log(self, scope: str, action: str, payload: Dict[str, Any], result: str = "ok") -> None:
        """Record one audit event (no-op if AUDIT_ENABLED=false)."""
        if not self._enabled:
            return
        if self._log_to_std:
            _log.info("[audit] scope=%s action=%s result=%s payload=%s", scope, action, result, payload)
        if self._db is not None:
            try:
                self._db.log(scope, action, payload, result)
            except Exception as e:
                _log.warning("AuditDB log failed: %s", e)

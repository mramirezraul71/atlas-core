"""Structured JSONL audit log for Brain Core decisions."""
from __future__ import annotations

import json
import logging
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

logger = logging.getLogger("atlas.brain.audit")


def _default_audit_path() -> Path:
    repo_root = Path(__file__).resolve().parents[2]
    return repo_root / "data" / "operation" / "brain_audit_log.jsonl"


class AuditLog:
    def __init__(self, path: Path | None = None) -> None:
        self.path = path or _default_audit_path()
        self._lock = threading.RLock()

    def write(self, event_type: str, **fields: Any) -> None:
        rec = {
            "ts_utc": datetime.now(timezone.utc).isoformat(),
            "event_type": event_type,
            **fields,
        }
        line = json.dumps(rec, ensure_ascii=True)
        with self._lock:
            try:
                self.path.parent.mkdir(parents=True, exist_ok=True)
                with self.path.open("a", encoding="utf-8") as fh:
                    fh.write(line + "\n")
            except Exception as exc:
                logger.warning("audit_log write failed: %s", exc)


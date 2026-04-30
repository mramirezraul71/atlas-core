"""Persistencia JSON + JSONL para auditoría y reconstrucción."""
from __future__ import annotations

import json
import logging
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

logger = logging.getLogger("quant.notifications.storage")


class NotificationAuditStore:
    def __init__(self, base_dir: Path) -> None:
        self.base_dir = Path(base_dir)
        self.snapshots_dir = self.base_dir / "snapshots"
        self.snapshots_dir.mkdir(parents=True, exist_ok=True)
        self.events_path = self.base_dir / "notification_events.jsonl"

    def save_snapshot(self, kind: str, payload: dict[str, Any]) -> Path:
        ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        safe_kind = "".join(c if c.isalnum() or c in "-_" else "_" for c in kind)[:48]
        path = self.snapshots_dir / f"{safe_kind}_{ts}.json"
        try:
            path.write_text(json.dumps(payload, ensure_ascii=True, indent=2), encoding="utf-8")
        except Exception as exc:
            logger.warning("snapshot write failed: %s", exc)
        return path

    def append_event(self, record: dict[str, Any]) -> None:
        line = json.dumps(record, ensure_ascii=True) + "\n"
        try:
            with self.events_path.open("a", encoding="utf-8") as f:
                f.write(line)
        except Exception as exc:
            logger.warning("jsonl append failed: %s", exc)

    def list_recent_snapshots(self, *, limit: int = 20) -> list[dict[str, Any]]:
        files = sorted(self.snapshots_dir.glob("*.json"), key=lambda p: p.stat().st_mtime, reverse=True)
        out: list[dict[str, Any]] = []
        for p in files[:limit]:
            try:
                data = json.loads(p.read_text(encoding="utf-8"))
                out.append({"path": str(p), "kind": data.get("kind"), "generated_at": data.get("generated_at")})
            except Exception:
                out.append({"path": str(p), "error": "unreadable"})
        return out

    def tail_events(self, *, limit: int = 50) -> list[dict[str, Any]]:
        if not self.events_path.exists():
            return []
        try:
            lines = self.events_path.read_text(encoding="utf-8", errors="replace").splitlines()
        except Exception:
            return []
        rows: list[dict[str, Any]] = []
        for line in lines[-limit:]:
            try:
                rows.append(json.loads(line))
            except Exception:
                continue
        return rows

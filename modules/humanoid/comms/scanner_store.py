"""Store para último scan y descargas: visible en dashboard."""
from __future__ import annotations

from collections import deque
from datetime import datetime, timezone
from typing import Any, Dict, List

_LAST_SCAN: Dict[str, Any] = {}
_DOWNLOADS: deque = deque(maxlen=50)


def record_scan(snapshot: Dict[str, Any]) -> None:
    global _LAST_SCAN
    _LAST_SCAN = dict(snapshot)
    _LAST_SCAN["ts"] = datetime.now(timezone.utc).isoformat()


def record_download(packages: List[str], ok: bool, message: str = "", comment_es: str = "", comment_en: str = "") -> None:
    _DOWNLOADS.append({
        "ts": datetime.now(timezone.utc).isoformat(),
        "packages": list(packages),
        "ok": ok,
        "message": message[:200],
        "comment_es": comment_es[:120],
        "comment_en": comment_en[:120],
    })


def get_last_scan() -> Dict[str, Any]:
    return dict(_LAST_SCAN)


def get_last_downloads(limit: int = 20) -> List[Dict[str, Any]]:
    return list(_DOWNLOADS)[-limit:]

"""Bitácora ANS: entradas del módulo ATLAS_EVOLUTION para registro industrial. Sin silencio operativo.
Persistencia en logs/ans_evolution_bitacora.json para mantener registros estables tras reinicio."""
from __future__ import annotations

import json
import os
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List

_MAX_EVOLUTION_ENTRIES = 500
_evolution_entries: deque = deque(maxlen=_MAX_EVOLUTION_ENTRIES)


def _bitacora_path() -> Path:
    base = Path(os.environ.get("ATLAS_BASE", os.getcwd()))
    if not base.is_dir():
        base = Path(__file__).resolve().parent.parent.parent.parent
    return base / "logs" / "ans_evolution_bitacora.json"


def _load_from_disk() -> None:
    """Carga entradas desde disco al arranque."""
    global _evolution_entries
    p = _bitacora_path()
    if not p.exists():
        return
    try:
        data = json.loads(p.read_text(encoding="utf-8"))
        entries = data.get("entries", [])
        for e in entries[-_MAX_EVOLUTION_ENTRIES:]:
            _evolution_entries.append(e)
    except Exception:
        pass


def _save_to_disk() -> None:
    """Persiste las últimas entradas en disco."""
    p = _bitacora_path()
    try:
        p.parent.mkdir(parents=True, exist_ok=True)
        entries = list(_evolution_entries)
        p.write_text(
            json.dumps({"entries": entries, "updated": datetime.now(timezone.utc).isoformat()}, ensure_ascii=False),
            encoding="utf-8",
        )
    except Exception:
        pass


# Carga inicial desde disco
_load_from_disk()


def append_evolution_log(message: str, ok: bool = True, source: str = "evolution") -> None:
    """Append one step to the bitácora. Called by POST /ans/evolution-log. source: evolution | repo_monitor | ..."""
    entry = {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "message": (message or "")[:500],
        "ok": ok,
        "source": (source or "evolution").strip() or "evolution",
    }
    _evolution_entries.append(entry)
    _save_to_disk()


def get_evolution_entries(limit: int = 100) -> List[Dict]:
    """Return the most recent evolution log entries (newest first for merge)."""
    return list(_evolution_entries)[-limit:][::-1]

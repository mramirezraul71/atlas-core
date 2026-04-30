"""Parser mínimo para artefactos LEAN (F1)."""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any


def parse_statistics(path: str | Path) -> dict[str, Any]:
    """Parsea JSON de estadísticas LEAN si existe; fallback vacío."""
    p = Path(path)
    if not p.exists():
        return {}
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except Exception:
        return {}

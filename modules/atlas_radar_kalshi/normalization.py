from __future__ import annotations

import re
from datetime import datetime


def _normalize_text(text: str) -> str:
    t = (text or "").lower().strip()
    t = re.sub(r"[^a-z0-9\s]", " ", t)
    t = re.sub(r"\s+", " ", t).strip()
    return t


def canonical_market_key(title: str, close_time: str | None) -> str:
    """Genera clave canónica simple para matching entre venues."""
    base = _normalize_text(title)
    day = "na"
    if close_time:
        try:
            dt = datetime.fromisoformat(str(close_time).replace("Z", "+00:00"))
            day = dt.strftime("%Y-%m-%d")
        except Exception:
            day = str(close_time)[:10]
    tokens = base.split()
    # Conservador: recorta ruido pero mantiene semántica base.
    keep = [x for x in tokens if len(x) > 2][:12]
    return f"{' '.join(keep)}|{day}"


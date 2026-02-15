"""Locate elements by text or position (from OCR/layout)."""
from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple

from .layout import get_layout


def locate(query: str, region: Optional[Tuple[int, int, int, int]] = None) -> Dict[str, Any]:
    """
    Find elements matching query (text substring). Returns {ok, matches: [{bbox, text}], error}.
    bbox = [x, y, w, h] approximate (full region if no granular boxes).
    """
    layout = get_layout(region=region)
    if layout.get("error"):
        return {"ok": False, "matches": [], "error": layout["error"]}
    q = (query or "").strip().lower()
    if not q:
        return {"ok": True, "matches": [], "error": None}
    full_text = (layout.get("full_text") or "").lower()
    matches: List[Dict[str, Any]] = []
    for reg in layout.get("regions", []):
        text = (reg.get("text") or "")
        low = text.lower()
        if q and q in low:
            matches.append({"bbox": reg.get("bbox", [0, 0, 0, 0]), "text": text[:200]})
    # Fallback: si solo hay full_text sin regiones granulares, devolver región completa
    if not matches and full_text and q and q in full_text:
        matches = [{"bbox": [0, 0, layout.get("width", 0), layout.get("height", 0)], "text": full_text[:200]}]
    return {"ok": True, "matches": matches, "error": None}

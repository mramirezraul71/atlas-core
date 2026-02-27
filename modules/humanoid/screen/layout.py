"""Segmentación: regiones UI + bounding boxes (simplificado: full screen + OCR boxes)."""
from __future__ import annotations

import io
from typing import Any, Dict, Optional, Tuple

from .capture import capture_screen
from .ocr import run_ocr, run_ocr_data


def get_layout(region: Optional[Tuple[int, int, int, int]] = None) -> Dict[str, Any]:
    """
    Returns layout: {regions: [{bbox, text?}], full_text, width, height}.
    Simple implementation: one full-screen region + full_text from OCR.
    """
    png, err = capture_screen(region=region)
    if err or not png:
        return {"regions": [], "full_text": "", "width": 0, "height": 0, "error": err}

    try:
        from PIL import Image
        img = Image.open(io.BytesIO(png))
        w, h = img.size
    except Exception as e:
        return {"regions": [], "full_text": "", "width": 0, "height": 0, "error": str(e)}

    # OCR detallado (boxes) si está disponible
    items, derr = run_ocr_data(image_bytes=png)
    if not derr and items:
        regions = [{"bbox": it["bbox"], "text": it["text"]} for it in items[:1200]]
        full_text = " ".join([it["text"] for it in items[:5000]])
        return {"regions": regions, "full_text": full_text, "width": w, "height": h, "error": None}

    # Fallback simple: texto completo sin boxes
    text, _ = run_ocr(image_bytes=png)
    regions = [{"bbox": [0, 0, w, h], "text": text[:500] if text else ""}]
    return {"regions": regions, "full_text": text, "width": w, "height": h, "error": None}

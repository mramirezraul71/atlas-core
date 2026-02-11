"""Image analysis: read local image, OCR (pytesseract), LLM vision. Fallbacks: OCR only / metadata only."""
from __future__ import annotations

import base64
import logging
import os
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

_log = logging.getLogger("humanoid.vision")
_ocr_available: Optional[bool] = None
_llm_vision_available: Optional[bool] = None

VISION_TIMEOUT_SEC = 30


def _check_pytesseract() -> bool:
    global _ocr_available
    if _ocr_available is not None:
        return _ocr_available
    try:
        import pytesseract
        pytesseract.get_tesseract_version()
        _ocr_available = True
    except Exception:
        _ocr_available = False
    return _ocr_available


def _check_pillow() -> bool:
    try:
        from PIL import Image
        return True
    except ImportError:
        return False


def _read_image_as_base64(path: str, max_size_mb: float = 5.0) -> Optional[str]:
    try:
        p = Path(path).resolve()
        if not p.exists() or not p.is_file():
            return None
        if p.stat().st_size > max_size_mb * 1024 * 1024:
            return None
        with open(p, "rb") as f:
            return base64.b64encode(f.read()).decode("utf-8")
    except Exception:
        return None


def ocr(image_path: str) -> Dict[str, Any]:
    """Extract text with pytesseract. Fallback: empty if not available."""
    t0 = time.perf_counter()
    out = {"ok": False, "extracted_text": "", "interpretation": "", "entities": [], "ms": 0, "error": None}
    if not _check_pillow():
        out["error"] = "PIL/Pillow not available"
        out["ms"] = int((time.perf_counter() - t0) * 1000)
        return out
    if not _check_pytesseract():
        out["error"] = "pytesseract not available"
        out["ms"] = int((time.perf_counter() - t0) * 1000)
        return out
    try:
        from PIL import Image
        import pytesseract
        img = Image.open(image_path)
        text = pytesseract.image_to_string(img)
        out["ok"] = True
        out["extracted_text"] = (text or "").strip()
        out["entities"] = [t for t in out["extracted_text"].split() if len(t) > 2][:50]
    except Exception as e:
        out["error"] = str(e)
    out["ms"] = int((time.perf_counter() - t0) * 1000)
    return out


def analyze_with_llm(image_path: str, prompt: str = "Describe what you see and any text.") -> Dict[str, Any]:
    """Analyze image with vision model (Ollama llama3.2-vision etc). Fallback: metadata only."""
    t0 = time.perf_counter()
    out = {"ok": False, "extracted_text": "", "interpretation": "", "entities": [], "ms": 0, "error": None}
    b64 = _read_image_as_base64(image_path)
    if not b64:
        out["error"] = "could not read image"
        out["ms"] = int((time.perf_counter() - t0) * 1000)
        return out
    try:
        import httpx
        url = os.getenv("OLLAMA_BASE_URL", "http://127.0.0.1:11434") + "/api/generate"
        payload = {
            "model": os.getenv("LLM_VISION_MODEL", "llava:7b"),
            "prompt": prompt,
            "images": [b64],
            "stream": False,
        }
        with httpx.Client(timeout=VISION_TIMEOUT_SEC) as client:
            r = client.post(url, json=payload)
            if r.status_code != 200:
                out["error"] = f"ollama {r.status_code}"
                out["ms"] = int((time.perf_counter() - t0) * 1000)
                return out
            data = r.json()
            out["interpretation"] = (data.get("response") or "").strip()
            out["ok"] = True
    except Exception as e:
        out["error"] = str(e)
    out["ms"] = int((time.perf_counter() - t0) * 1000)
    return out


def analyze(image_path: str, use_ocr: bool = True, use_llm_vision: bool = True) -> Dict[str, Any]:
    """Full analysis: OCR + LLM vision. Fallbacks: OCR only; then metadata only."""
    t0 = time.perf_counter()
    extracted_text = ""
    interpretation = ""
    entities = []
    if use_ocr:
        ocr_result = ocr(image_path)
        extracted_text = ocr_result.get("extracted_text", "")
        entities = ocr_result.get("entities", [])
    if use_llm_vision:
        llm_result = analyze_with_llm(image_path)
        if llm_result.get("ok"):
            interpretation = llm_result.get("interpretation", "")
        elif not extracted_text and not interpretation:
            interpretation = "(LLM vision unavailable; use OCR or check model)"
    if not extracted_text and not interpretation:
        try:
            from PIL import Image
            img = Image.open(image_path)
            interpretation = f"Image {img.size[0]}x{img.size[1]} mode={img.mode}"
        except Exception:
            interpretation = "Could not read image"
    # Insights y acciones sugeridas (respuesta profesional)
    insights = _build_insights(extracted_text, interpretation, entities)
    acciones_sugeridas = _build_acciones_sugeridas(extracted_text, interpretation, entities)
    ms = int((time.perf_counter() - t0) * 1000)
    return {
        "ok": True,
        "extracted_text": extracted_text,
        "interpretation": interpretation,
        "entities": entities,
        "insights": insights,
        "acciones_sugeridas": acciones_sugeridas,
        "ms": ms,
        "error": None,
    }


def _build_insights(extracted_text: str, interpretation: str, entities: List[str]) -> List[str]:
    """Deriva insights cortos a partir de texto e interpretación."""
    insights: List[str] = []
    if interpretation:
        insights.append(interpretation[:300] if len(interpretation) > 300 else interpretation)
    if extracted_text and extracted_text not in (interpretation or ""):
        insights.append(f"Texto detectado: {len(extracted_text)} caracteres")
    if entities:
        insights.append(f"Entidades: {', '.join(entities[:10])}")
    return insights[:5]


def _build_acciones_sugeridas(extracted_text: str, interpretation: str, entities: List[str]) -> List[str]:
    """Sugerencias de acción según contenido."""
    acciones: List[str] = []
    if extracted_text:
        acciones.append("Copiar texto extraído al portapapeles o guardar en memoria")
    if interpretation and ("código" in interpretation.lower() or "code" in interpretation.lower()):
        acciones.append("Revisar si hay código en imagen para ejecutar o guardar")
    if entities:
        acciones.append("Usar entidades para búsqueda o etiquetado")
    if not acciones:
        acciones.append("Usar imagen como referencia o adjunto en tarea")
    return acciones[:5]


def screenshot_analyze(screenshot_path: str) -> Dict[str, Any]:
    """Analyze a screenshot (same as analyze)."""
    return analyze(screenshot_path, use_ocr=True, use_llm_vision=True)


def vision_status() -> Dict[str, Any]:
    return {
        "ocr_available": _check_pytesseract(),
        "pillow_available": _check_pillow(),
        "llm_vision_available": _check_llm_vision(),
        "missing_deps": _missing_deps(),
    }


def _check_llm_vision() -> bool:
    try:
        import httpx
        url = os.getenv("OLLAMA_BASE_URL", "http://127.0.0.1:11434") + "/api/tags"
        with httpx.Client(timeout=5) as c:
            r = c.get(url)
            return r.status_code == 200
    except Exception:
        return False


def _missing_deps() -> List[str]:
    missing = []
    if not _check_pillow():
        missing.append("pillow")
    if not _check_pytesseract():
        missing.append("pytesseract")
    return missing

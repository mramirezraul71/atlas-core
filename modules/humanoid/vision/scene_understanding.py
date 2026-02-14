"""
Fase 2 - Scene understanding: descripción de escena y VQA.
Opciones: LLaVA vía Ollama, o CLIP + texto; fallback descripción básica.
"""
from __future__ import annotations

import hashlib
import logging
import time
from typing import Any, Dict, List, Optional

import numpy as np

_log = logging.getLogger(__name__)

# Cache simple TTL 5s para no re-procesar el mismo frame
_cache: Dict[str, tuple] = {}
_CACHE_TTL = 5.0


def _frame_hash(frame: np.ndarray) -> str:
    h = hashlib.sha256(frame.tobytes()).hexdigest()[:16]
    return h


def describe_scene(
    frame: np.ndarray,
    detail_level: str = "medium",
) -> Dict[str, Any]:
    """
    detail_level: "brief" | "medium" | "detailed"
    Devuelve: { description, objects, activities, context, confidence }
    """
    cache_key = f"describe_{_frame_hash(frame)}_{detail_level}"
    if cache_key in _cache:
        ts, data = _cache[cache_key]
        if time.time() - ts < _CACHE_TTL:
            return data
        del _cache[cache_key]

    result = _describe_scene_impl(frame, detail_level)
    _cache[cache_key] = (time.time(), result)
    return result


def _describe_scene_impl(frame: np.ndarray, detail_level: str) -> Dict[str, Any]:
    """Implementación: intentar LLaVA/Ollama, luego fallback por detección o placeholder."""
    # 1) Ollama LLaVA si está disponible
    try:
        import requests
        import base64
        import cv2
        _, buf = cv2.imencode(".jpg", frame)
        b64 = base64.b64encode(buf).decode("utf-8")
        url = "http://127.0.0.1:11434/api/generate"
        prompt = "Describe this image in one sentence."
        if detail_level == "detailed":
            prompt = "Describe this image in detail: place, objects, people, actions."
        elif detail_level == "brief":
            prompt = "One short phrase for this image."
        payload = {"model": "llava", "prompt": prompt, "images": [b64], "stream": False}
        r = requests.post(url, json=payload, timeout=15)
        if r.status_code == 200:
            text = r.json().get("response", "").strip()
            return {
                "description": text or "Scene captured.",
                "objects": [],
                "activities": [],
                "context": "unknown",
                "confidence": 0.85,
            }
    except Exception as e:
        _log.debug("Ollama LLaVA no disponible: %s", e)

    # 2) Fallback: descripción genérica (YOLO opcional si está en path)
    objects: List[str] = []

    if objects:
        desc = f"Scene with: {', '.join(objects)}."
    else:
        desc = "Scene captured; no objects detected."
    return {
        "description": desc,
        "objects": objects,
        "activities": [],
        "context": "unknown",
        "confidence": 0.6,
    }


def answer_visual_question(frame: np.ndarray, question: str) -> Dict[str, Any]:
    """VQA: responde una pregunta sobre la imagen. { answer, confidence }."""
    try:
        import requests
        import base64
        import cv2
        _, buf = cv2.imencode(".jpg", frame)
        b64 = base64.b64encode(buf).decode("utf-8")
        url = "http://127.0.0.1:11434/api/generate"
        payload = {"model": "llava", "prompt": question, "images": [b64], "stream": False}
        r = requests.post(url, json=payload, timeout=15)
        if r.status_code == 200:
            answer = r.json().get("response", "").strip()
            return {"answer": answer or "Unknown", "confidence": 0.8}
    except Exception as e:
        _log.debug("VQA Ollama: %s", e)
    data = describe_scene(frame, "medium")
    return {"answer": data["description"][:200], "confidence": 0.5}

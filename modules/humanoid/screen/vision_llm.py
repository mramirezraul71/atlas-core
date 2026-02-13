"""Analyze screenshot with AI_VISION_MODEL (Ollama vision)."""
from __future__ import annotations

import base64
import os
from typing import Any, Dict, Optional

from .status import _screen_deps_ok


def analyze_image(image_base64: Optional[str] = None, image_path: Optional[str] = None, prompt: str = "Describe what you see. If UI: list buttons, fields, labels and suggest actions.") -> Dict[str, Any]:
    """
    Send image to vision model. Returns {ok, description, suggestions, error}.
    Uses Ollama vision model from config.
    """
    if not _screen_deps_ok():
        return {"ok": False, "description": "", "suggestions": [], "error": "screen_deps_missing"}
    b64 = image_base64
    if not b64 and image_path and os.path.isfile(image_path):
        with open(image_path, "rb") as f:
            b64 = base64.b64encode(f.read()).decode("ascii")
    if not b64:
        return {"ok": False, "description": "", "suggestions": [], "error": "no image"}

    model = (os.getenv("AI_VISION_MODEL") or "ollama:llama3.2-vision:11b").strip()
    if ":" in model:
        model = model.split(":", 1)[1]
    url = (os.getenv("OLLAMA_BASE_URL") or "http://127.0.0.1:11434").rstrip("/")
    timeout = int(os.getenv("OLLAMA_TIMEOUT_S", "60") or "60")

    try:
        import httpx
        r = httpx.post(
            f"{url}/api/generate",
            json={
                "model": model,
                "prompt": prompt,
                "images": [b64],
                "stream": False,
            },
            timeout=timeout,
        )
        if r.status_code != 200:
            return {"ok": False, "description": "", "suggestions": [], "error": f"ollama status {r.status_code}"}
        data = r.json()
        desc = (data.get("response") or "").strip()
        return {"ok": True, "description": desc, "suggestions": [], "error": None}
    except Exception as e:
        return {"ok": False, "description": "", "suggestions": [], "error": str(e)}

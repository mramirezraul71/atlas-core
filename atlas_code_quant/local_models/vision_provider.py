"""Vision provider to analyze local dashboard screenshots."""
from __future__ import annotations

import base64
import json
from pathlib import Path
from typing import Any

from .config import get_role_model
from .metrics import record_local_model_result
from .ollama_client import OllamaClient


def _emit_fallback(model: str, reason: str) -> None:
    try:
        from learning.event_store import get_event_store
    except Exception:
        try:
            from atlas_code_quant.learning.event_store import get_event_store  # type: ignore
        except Exception:
            return
    try:
        get_event_store().append(
            "local_model_fallback",
            {"role": "vision", "model": model, "reason": reason},
            source="local_models",
        )
    except Exception:
        pass
    record_local_model_result(
        provider_role="vision",
        requested_model=model,
        used_fallback=True,
        fallback_reason=reason,
        outcome="fallback",
        duration_ms=None,
    )


class LocalVisionProvider:
    def __init__(self, client: OllamaClient, cfg: dict[str, Any]) -> None:
        self._client = client
        self._cfg = cfg
        self._enabled = bool(cfg.get("enabled", True))
        self._model = get_role_model("vision", cfg) or "llama3.2-vision:11b"
        self._premium_model = get_role_model("vision_premium", cfg) or "qwen3-vl:30b"

    def analyze_screenshot(self, image_path: str, *, prompt: str | None = None, premium: bool = False) -> dict[str, Any]:
        path = Path(image_path)
        model = self._premium_model if premium else self._model
        if not path.exists():
            _emit_fallback(model, "file_not_found")
            return {
                "ok": False,
                "error": "file_not_found",
                "path": str(path),
                "model": model,
                "requested_model": model,
                "provider_role": "vision",
                "used_fallback": True,
                "fallback_reason": "file_not_found",
            }
        if not self._enabled:
            _emit_fallback(model, "provider_disabled")
            return {
                "ok": False,
                "error": "provider_disabled",
                "path": str(path),
                "model": model,
                "requested_model": model,
                "provider_role": "vision",
                "used_fallback": True,
                "fallback_reason": "provider_disabled",
            }
        user_prompt = prompt or (
            "Analyze this ATLAS dashboard screenshot and return JSON with "
            "status, observations (list), warnings (list), and suggested_actions (list)."
        )
        encoded = base64.b64encode(path.read_bytes()).decode("utf-8")
        messages = [{"role": "user", "content": user_prompt, "images": [encoded]}]
        try:
            raw = self._client.chat(model=model, messages=messages, role="vision")
            parsed: dict[str, Any]
            try:
                parsed = json.loads(raw)
                if not isinstance(parsed, dict):
                    parsed = {"raw": raw}
            except Exception:
                parsed = {"raw": raw}
            parsed["ok"] = True
            parsed["model"] = model
            parsed["requested_model"] = model
            parsed["provider_role"] = "vision"
            parsed["used_fallback"] = False
            parsed["fallback_reason"] = None
            parsed["path"] = str(path)
            return parsed
        except Exception as exc:
            _emit_fallback(model, f"ollama_unavailable:{exc}")
            return {
                "ok": False,
                "error": str(exc),
                "model": model,
                "requested_model": model,
                "provider_role": "vision",
                "used_fallback": True,
                "fallback_reason": f"ollama_unavailable:{exc}",
                "path": str(path),
            }

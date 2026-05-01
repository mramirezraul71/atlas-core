"""Cheap local classifier for event/journal lightweight review."""
from __future__ import annotations

import json
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
            {"role": "classifier", "model": model, "reason": reason},
            source="local_models",
        )
    except Exception:
        pass
    record_local_model_result(
        provider_role="classifier",
        requested_model=model,
        used_fallback=True,
        fallback_reason=reason,
        outcome="fallback",
        duration_ms=None,
    )


def _extract_json(text: str) -> dict[str, Any]:
    candidate = str(text or "").strip()
    if not candidate:
        return {}
    try:
        data = json.loads(candidate)
        return data if isinstance(data, dict) else {}
    except Exception:
        pass
    start = candidate.find("{")
    end = candidate.rfind("}")
    if start >= 0 and end > start:
        try:
            data = json.loads(candidate[start : end + 1])
            return data if isinstance(data, dict) else {}
        except Exception:
            return {}
    return {}


class LocalClassifierProvider:
    def __init__(self, client: OllamaClient, cfg: dict[str, Any]) -> None:
        self._client = client
        self._cfg = cfg
        self._enabled = bool(cfg.get("enabled", True))
        self._model = get_role_model("classifier", cfg) or "llama3.1:8b"

    def _heuristic(self, payload: dict[str, Any], reason: str) -> dict[str, Any]:
        text = json.dumps(payload, ensure_ascii=False, default=str).lower()
        label = "neutral"
        if any(token in text for token in ("error", "fail", "blocked", "exception")):
            label = "incident"
        elif any(token in text for token in ("entry_execution", "close_execution", "filled", "profit")):
            label = "trading_event"
        summary = f"Heuristic review ({label}) generated due to {reason}."
        _emit_fallback(self._model, reason)
        return {
            "ok": True,
            "label": label,
            "summary": summary,
            "tags": [label],
            "confidence": 0.35,
            "model": "heuristic-fallback",
            "requested_model": self._model,
            "provider_role": "classifier",
            "used_fallback": True,
            "fallback_reason": reason,
        }

    def classify(self, payload: dict[str, Any]) -> dict[str, Any]:
        if not self._enabled:
            return self._heuristic(payload, "provider_disabled")
        prompt = (
            "You are a lightweight ATLAS reviewer. Return strict JSON with keys: "
            "label, summary, tags (list of short tags), confidence (0..1). "
            "Payload:\n"
            f"{json.dumps(payload, ensure_ascii=False, default=str)}"
        )
        try:
            out = self._client.generate(model=self._model, prompt=prompt, role="classifier")
            parsed = _extract_json(out)
            if not parsed:
                return self._heuristic(payload, "invalid_json_response")
            parsed.setdefault("label", "neutral")
            parsed.setdefault("summary", "Local classifier response")
            parsed.setdefault("tags", [str(parsed.get("label"))])
            parsed.setdefault("confidence", 0.5)
            parsed["ok"] = True
            parsed["model"] = self._model
            parsed["requested_model"] = self._model
            parsed["provider_role"] = "classifier"
            parsed["used_fallback"] = False
            parsed["fallback_reason"] = None
            return parsed
        except Exception as exc:
            return self._heuristic(payload, f"ollama_unavailable:{exc}")

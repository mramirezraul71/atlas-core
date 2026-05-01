"""Resilient Ollama client with local observability events."""
from __future__ import annotations

import json
import logging
import time
from typing import Any

import requests

from .metrics import record_local_model_result

logger = logging.getLogger("quant.local_models.ollama")


class OllamaUnavailable(RuntimeError):
    """Raised when Ollama local service is not reachable."""


def _emit_event(topic: str, payload: dict[str, Any]) -> None:
    try:
        from learning.event_store import get_event_store
    except Exception:
        try:
            from atlas_code_quant.learning.event_store import get_event_store  # type: ignore
        except Exception:
            return
    try:
        get_event_store().append(topic, payload, source="local_models")
    except Exception:
        pass


class OllamaClient:
    def __init__(self, *, base_url: str, timeout_seconds: float, enabled: bool = True) -> None:
        self.base_url = str(base_url).rstrip("/")
        self.timeout_seconds = float(timeout_seconds)
        self.enabled = bool(enabled)

    def _post(self, path: str, payload: dict[str, Any], *, role: str, model: str) -> dict[str, Any]:
        if not self.enabled:
            raise OllamaUnavailable("local_models_disabled")
        url = f"{self.base_url}{path}"
        started = time.perf_counter()
        try:
            response = requests.post(url, json=payload, timeout=self.timeout_seconds)
            response.raise_for_status()
            data = response.json()
            _emit_event(
                "local_model_call",
                {
                    "role": role,
                    "model": model,
                    "duration_ms": round((time.perf_counter() - started) * 1000.0, 3),
                    "success": True,
                    "path": path,
                },
            )
            record_local_model_result(
                provider_role=role,
                requested_model=model,
                used_fallback=False,
                fallback_reason=None,
                outcome="success",
                duration_ms=round((time.perf_counter() - started) * 1000.0, 3),
            )
            return data if isinstance(data, dict) else {"response": data}
        except Exception as exc:
            duration_ms = round((time.perf_counter() - started) * 1000.0, 3)
            _emit_event(
                "local_model_failure",
                {
                    "role": role,
                    "model": model,
                    "duration_ms": duration_ms,
                    "success": False,
                    "path": path,
                    "error": str(exc),
                },
            )
            record_local_model_result(
                provider_role=role,
                requested_model=model,
                used_fallback=False,
                fallback_reason=None,
                outcome="failure",
                duration_ms=duration_ms,
            )
            raise OllamaUnavailable(str(exc)) from exc

    def generate(self, *, model: str, prompt: str, role: str, options: dict[str, Any] | None = None) -> str:
        payload: dict[str, Any] = {
            "model": model,
            "prompt": prompt,
            "stream": False,
        }
        if options:
            payload["options"] = dict(options)
        data = self._post("/api/generate", payload, role=role, model=model)
        return str(data.get("response") or "")

    def chat(
        self,
        *,
        model: str,
        messages: list[dict[str, Any]],
        role: str,
        options: dict[str, Any] | None = None,
    ) -> str:
        payload: dict[str, Any] = {
            "model": model,
            "messages": messages,
            "stream": False,
        }
        if options:
            payload["options"] = dict(options)
        data = self._post("/api/chat", payload, role=role, model=model)
        message = data.get("message")
        if isinstance(message, dict):
            return str(message.get("content") or "")
        return str(data.get("response") or "")

    def embeddings(self, *, model: str, input_texts: list[str], role: str) -> list[list[float]]:
        vectors: list[list[float]] = []
        for text in input_texts:
            payload = {"model": model, "prompt": text}
            data = self._post("/api/embeddings", payload, role=role, model=model)
            emb = data.get("embedding")
            if isinstance(emb, list):
                vectors.append([float(x) for x in emb])
                continue
            multi = data.get("embeddings")
            if isinstance(multi, list) and multi and isinstance(multi[0], list):
                vectors.append([float(x) for x in multi[0]])
                continue
            raise OllamaUnavailable(f"invalid_embeddings_response:{json.dumps(data)[:180]}")
        return vectors

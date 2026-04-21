"""Embedding provider for journal semantic retrieval."""
from __future__ import annotations

import hashlib
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
            {"role": "embeddings", "model": model, "reason": reason},
            source="local_models",
        )
    except Exception:
        pass
    record_local_model_result(
        provider_role="embeddings",
        requested_model=model,
        used_fallback=True,
        fallback_reason=reason,
        outcome="fallback",
        duration_ms=None,
    )


class LocalEmbeddingProvider:
    def __init__(self, client: OllamaClient, cfg: dict[str, Any]) -> None:
        self._client = client
        self._cfg = cfg
        self._model = get_role_model("embeddings", cfg) or "nomic-embed-text:latest"
        self._enabled = bool(cfg.get("enabled", True))

    def _fallback_embed(self, text: str, dim: int = 32) -> list[float]:
        digest = hashlib.sha256(text.encode("utf-8", errors="ignore")).digest()
        raw = list(digest) * ((dim // len(digest)) + 1)
        return [(float(v) / 255.0) for v in raw[:dim]]

    def embed_text(self, texts: list[str]) -> list[list[float]]:
        if not texts:
            return []
        if not self._enabled:
            _emit_fallback(self._model, "provider_disabled")
            return [self._fallback_embed(text) for text in texts]
        try:
            return self._client.embeddings(model=self._model, input_texts=texts, role="embeddings")
        except Exception as exc:
            _emit_fallback(self._model, f"ollama_unavailable:{exc}")
            return [self._fallback_embed(text) for text in texts]

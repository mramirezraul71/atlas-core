"""Registry/router for local model providers."""
from __future__ import annotations

from typing import Any

from .classifier_provider import LocalClassifierProvider
from .config import load_local_model_config
from .embedding_provider import LocalEmbeddingProvider
from .ollama_client import OllamaClient
from .vision_provider import LocalVisionProvider

_CACHE: dict[str, Any] = {}


def _config() -> dict[str, Any]:
    if "cfg" not in _CACHE:
        _CACHE["cfg"] = load_local_model_config()
    return _CACHE["cfg"]


def get_ollama_client() -> OllamaClient:
    if "client" not in _CACHE:
        cfg = _config()
        _CACHE["client"] = OllamaClient(
            base_url=str(cfg.get("ollama_base_url", "http://localhost:11434")),
            timeout_seconds=float(cfg.get("timeout_seconds", 20.0)),
            enabled=bool(cfg.get("enabled", True)),
        )
    return _CACHE["client"]


def get_embedding_provider() -> LocalEmbeddingProvider:
    if "embedding_provider" not in _CACHE:
        _CACHE["embedding_provider"] = LocalEmbeddingProvider(get_ollama_client(), _config())
    return _CACHE["embedding_provider"]


def get_classifier_provider() -> LocalClassifierProvider:
    if "classifier_provider" not in _CACHE:
        _CACHE["classifier_provider"] = LocalClassifierProvider(get_ollama_client(), _config())
    return _CACHE["classifier_provider"]


def get_vision_provider() -> LocalVisionProvider:
    if "vision_provider" not in _CACHE:
        _CACHE["vision_provider"] = LocalVisionProvider(get_ollama_client(), _config())
    return _CACHE["vision_provider"]


def get_coder_profile() -> dict[str, Any]:
    cfg = _config()
    roles = cfg.get("roles") if isinstance(cfg.get("roles"), dict) else {}
    return {
        "enabled": bool(cfg.get("enabled", True)),
        "coder_primary": roles.get("coder_primary"),
        "coder_fallback": roles.get("coder_fallback"),
        "coder_premium": roles.get("coder_premium"),
    }


def reset_router_cache() -> None:
    _CACHE.clear()

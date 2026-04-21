"""Central config for local Ollama models used by ATLAS helpers."""
from __future__ import annotations

import copy
import os
from typing import Any

LOCAL_MODEL_CONFIG: dict[str, Any] = {
    "enabled": True,
    "ollama_base_url": "http://localhost:11434",
    "timeout_seconds": 20.0,
    "roles": {
        "embeddings": "nomic-embed-text:latest",
        "classifier": "llama3.1:8b",
        "coder_primary": "deepseek-coder-v2:16b",
        "coder_fallback": "qwen2.5-coder:7b",
        "coder_premium": "qwen3-coder:30b",
        "vision": "llama3.2-vision:11b",
        "vision_premium": "qwen3-vl:30b",
    },
}

_ROLE_ENV_MAP = {
    "embeddings": "ATLAS_LOCAL_MODEL_EMBEDDINGS",
    "classifier": "ATLAS_LOCAL_MODEL_CLASSIFIER",
    "coder_primary": "ATLAS_LOCAL_MODEL_CODER_PRIMARY",
    "coder_fallback": "ATLAS_LOCAL_MODEL_CODER_FALLBACK",
    "coder_premium": "ATLAS_LOCAL_MODEL_CODER_PREMIUM",
    "vision": "ATLAS_LOCAL_MODEL_VISION",
    "vision_premium": "ATLAS_LOCAL_MODEL_VISION_PREMIUM",
}


def _env_bool(name: str, default: bool) -> bool:
    raw = os.getenv(name)
    if raw is None:
        return default
    return str(raw).strip().lower() in {"1", "true", "yes", "on"}


def load_local_model_config() -> dict[str, Any]:
    """Build effective config with env overrides."""
    cfg = copy.deepcopy(LOCAL_MODEL_CONFIG)
    cfg["enabled"] = _env_bool("ATLAS_LOCAL_MODELS_ENABLED", bool(cfg.get("enabled", True)))
    cfg["ollama_base_url"] = str(os.getenv("ATLAS_OLLAMA_BASE_URL", cfg["ollama_base_url"])).strip()
    try:
        cfg["timeout_seconds"] = float(os.getenv("ATLAS_OLLAMA_TIMEOUT_SECONDS", str(cfg["timeout_seconds"])))
    except Exception:
        cfg["timeout_seconds"] = float(cfg["timeout_seconds"])
    roles = cfg.setdefault("roles", {})
    for role, env_name in _ROLE_ENV_MAP.items():
        override = os.getenv(env_name)
        if override:
            roles[role] = str(override).strip()
    return cfg


def get_role_model(role: str, cfg: dict[str, Any] | None = None) -> str | None:
    active = cfg or load_local_model_config()
    roles = active.get("roles") if isinstance(active.get("roles"), dict) else {}
    value = roles.get(role)
    return str(value).strip() if value else None

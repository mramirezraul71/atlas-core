"""Provider and model registry (Ollama priority 1, optional paid, NullProvider fallback)."""
from __future__ import annotations

import os
from typing import List, Optional

from .models import ModelSpec, Provider


def _env(name: str, default: str) -> str:
    v = os.getenv(name)
    return (v or "").strip() or default


def _env_bool(name: str, default: bool) -> bool:
    v = _env(name, "true" if default else "false").lower()
    return v in ("1", "true", "yes", "y", "on")


def _parse_model_key(key: str) -> tuple[str, str]:
    """Return (provider_id, model_name) for keys like ollama:llama3.2:3b."""
    if ":" in key:
        parts = key.split(":", 1)
        return parts[0].lower(), (parts[1] or "").strip()
    return "ollama", key


def list_providers(ollama_available: bool) -> List[Provider]:
    """Build provider list; ollama_available comes from health check."""
    providers: List[Provider] = []

    # 1) Ollama (priority 1, free)
    ollama_models = [
        _env("AI_FAST_MODEL", "ollama:llama3.2:3b"),
        _env("AI_CHAT_MODEL", "ollama:llama3.1:latest"),
        _env("AI_CODE_MODEL", "ollama:deepseek-coder:6.7b"),
        _env("AI_REASON_MODEL", "ollama:deepseek-r1:14b"),
        _env("AI_TOOLS_MODEL", "ollama:qwen2.5:7b"),
        _env("AI_VISION_MODEL", "ollama:llama3.2-vision:11b"),
        _env("AI_ARCHITECT_MODEL", "ollama:deepseek-r1:14b"),
        _env("AI_OPTIMIZER_MODEL", "ollama:deepseek-coder:6.7b"),
    ]
    providers.append(
        Provider(
            id="ollama",
            name="Ollama (local)",
            is_free=True,
            supports_vision=True,
            available=ollama_available,
            models=[_parse_model_key(m)[1] or m for m in ollama_models if m.startswith("ollama:")],
        )
    )

    # 2–5) Optional paid (available only if key set and APIs allowed)
    allow_external = _env_bool("AI_ALLOW_EXTERNAL_APIS", False)
    if _env("OPENAI_API_KEY", "").strip() and allow_external:
        providers.append(
            Provider(id="openai", name="OpenAI", is_free=False, supports_vision=True, available=True, models=[])
        )
    if _env("ANTHROPIC_API_KEY", "").strip() and allow_external:
        providers.append(
            Provider(id="anthropic", name="Anthropic", is_free=False, supports_vision=True, available=True, models=[])
        )
    if _env("GEMINI_API_KEY", "").strip() and allow_external:
        providers.append(
            Provider(id="gemini", name="Google Gemini", is_free=False, supports_vision=True, available=True, models=[])
        )
    if _env("PERPLEXITY_API_KEY", "").strip() and allow_external:
        providers.append(
            Provider(id="perplexity", name="Perplexity", is_free=False, supports_vision=False, available=True, models=[])
        )

    # 6) NullProvider fallback
    providers.append(
        Provider(
            id="null",
            name="Null (no IA)",
            is_free=True,
            supports_vision=False,
            available=True,
            models=[],
        )
    )
    return providers


def get_model_specs(ollama_available: bool) -> List[ModelSpec]:
    """Model specs from config; only include providers that are available."""
    specs: List[ModelSpec] = []
    route_keys = [
        ("FAST", "AI_FAST_MODEL"),
        ("CHAT", "AI_CHAT_MODEL"),
        ("CODE", "AI_CODE_MODEL"),
        ("REASON", "AI_REASON_MODEL"),
        ("TOOLS", "AI_TOOLS_MODEL"),
        ("VISION", "AI_VISION_MODEL"),
        ("ARCHITECT", "AI_ARCHITECT_MODEL"),
        ("OPTIMIZER", "AI_OPTIMIZER_MODEL"),
    ]
    for route, env_key in route_keys:
        full = _env(env_key, "")
        if not full:
            continue
        prov_id, model_name = _parse_model_key(full)
        is_free = prov_id in ("ollama", "null")
        if prov_id == "ollama" and not ollama_available:
            continue
        if prov_id in ("openai", "anthropic", "gemini", "perplexity") and not _env_bool("AI_ALLOW_EXTERNAL_APIS", False):
            continue
        specs.append(
            ModelSpec(
                provider_id=prov_id,
                model_name=model_name or full,
                full_key=full,
                is_free=is_free,
                route=route,
            )
        )
    # Ensure at least null fallback per route
    return specs


def resolve_model_for_route(route: str, ollama_available: bool, prefer_free: bool = True) -> Optional[ModelSpec]:
    """Return best ModelSpec for route; free-first if prefer_free."""
    specs = [s for s in get_model_specs(ollama_available) if s.route == route]
    if not specs:
        return None
    free = [s for s in specs if s.is_free]
    paid = [s for s in specs if not s.is_free]
    if prefer_free and free:
        return free[0]
    if paid:
        return paid[0]
    return free[0] if free else None

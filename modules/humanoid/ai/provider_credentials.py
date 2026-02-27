"""Almacén de API keys por proveedor (OpenAI, Anthropic, Gemini, Perplexity). Env tiene prioridad; las claves guardadas aquí se usan si no hay variable de entorno."""
from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Dict, Optional

_VALID_IDS = frozenset({"openai", "anthropic", "gemini", "perplexity", "groq", "xai", "deepseek", "mistral"})
_ENV_KEYS = {
    "openai": "OPENAI_API_KEY",
    "anthropic": "ANTHROPIC_API_KEY",
    "gemini": "GEMINI_API_KEY",
    "perplexity": "PERPLEXITY_API_KEY",
    "groq": "GROQ_API_KEY",
    "xai": "XAI_API_KEY",
    "deepseek": "DEEPSEEK_API_KEY",
    "mistral": "MISTRAL_API_KEY",
}

_store: Optional[Dict[str, str]] = None


def _store_path() -> Path:
    root = os.getenv("ATLAS_REPO_PATH") or os.getenv("ATLAS_PUSH_ROOT") or ""
    if root:
        return Path(root).resolve() / "config" / "provider_api_keys.json"
    return Path(__file__).resolve().parent.parent.parent.parent / "config" / "provider_api_keys.json"


def _load_store() -> Dict[str, str]:
    global _store
    if _store is not None:
        return _store
    p = _store_path()
    if not p.is_file():
        _store = {}
        return _store
    try:
        with open(p, "r", encoding="utf-8") as f:
            data = json.load(f)
        _store = {k: v for k, v in (data or {}).items() if k in _VALID_IDS and isinstance(v, str) and v.strip()}
        return _store
    except Exception:
        _store = {}
        return _store


def _save_store(data: Dict[str, str]) -> None:
    global _store
    p = _store_path()
    try:
        p.parent.mkdir(parents=True, exist_ok=True)
        with open(p, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
        _store = data
    except (OSError, IOError, PermissionError) as e:
        import logging
        logging.getLogger(__name__).warning("provider_credentials: no se pudo guardar en %s: %s", p, e)
        raise RuntimeError(
            "No se pudo guardar la clave en %s. Comprueba que la carpeta config existe y tiene permisos de escritura."
            % str(p)
        ) from e


def get_provider_api_key(provider_id: str) -> Optional[str]:
    """Devuelve la API key del proveedor: primero env, luego almacén. None si no está configurada."""
    if provider_id not in _VALID_IDS:
        return None
    env_key = _ENV_KEYS.get(provider_id)
    if env_key:
        v = (os.getenv(env_key) or "").strip()
        if v:
            return v
    return _load_store().get(provider_id) or None


def set_provider_api_key(provider_id: str, api_key: Optional[str]) -> None:
    """Guarda o borra la clave del proveedor en el almacén (solo archivo; env sigue teniendo prioridad al leer)."""
    if provider_id not in _VALID_IDS:
        raise ValueError("provider_id no válido: %s" % provider_id)
    data = dict(_load_store())
    if api_key and isinstance(api_key, str) and api_key.strip():
        data[provider_id] = api_key.strip()
    else:
        data.pop(provider_id, None)
    _save_store(data)


def get_credentials_status() -> Dict[str, Any]:
    """Estado por proveedor: configured (bool), masked (últimos 4 caracteres, ej. ***xyz1). Nunca devuelve la clave en claro."""
    out: Dict[str, Any] = {}
    for pid in _VALID_IDS:
        key = get_provider_api_key(pid)
        configured = bool(key and key.strip())
        masked = "***" + (key[-4:] if configured and len(key) >= 4 else "****")
        out[pid] = {"configured": configured, "masked": masked}
    return out

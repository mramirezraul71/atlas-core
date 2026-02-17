"""
Estado del cerebro: modo (auto/manual) e IA dominante.
Persistido en config para que el usuario decida qué IA usa.

Sistema Full Automático:
- mode: auto (cerebro decide) | manual (usuario decide)
- auto_route: True para enrutamiento inteligente
- specialists: modelos por tipo de tarea
- parameters: parámetros de generación
- features: características habilitadas
"""
from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Dict, Optional

_BRAIN_STATE: Optional[Dict[str, Any]] = None

# Configuración por defecto para sistema full
DEFAULT_STATE = {
    "mode": "auto",
    "override_model": None,
    "auto_route": True,
    "specialists": {
        "code": "ollama:deepseek-coder:6.7b",
        "vision": "ollama:llama3.2-vision:11b",
        "chat": "ollama:llama3.1:latest",
        "analysis": "ollama:deepseek-r1:14b",
        "creative": "ollama:llama3.1:latest",
    },
    "parameters": {
        "temperature": 0.7,
        "top_p": 0.9,
        "top_k": 40,
        "max_tokens": 2048,
        "repeat_penalty": 1.1,
    },
    "features": {
        "auto_fallback": True,
        "context_memory": "long_term",
        "specialist_routing": True,
    },
}


def _state_path() -> Path:
    root = os.getenv("ATLAS_REPO_PATH") or os.getenv("ATLAS_PUSH_ROOT") or ""
    if root:
        return Path(root).resolve() / "config" / "brain_state.json"
    return Path(__file__).resolve().parent.parent.parent.parent / "config" / "brain_state.json"


def _load() -> Dict[str, Any]:
    global _BRAIN_STATE
    if _BRAIN_STATE is not None:
        return _BRAIN_STATE
    p = _state_path()
    if not p.is_file():
        _BRAIN_STATE = dict(DEFAULT_STATE)
        return _BRAIN_STATE
    try:
        with open(p, "r", encoding="utf-8") as f:
            loaded = json.load(f)
            # Merge con defaults para campos faltantes
            _BRAIN_STATE = {**DEFAULT_STATE, **loaded}
            # Merge nested dicts
            for key in ("specialists", "parameters", "features"):
                if key in DEFAULT_STATE and key in loaded:
                    _BRAIN_STATE[key] = {**DEFAULT_STATE[key], **loaded.get(key, {})}
    except Exception:
        _BRAIN_STATE = dict(DEFAULT_STATE)
    return _BRAIN_STATE


def _save(state: Dict[str, Any]) -> None:
    try:
        p = _state_path()
        p.parent.mkdir(parents=True, exist_ok=True)
        with open(p, "w", encoding="utf-8") as f:
            json.dump(state, f, indent=2, ensure_ascii=False)
    except (OSError, IOError, PermissionError) as e:
        # Si no se puede escribir (permisos, disco, etc.), el estado queda solo en memoria
        import logging
        logging.getLogger(__name__).warning("brain_state: no se pudo guardar en %s: %s", _state_path(), e)


def get_brain_state() -> Dict[str, Any]:
    """Estado actual: mode (auto|manual), override_model (full_key o null)."""
    return dict(_load())


def set_brain_state(mode: Optional[str] = None, override_model: Optional[str] = None) -> Dict[str, Any]:
    """Actualiza modo y/o modelo. mode: 'auto' | 'manual'. override_model: full_key (ej. ollama:llama3.1:latest) o null."""
    state = _load()
    if mode is not None and isinstance(mode, str):
        state["mode"] = "manual" if mode.strip().lower() == "manual" else "auto"
    if override_model is not None and isinstance(override_model, str):
        state["override_model"] = override_model.strip() or None
    if state.get("mode") == "auto":
        state["override_model"] = None
    _save(state)
    global _BRAIN_STATE
    _BRAIN_STATE = state
    return get_brain_state()


def get_override_model_for_router() -> Optional[str]:
    """Si modo manual y hay override, devuelve el model_name para Ollama (sin prefijo provider)."""
    full = get_override_full_key()
    if not full or ":" not in full:
        return full
    return full.split(":", 1)[1].strip()


def get_override_full_key() -> Optional[str]:
    """Si modo manual y hay override, devuelve el full_key (ej. ollama:llama3.1:latest o gemini:gemini-1.5-flash)."""
    state = _load()
    if state.get("mode") != "manual":
        return None
    key = state.get("override_model")
    if not key or not isinstance(key, str):
        return None
    return key.strip() or None

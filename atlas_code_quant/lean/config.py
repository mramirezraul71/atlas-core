"""Configuración base para futuro adapter LEAN."""
from __future__ import annotations

import os
from dataclasses import dataclass


def _as_bool(raw: str | None, default: bool) -> bool:
    if raw is None:
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on"}


@dataclass(slots=True)
class LeanConfig:
    """Flags seguras para LEAN en F1 (todo OFF por defecto)."""

    enabled: bool = _as_bool(os.getenv("ATLAS_LEAN_ENABLED"), False)
    mode: str = os.getenv("ATLAS_LEAN_MODE", "external")
    docker_image: str = os.getenv(
        "ATLAS_LEAN_DOCKER_IMAGE", "quantconnect/lean:latest"
    )
    output_dir: str = os.getenv("ATLAS_LEAN_OUTPUT_DIR", "")

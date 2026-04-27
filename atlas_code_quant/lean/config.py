"""Configuración LEAN external mode (F4).

Hereda y amplía el scaffold F1 con paths reproducibles y validación de modo.
``ATLAS_LEAN_ENABLED=false`` por defecto: ningún subprocess se lanza
sin opt-in explícito del operador.
"""
from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Literal


def _as_bool(raw: str | None, default: bool) -> bool:
    if raw is None:
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on"}


LeanMode = Literal["external", "vendor", "docker"]
_VALID_MODES: tuple[LeanMode, ...] = ("external", "vendor", "docker")


@dataclass(slots=True)
class LeanConfig:
    """Configuración LEAN F4. Defaults conservadores y verificables."""

    enabled: bool = False
    mode: LeanMode = "external"
    docker_image: str = "quantconnect/lean:latest"
    output_dir: str = ""
    binary: str = "lean"
    project_dir: str = ""
    timeout_sec: int = 600

    @classmethod
    def from_env(cls) -> "LeanConfig":
        mode = os.getenv("ATLAS_LEAN_MODE", "external").strip().lower() or "external"
        if mode not in _VALID_MODES:
            mode = "external"
        return cls(
            enabled=_as_bool(os.getenv("ATLAS_LEAN_ENABLED"), False),
            mode=mode,  # type: ignore[arg-type]
            docker_image=os.getenv("ATLAS_LEAN_DOCKER_IMAGE", "quantconnect/lean:latest"),
            output_dir=os.getenv("ATLAS_LEAN_OUTPUT_DIR", ""),
            binary=os.getenv("ATLAS_LEAN_BINARY", "lean"),
            project_dir=os.getenv("ATLAS_LEAN_PROJECT_DIR", ""),
            timeout_sec=int(os.getenv("ATLAS_LEAN_TIMEOUT_SEC", "600") or "600"),
        )

    def output_path(self) -> Path | None:
        if not self.output_dir:
            return None
        return Path(self.output_dir)

    def is_external(self) -> bool:
        return self.mode == "external"

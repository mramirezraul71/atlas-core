"""Atlas Code Quant — LEAN adapter config (F13).

Configuración del wrapper LEAN externo. Por defecto **deshabilitado**
(``ATLAS_LEAN_ENABLED=False``): el adapter F13 jamás lanza un proceso
real cuando la flag está apagada, sólo emite resultados ``disabled``
honestos.

Reglas duras:

    * Default OFF. Habilitar requiere setear `ATLAS_LEAN_ENABLED=true`
      en environment + path válido a un binario LEAN.
    * Modo ``mock`` permite tests deterministas sin LEAN real.
    * No se invoca desde ``execution/`` ni ``operations/`` en F13.
    * El wrapper produce backtests / fitness; **NUNCA** órdenes reales.

Variables de entorno reconocidas:

    * ``ATLAS_LEAN_ENABLED`` — "true"/"false" (default ``false``)
    * ``ATLAS_LEAN_MODE``    — "external" | "mock" (default ``mock``
      cuando no hay binario; ``external`` cuando sí)
    * ``ATLAS_LEAN_BIN``     — path al CLI LEAN (subprocess)
    * ``ATLAS_LEAN_RESULTS_DIR`` — dir donde el subprocess deposita
      ``statistics.json`` / ``orders.json``
    * ``ATLAS_LEAN_TIMEOUT_SEC`` — timeout duro del subprocess

Ver:
    * docs/ATLAS_CODE_QUANT_F13_LEAN_ADAPTER_MVP.md
"""

from __future__ import annotations

import os
from dataclasses import dataclass


__all__ = [
    "LeanAdapterConfig",
    "load_config_from_env",
]


def _env_bool(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.strip().lower() in ("1", "true", "yes", "on")


def _env_int(name: str, default: int) -> int:
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        return int(raw)
    except ValueError:
        return default


@dataclass(frozen=True)
class LeanAdapterConfig:
    """Snapshot inmutable de la configuración del adapter."""

    enabled: bool = False
    mode: str = "mock"  # "mock" | "external"
    bin_path: str | None = None
    results_dir: str | None = None
    timeout_sec: int = 60

    @property
    def is_external(self) -> bool:
        return self.enabled and self.mode == "external" and bool(self.bin_path)

    @property
    def is_mock(self) -> bool:
        return self.mode == "mock"

    def to_dict(self) -> dict[str, object]:
        return {
            "enabled": self.enabled,
            "mode": self.mode,
            "bin_path": self.bin_path,
            "results_dir": self.results_dir,
            "timeout_sec": self.timeout_sec,
        }


def load_config_from_env() -> LeanAdapterConfig:
    """Construye la configuración leyendo variables de entorno.

    Si ``ATLAS_LEAN_ENABLED`` no está o es false → modo ``mock``.
    """
    enabled = _env_bool("ATLAS_LEAN_ENABLED", False)
    mode = (os.environ.get("ATLAS_LEAN_MODE") or "").strip().lower()
    if mode not in ("mock", "external"):
        mode = "external" if enabled and os.environ.get("ATLAS_LEAN_BIN") else "mock"
    bin_path = os.environ.get("ATLAS_LEAN_BIN") or None
    results_dir = os.environ.get("ATLAS_LEAN_RESULTS_DIR") or None
    timeout_sec = _env_int("ATLAS_LEAN_TIMEOUT_SEC", 60)
    return LeanAdapterConfig(
        enabled=enabled,
        mode=mode,
        bin_path=bin_path,
        results_dir=results_dir,
        timeout_sec=timeout_sec,
    )

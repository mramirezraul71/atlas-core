"""Lectura centralizada de feature flags F1 con defaults seguros."""
from __future__ import annotations

import os
from dataclasses import dataclass, field


def _env_bool(name: str, default: bool) -> bool:
    raw = os.getenv(name)
    if raw is None:
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on"}


def _env_int(name: str, default: int, *, lo: int | None = None, hi: int | None = None) -> int:
    raw = os.getenv(name)
    if raw is None:
        v = default
    else:
        try:
            v = int(raw.strip())
        except ValueError:
            v = default
    if lo is not None:
        v = max(lo, v)
    if hi is not None:
        v = min(hi, v)
    return v


@dataclass(slots=True)
class AtlasFeatureFlags:
    """Flags leídos del entorno en cada instanciación (tests y reload honestos)."""

    legacy_scanner_enabled: bool = field(
        default_factory=lambda: _env_bool("ATLAS_LEGACY_SCANNER_ENABLED", True)
    )
    lean_enabled: bool = field(default_factory=lambda: _env_bool("ATLAS_LEAN_ENABLED", False))
    vision_gate_enabled: bool = field(
        default_factory=lambda: _env_bool("ATLAS_VISION_GATE_ENABLED", False)
    )
    live_trading_enabled: bool = field(
        default_factory=lambda: _env_bool("ATLAS_LIVE_TRADING_ENABLED", False)
    )
    tradier_dry_run: bool = field(default_factory=lambda: _env_bool("ATLAS_TRADIER_DRY_RUN", True))
    kill_switch_file: str = field(
        default_factory=lambda: os.getenv("ATLAS_KILL_SWITCH_FILE", "")
    )
    # F2 Radar multi-símbolo (PUSH); default apagado.
    radar_multi_symbol_enabled: bool = field(
        default_factory=lambda: _env_bool("ATLAS_RADAR_MULTI_SYMBOL_ENABLED", False)
    )
    radar_max_symbols_per_batch: int = field(
        default_factory=lambda: _env_int(
            "ATLAS_RADAR_MAX_SYMBOLS_PER_BATCH", 100, lo=1, hi=5000
        )
    )
    radar_min_score: int = field(
        default_factory=lambda: _env_int("ATLAS_RADAR_MIN_SCORE", 80, lo=0, hi=100)
    )
    radar_universe_refresh_sec: int = field(
        default_factory=lambda: _env_int(
            "ATLAS_RADAR_UNIVERSE_REFRESH_SEC", 300, lo=15, hi=86400
        )
    )

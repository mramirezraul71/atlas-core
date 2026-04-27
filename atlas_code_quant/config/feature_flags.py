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


def _env_float(name: str, default: float, *, lo: float | None = None, hi: float | None = None) -> float:
    raw = os.getenv(name)
    if raw is None:
        v = default
    else:
        try:
            v = float(raw.strip())
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
        default_factory=lambda: _env_bool("ATLAS_LEGACY_SCANNER_ENABLED", False)
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
    # F3: intake de oportunidades desde Radar (cutover scanner).
    radar_intake_enabled: bool = field(
        default_factory=lambda: _env_bool("ATLAS_RADAR_INTAKE_ENABLED", True)
    )
    radar_opportunities_url: str = field(
        default_factory=lambda: os.getenv(
            "ATLAS_RADAR_OPPORTUNITIES_URL",
            "http://127.0.0.1:8791/api/radar/opportunities",
        ).strip()
    )
    radar_stream_url: str = field(
        default_factory=lambda: os.getenv(
            "ATLAS_RADAR_STREAM_URL",
            "http://127.0.0.1:8791/api/radar/stream/opportunities",
        ).strip()
    )
    radar_intake_timeout_sec: float = field(
        default_factory=lambda: _env_float("ATLAS_RADAR_INTAKE_TIMEOUT_SEC", 4.0, lo=0.5, hi=60.0)
    )
    radar_intake_poll_sec: int = field(
        default_factory=lambda: _env_int("ATLAS_RADAR_INTAKE_POLL_SEC", 60, lo=5, hi=3600)
    )
    radar_intake_limit: int = field(
        default_factory=lambda: _env_int("ATLAS_RADAR_INTAKE_LIMIT", 24, lo=1, hi=500)
    )

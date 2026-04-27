"""Lectura centralizada de feature flags F1 con defaults seguros."""
from __future__ import annotations

import os
from dataclasses import dataclass


def _env_bool(name: str, default: bool) -> bool:
    raw = os.getenv(name)
    if raw is None:
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on"}


@dataclass(slots=True)
class AtlasFeatureFlags:
    legacy_scanner_enabled: bool = _env_bool("ATLAS_LEGACY_SCANNER_ENABLED", True)
    lean_enabled: bool = _env_bool("ATLAS_LEAN_ENABLED", False)
    vision_gate_enabled: bool = _env_bool("ATLAS_VISION_GATE_ENABLED", False)
    live_trading_enabled: bool = _env_bool("ATLAS_LIVE_TRADING_ENABLED", False)
    tradier_dry_run: bool = _env_bool("ATLAS_TRADIER_DRY_RUN", True)
    kill_switch_file: str = os.getenv("ATLAS_KILL_SWITCH_FILE", "")

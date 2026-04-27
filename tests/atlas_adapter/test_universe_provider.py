"""Universo optionable F2: catálogo y búsqueda local."""
from __future__ import annotations

import os

import pytest

from atlas_adapter.services.universe_provider import UniverseEntry, UniverseProvider
from atlas_code_quant.config.feature_flags import AtlasFeatureFlags


def test_universe_returns_optionable_symbols(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("ATLAS_RADAR_UNIVERSE_REFRESH_SEC", "30")
    up = UniverseProvider(flags=AtlasFeatureFlags())
    u = up.refresh(force=True)
    assert len(u) >= 8
    symbols = {e.symbol for e in u}
    assert "SPY" in symbols
    assert all(e.optionable for e in u)
    assert all(isinstance(e, UniverseEntry) for e in u)


def test_universe_respects_max_batch(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("ATLAS_RADAR_MAX_SYMBOLS_PER_BATCH", "5")
    flags = AtlasFeatureFlags()
    up = UniverseProvider(flags=flags)
    capped = up.get_optionable_universe()
    assert len(capped) <= 5


def test_search_prefix(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("ATLAS_RADAR_UNIVERSE_REFRESH_SEC", "1")
    up = UniverseProvider(flags=AtlasFeatureFlags())
    up.refresh(force=True)
    hits = up.search("SP", limit=10)
    assert any(h.symbol == "SPY" for h in hits)


def test_refresh_ttl_cache(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("ATLAS_RADAR_UNIVERSE_REFRESH_SEC", "3600")
    up = UniverseProvider(flags=AtlasFeatureFlags())
    a = up.refresh(force=True)
    b = up.refresh(force=False)
    assert len(a) == len(b)

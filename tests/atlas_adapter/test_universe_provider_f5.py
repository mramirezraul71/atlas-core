"""F5 — Tests del universe provider determinista del Radar.

Verifica:
    * El universo por defecto NO está vacío y es determinista entre llamadas.
    * Filtros AND (asset_class, sector, optionable, enabled_only, symbols).
    * ``limit`` recorta sin reordenar.
    * Lookup case-insensitive y subset por símbolos respeta orden recibido.
"""
from __future__ import annotations

from atlas_adapter.services.universe_provider import (
    DEFAULT_UNIVERSE,
    UniverseEntry,
    UniverseProvider,
    default_universe_provider,
)


def test_default_universe_is_non_empty_and_stable() -> None:
    a = default_universe_provider().all()
    b = default_universe_provider().all()
    assert a is b  # singleton lazy
    assert len(a) >= 30
    # Determinismo: orden estable entre instancias frescas.
    fresh1 = UniverseProvider().all()
    fresh2 = UniverseProvider().all()
    assert tuple(e.symbol for e in fresh1) == tuple(e.symbol for e in fresh2)


def test_default_universe_has_expected_categories() -> None:
    p = UniverseProvider()
    classes = set(p.asset_classes())
    assert {"etf", "index", "equity"}.issubset(classes)
    # Símbolos canónicos críticos presentes.
    syms = {e.symbol for e in p.all()}
    for must in ("SPY", "QQQ", "VIX", "AAPL", "NVDA"):
        assert must in syms, f"falta símbolo crítico {must!r}"


def test_index_entries_marked_non_optionable() -> None:
    p = UniverseProvider()
    indices = [e for e in p.all() if e.asset_class == "index"]
    assert indices
    for e in indices:
        assert e.optionable is False, f"{e.symbol} debería marcarse no-optionable"


def test_filter_asset_class_only_returns_matching() -> None:
    p = UniverseProvider()
    etfs = p.list(asset_class="etf")
    assert etfs and all(e.asset_class == "etf" for e in etfs)


def test_filter_sector_case_insensitive() -> None:
    p = UniverseProvider()
    tech = p.list(sector="TECHNOLOGY")
    assert tech
    assert all((e.sector or "").lower() == "technology" for e in tech)


def test_filter_optionable_true_excludes_indices() -> None:
    p = UniverseProvider()
    only_opt = p.list(optionable=True)
    assert only_opt
    assert all(e.optionable for e in only_opt)
    assert all(e.asset_class != "index" for e in only_opt)


def test_limit_truncates_without_reordering() -> None:
    p = UniverseProvider()
    full = p.list()
    cut = p.list(limit=5)
    assert len(cut) == 5
    assert tuple(e.symbol for e in cut) == tuple(e.symbol for e in full[:5])


def test_subset_symbols_respects_input_order_and_skips_unknown() -> None:
    p = UniverseProvider()
    out = p.list(symbols=["NVDA", "spy", "ZZZNOTREAL", "AAPL"])
    assert [e.symbol for e in out] == ["NVDA", "SPY", "AAPL"]


def test_disabled_entries_excluded_by_default() -> None:
    custom = (
        UniverseEntry("AAA", "Test A", "etf", "broad_market", True, True),
        UniverseEntry("BBB", "Test B", "etf", "broad_market", True, False),
    )
    p = UniverseProvider(custom)
    syms = [e.symbol for e in p.list()]
    assert syms == ["AAA"]
    syms_all = [e.symbol for e in p.list(enabled_only=False)]
    assert syms_all == ["AAA", "BBB"]


def test_get_is_case_insensitive() -> None:
    p = UniverseProvider()
    assert p.get("spy") is not None
    assert p.get("SPY") is not None
    assert p.get("nonexistent_xxx") is None


def test_default_universe_module_constant_immutable_tuple() -> None:
    # Garantía estática: DEFAULT_UNIVERSE es tuple → inmutable.
    assert isinstance(DEFAULT_UNIVERSE, tuple)
    assert all(isinstance(e, UniverseEntry) for e in DEFAULT_UNIVERSE)

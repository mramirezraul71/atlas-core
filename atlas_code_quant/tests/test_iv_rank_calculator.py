"""Tests unitarios para IV Rank (sin llamadas de red reales)."""
from __future__ import annotations

import math
from datetime import date, timedelta
from unittest.mock import MagicMock

import numpy as np
import pytest

from options.iv_rank_calculator import (
    IVRankCalculator,
    annualized_rv_from_closes,
    atm_iv_from_chain,
    compute_iv_rank_linear,
    pick_expiration_for_dte,
)


def test_compute_iv_rank_linear_nominal() -> None:
    r, flag = compute_iv_rank_linear(0.15, 0.10, 0.20)
    assert flag == "ok"
    assert abs(r - 50.0) < 1e-6


def test_compute_iv_rank_clamp() -> None:
    lo, hi = 0.1, 0.2
    r0, _ = compute_iv_rank_linear(0.05, lo, hi)
    r1, _ = compute_iv_rank_linear(0.99, lo, hi)
    assert r0 == 0.0
    assert r1 == 100.0


def test_compute_iv_rank_degenerate() -> None:
    r, flag = compute_iv_rank_linear(0.2, 0.2, 0.2)
    assert flag == "degenerate_window"
    assert r == 50.0


def test_pick_expiration_for_dte() -> None:
    today = date(2026, 4, 1)
    exps = ["2026-04-05", "2026-04-20", "2026-06-15"]
    exp, dte, err = pick_expiration_for_dte(exps, 14, today=today)
    assert err is None
    assert exp == "2026-04-20"
    assert dte == 19


def test_pick_expiration_empty() -> None:
    exp, dte, err = pick_expiration_for_dte([], 7)
    assert exp is None
    assert err == "no_expirations"


def test_atm_iv_from_chain_median() -> None:
    spot = 100.0
    chain = [
        {"strike": 98.0, "greeks": {"mid_iv": 0.22}},
        {"strike": 100.0, "greeks": {"mid_iv": 0.25}},
        {"strike": 100.0, "greeks": {"mid_iv": 0.24}},
        {"strike": 102.0, "greeks": {"mid_iv": 0.26}},
    ]
    iv, n = atm_iv_from_chain(chain, spot)
    assert n >= 1
    assert iv is not None
    assert 0.22 <= iv <= 0.26


def test_annualized_rv_from_closes_shape() -> None:
    rng = np.random.default_rng(0)
    closes = 100.0 * np.exp(np.cumsum(rng.normal(0, 0.01, size=120)))
    rv = annualized_rv_from_closes(closes, 21)
    assert rv.size == len(closes) - 21
    assert np.all(rv > 0)


def _make_mock_client(
    *,
    spot: float = 100.0,
    chain_iv: float = 0.30,
    n_hist: int = 300,
) -> MagicMock:
    client = MagicMock()
    client.quote.return_value = {"last": spot}

    def fake_exp(sym: str, scope: str | None = None) -> list[str]:
        del sym, scope
        d = date.today() + timedelta(days=30)
        return [d.isoformat()]

    def fake_chain(sym: str, expiration: str, scope: str | None = None) -> list[dict]:
        del sym, expiration, scope
        strikes = [95.0, 100.0, 105.0]
        out = []
        for k in strikes:
            for ot in ("call", "put"):
                out.append(
                    {
                        "strike": k,
                        "option_type": ot,
                        "greeks": {"mid_iv": chain_iv},
                    }
                )
        return out

    def fake_hist(
        sym: str,
        start: date,
        end: date,
        interval: str = "daily",
    ) -> list[dict]:
        del sym, interval
        days = (end - start).days
        rows = []
        p = 100.0
        rng = np.random.default_rng(42)
        for i in range(max(days, n_hist)):
            p *= math.exp(float(rng.normal(0, 0.012)))
            rows.append({"close": p})
        return rows[-n_hist:]

    client.get_option_expirations.side_effect = fake_exp
    client.get_option_chain.side_effect = fake_chain
    client.history.side_effect = fake_hist
    return client


def test_iv_rank_calculator_full_mock() -> None:
    client = _make_mock_client(chain_iv=0.28)
    cache = MagicMock()
    cache.get_expirations = MagicMock(
        side_effect=lambda s, c, scope="paper": client.get_option_expirations(s, scope)
    )
    cache.get_or_fetch = MagicMock(
        side_effect=lambda s, e, c, scope="paper": client.get_option_chain(s, e, scope)
    )

    calc = IVRankCalculator(
        client,
        chain_cache=cache,
        scope="paper",
        lookback_trading_days=280,
        rv_window=21,
        min_rv_samples=30,
    )
    out = calc.get_iv_rank("SPY", dte_target=30)
    assert out["symbol"] == "SPY"
    assert out["iv_current"] is not None
    assert out["iv_rank"] is not None
    assert out["quality"] in {"ok", "approx"}
    assert out["iv_hv_ratio"] is not None


def test_iv_rank_insufficient_history_short_chain() -> None:
    client = _make_mock_client(n_hist=300)
    cache = MagicMock()
    cache.get_expirations = MagicMock(return_value=[(date.today() + timedelta(days=20)).isoformat()])
    cache.get_or_fetch = MagicMock(return_value=client.get_option_chain("X", "2026-05-01"))

    calc = IVRankCalculator(client, chain_cache=cache, min_rv_samples=500)
    out = calc.get_iv_rank("X", dte_target=7)
    assert out["quality"] == "insufficient_history"
    assert out["iv_rank"] is None


def test_iv_rank_degenerate_rv_window(monkeypatch: pytest.MonkeyPatch) -> None:
    """Fuerza min==max en serie RV para cubrir rama degenerada."""
    client = _make_mock_client(chain_iv=0.25)
    cache = MagicMock()
    cache.get_expirations = MagicMock(
        side_effect=lambda s, c, scope="paper": client.get_option_expirations(s, scope)
    )
    cache.get_or_fetch = MagicMock(
        side_effect=lambda s, e, c, scope="paper": client.get_option_chain(s, e, scope)
    )

    def flat_rv(*args: object, **kwargs: object) -> np.ndarray:
        return np.ones(80) * 0.5

    import options.iv_rank_calculator as irc_mod

    monkeypatch.setattr(irc_mod, "annualized_rv_from_closes", flat_rv)

    calc = IVRankCalculator(
        client,
        chain_cache=cache,
        min_rv_samples=30,
    )
    out = calc.get_iv_rank("SPY", dte_target=30)
    assert out["iv_rank"] == 50.0
    assert out["quality"] == "approx"
    assert out.get("warnings")


def test_option_chain_cache_real_class_integration() -> None:
    """OptionChainCache real con cliente mock (sin HTTP)."""
    try:
        from execution.option_chain_cache import OptionChainCache
    except ModuleNotFoundError:
        from atlas_code_quant.execution.option_chain_cache import OptionChainCache

    client = _make_mock_client()
    cache = OptionChainCache(ttl_sec=3600)
    calc = IVRankCalculator(client, chain_cache=cache, min_rv_samples=40)
    out = calc.get_iv_rank("QQQ", dte_target=25)
    assert out["iv_current"] is not None

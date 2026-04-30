from __future__ import annotations

from datetime import datetime, timedelta, timezone
from typing import Any

from atlas_code_quant.options.options_pipeline import (
    build_optionable_universe,
    rank_and_deduplicate_opportunities,
    run_options_trading_pipeline,
)


def _iso(days: int) -> str:
    return (datetime.now(timezone.utc) + timedelta(days=days)).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def _chain_for(price: float) -> list[dict[str, Any]]:
    exp_front = _iso(30)
    exp_back = _iso(60)
    rows: list[dict[str, Any]] = []
    put_deltas = {
        round(price * 0.9, 2): -0.08,
        round(price * 0.95, 2): -0.18,
        round(price * 1.0, 2): -0.50,
        round(price * 1.05, 2): -0.72,
        round(price * 1.1, 2): -0.90,
    }
    call_deltas = {
        round(price * 0.9, 2): 0.90,
        round(price * 0.95, 2): 0.72,
        round(price * 1.0, 2): 0.50,
        round(price * 1.05, 2): 0.18,
        round(price * 1.1, 2): 0.08,
    }
    for expiry in (exp_front, exp_back):
        for strike in [price * 0.9, price * 0.95, price * 1.0, price * 1.05, price * 1.1]:
            strike = round(strike, 2)
            rows.append(
                {
                    "expiry": expiry,
                    "type": "put",
                    "strike": strike,
                    "delta": round(put_deltas[strike], 3),
                    "bid": 1.2,
                    "ask": 1.22,
                    "open_interest": 12000,
                    "volume": 7000,
                    "iv": 0.24,
                }
            )
            rows.append(
                {
                    "expiry": expiry,
                    "type": "call",
                    "strike": strike,
                    "delta": round(call_deltas[strike], 3),
                    "bid": 1.1,
                    "ask": 1.12,
                    "open_interest": 13000,
                    "volume": 7500,
                    "iv": 0.23,
                }
            )
    return rows


class FakeAtlas:
    def __init__(self, *, contracts: int = 1) -> None:
        self.contracts = contracts
        self.events: list[dict[str, Any]] = []
        self.signals: list[dict[str, Any]] = []
        self.positions: list[dict[str, Any]] = []
        self._meta = {
            "SPY": {
                "asset_class": "ETF",
                "sector": "broad_market",
                "avg_option_volume_20d": 40000,
                "avg_bid_ask_pct": 0.012,
                "avg_open_interest_20d": 100000,
                "has_weeklies": True,
                "earnings_date": None,
                "market_cap": None,
                "beta": 1.0,
                "price": 500,
            },
            "AAPL": {
                "asset_class": "EQUITY",
                "sector": "technology",
                "avg_option_volume_20d": 25000,
                "avg_bid_ask_pct": 0.020,
                "avg_open_interest_20d": 35000,
                "has_weeklies": True,
                "earnings_date": None,
                "market_cap": 2800000000000,
                "beta": 1.3,
                "price": 210,
            },
            "ILLQ": {
                "asset_class": "EQUITY",
                "sector": "small_cap",
                "avg_option_volume_20d": 80,
                "avg_bid_ask_pct": 0.12,
                "avg_open_interest_20d": 120,
                "has_weeklies": False,
                "earnings_date": None,
                "market_cap": 500000000,
                "beta": 1.9,
                "price": 22,
            },
        }

    def get_global_regime(self) -> dict[str, Any]:
        return {"event_risk": False, "regime_id": "range_risk_on", "tradable_today": True}

    def get_optionable_universe(self) -> list[str]:
        return list(self._meta.keys())

    def filter_liquid_symbols(self, symbols: list[str]) -> list[str]:
        return symbols

    def get_asset_metadata(self, symbol: str) -> dict[str, Any]:
        return dict(self._meta[symbol])

    def get_vol_regime(self, symbol: str) -> dict[str, Any]:
        return {"iv_rank": 64.0, "vrp_20d": 9.0, "term_structure_slope": 0.9, "skew_25d": 6.0}

    def get_gex_surface(self, symbol: str) -> dict[str, Any]:
        return {"net_gex": -200.0, "gamma_flip_distance_pct": 1.0, "max_pain_distance_pct": 0.8}

    def get_oi_flow(self, symbol: str) -> dict[str, Any]:
        return {"oi_change_1d_pct": 12.0, "call_put_volume_ratio": 1.0, "max_pain_distance_pct": 0.7}

    def get_price_regime(self, symbol: str) -> dict[str, Any]:
        return {"adx": 20.0, "ema_alignment": "mixed", "breakout_status": "range"}

    def get_seasonal_multiplier(self, symbol: str, current_time: datetime) -> float:
        return 1.0

    def get_option_chain(self, symbol: str) -> list[dict[str, Any]]:
        return _chain_for(self._meta[symbol]["price"])

    def get_portfolio_state(self) -> dict[str, Any]:
        return {"open_positions": [], "net_delta": 0.0, "net_vega": 0.0, "symbol_exposure": {}, "sector_exposure": {}}

    def get_correlation_matrix(self) -> dict[str, Any]:
        return {}

    def calculate_position_sizing(self, **kwargs: Any) -> dict[str, Any]:
        return {"contracts": self.contracts, "risk_pct": 0.008, "risk_dollars": 80.0}

    def has_recent_signal(self, symbol: str, strategy: str, expiry: str, within_minutes: int = 30) -> bool:
        return False

    def emit_signal(self, signal: dict[str, Any]) -> None:
        self.signals.append(signal)

    def create_position(self, position: dict[str, Any], exit_rules: dict[str, Any]) -> dict[str, Any]:
        payload = {"position_id": f"pos-{len(self.positions)+1}", **position, "exit_rules": exit_rules}
        self.positions.append(payload)
        return payload

    def log_event(self, event: dict[str, Any]) -> None:
        self.events.append(event)


def test_build_optionable_universe_filters_illiquid_symbols():
    atlas = FakeAtlas()
    universe = build_optionable_universe(atlas)
    symbols = {u.symbol for u in universe}
    assert "SPY" in symbols
    assert "AAPL" in symbols
    assert "ILLQ" not in symbols


def test_rank_and_deduplicate_opportunities_limits_duplicates():
    opportunities = [
        {"symbol": "SPY", "strategy": "IRON_CONDOR", "expiry": "2026-06-19", "score": 90, "sector": "broad", "asset_family": "ETF", "direction": "NEUTRAL"},
        {"symbol": "SPY", "strategy": "IRON_CONDOR", "expiry": "2026-06-19", "score": 88, "sector": "broad", "asset_family": "ETF", "direction": "NEUTRAL"},
        {"symbol": "AAPL", "strategy": "PUT_CREDIT_SPREAD", "expiry": "2026-06-19", "score": 87, "sector": "technology", "asset_family": "LARGE_CAP_EQUITY", "direction": "BULL"},
    ]
    ranked = rank_and_deduplicate_opportunities(opportunities, portfolio_state={"open_positions": []}, limits={"max_trades_per_cycle": 5})
    assert len(ranked) == 2
    assert ranked[0]["symbol"] == "SPY"


def test_run_options_trading_pipeline_emits_and_creates_positions():
    atlas = FakeAtlas(contracts=1)
    out = run_options_trading_pipeline(datetime.now(timezone.utc), atlas, limits={"max_trades_per_cycle": 3})
    assert out["ok"] is True
    assert out["emitted"] >= 1
    assert out["created"] >= 1
    assert len(atlas.signals) == out["emitted"]
    assert len(atlas.positions) == out["created"]
    assert any(e.get("event_type") == "entry_execution" for e in atlas.events)


def test_run_options_trading_pipeline_blocks_when_contracts_zero():
    atlas = FakeAtlas(contracts=0)
    out = run_options_trading_pipeline(datetime.now(timezone.utc), atlas, limits={"max_trades_per_cycle": 2})
    assert out["blocked"] >= 1
    assert out["created"] == 0
    assert any(e.get("event_type") == "entry_blocked" for e in atlas.events)


def test_run_options_trading_pipeline_blocks_when_grok_rejects(monkeypatch):
    from atlas_code_quant.options import options_pipeline as module

    monkeypatch.setenv("QUANT_GROK_REVIEW_ENABLED", "true")
    atlas = FakeAtlas(contracts=1)
    monkeypatch.setattr(
        module.GrokExpertReviewProvider,
        "review_trade",
        lambda self, decision_pack: {
            "provider": "grok",
            "status": "ok",
            "verdict": "reject",
            "score_adjustment": None,
            "contracts_multiplier": None,
            "prefer_strategy": None,
            "rationale": "reject_for_test",
            "error": None,
        },
    )
    out = run_options_trading_pipeline(datetime.now(timezone.utc), atlas, limits={"max_trades_per_cycle": 2})
    assert out["created"] == 0
    assert out["blocked"] >= 1
    assert any(e.get("event_type") == "grok_review" for e in atlas.events)
    assert any(e.get("reason") == "grok_reject" for e in atlas.events if e.get("event_type") == "entry_blocked")


def test_run_options_trading_pipeline_applies_grok_contracts_multiplier(monkeypatch):
    from atlas_code_quant.options import options_pipeline as module

    monkeypatch.setenv("QUANT_GROK_REVIEW_ENABLED", "true")
    atlas = FakeAtlas(contracts=4)
    monkeypatch.setattr(
        module.GrokExpertReviewProvider,
        "review_trade",
        lambda self, decision_pack: {
            "provider": "grok",
            "status": "ok",
            "verdict": "approve",
            "score_adjustment": None,
            "contracts_multiplier": 0.5,
            "prefer_strategy": None,
            "rationale": "halve_size_for_test",
            "error": None,
        },
    )
    out = run_options_trading_pipeline(datetime.now(timezone.utc), atlas, limits={"max_trades_per_cycle": 1})
    assert out["created"] == 1
    assert atlas.positions[0]["contracts"] == 2


def test_run_options_trading_pipeline_skips_grok_when_flag_disabled(monkeypatch):
    from atlas_code_quant.options import options_pipeline as module

    monkeypatch.setenv("QUANT_GROK_REVIEW_ENABLED", "false")
    atlas = FakeAtlas(contracts=1)
    monkeypatch.setattr(
        module.GrokExpertReviewProvider,
        "_request_review_text",
        lambda self, decision_pack: (_ for _ in ()).throw(AssertionError("grok_api_should_not_be_called")),
    )
    out = run_options_trading_pipeline(datetime.now(timezone.utc), atlas, limits={"max_trades_per_cycle": 1})
    assert out["created"] == 1
    assert any(e.get("event_type") == "grok_review" and e.get("error") == "grok_review_disabled" for e in atlas.events)


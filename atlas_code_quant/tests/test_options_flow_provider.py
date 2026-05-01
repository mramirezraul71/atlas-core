from __future__ import annotations

import sqlite3
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
QUANT_ROOT = ROOT / "atlas_code_quant"
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from execution.option_chain_cache import OptionChainCache  # noqa: E402
from scanner.options_flow_provider import MarketTelemetryStore, OptionsFlowProvider  # noqa: E402
from scanner.opportunity_scanner import OpportunityScannerService  # noqa: E402


class _ClientStub:
    def quote(self, symbol: str) -> dict:
        assert symbol == "AAPL"
        return {
            "last": 100.0,
            "bid": 99.9,
            "ask": 100.1,
            "bidsize": 180,
            "asksize": 120,
        }

    def expirations(self, symbol: str) -> list[str]:
        assert symbol == "AAPL"
        return ["2099-01-21", "2099-02-18"]

    def chain(self, symbol: str, expiration: str) -> list[dict]:
        assert symbol == "AAPL"
        if expiration == "2099-01-21":
            return [
                {
                    "symbol": "AAPL20990121C00095000",
                    "option_type": "call",
                    "strike": 95.0,
                    "volume": 320,
                    "open_interest": 900,
                    "bid": 6.1,
                    "ask": 6.4,
                    "greeks": {"mid_iv": 0.23, "gamma": 0.022},
                },
                {
                    "symbol": "AAPL20990121C00100000",
                    "option_type": "call",
                    "strike": 100.0,
                    "volume": 460,
                    "open_interest": 1200,
                    "bid": 3.2,
                    "ask": 3.5,
                    "greeks": {"mid_iv": 0.21, "gamma": 0.028},
                },
                {
                    "symbol": "AAPL20990121P00100000",
                    "option_type": "put",
                    "strike": 100.0,
                    "volume": 180,
                    "open_interest": 700,
                    "bid": 2.8,
                    "ask": 3.0,
                    "greeks": {"mid_iv": 0.24, "gamma": 0.024},
                },
                {
                    "symbol": "AAPL20990121P00105000",
                    "option_type": "put",
                    "strike": 105.0,
                    "volume": 120,
                    "open_interest": 620,
                    "bid": 5.8,
                    "ask": 6.1,
                    "greeks": {"mid_iv": 0.26, "gamma": 0.020},
                },
            ]
        return [
            {
                "symbol": "AAPL20990218C00100000",
                "option_type": "call",
                "strike": 100.0,
                "volume": 240,
                "open_interest": 820,
                "bid": 4.8,
                "ask": 5.2,
                "greeks": {"mid_iv": 0.25, "gamma": 0.018},
            },
            {
                "symbol": "AAPL20990218P00100000",
                "option_type": "put",
                "strike": 100.0,
                "volume": 190,
                "open_interest": 760,
                "bid": 4.4,
                "ask": 4.8,
                "greeks": {"mid_iv": 0.27, "gamma": 0.017},
            },
        ]


def test_options_flow_provider_builds_bullish_snapshot_and_persists_sqlite(tmp_path: Path) -> None:
    store = MarketTelemetryStore(
        backend="sqlite",
        sqlite_path=tmp_path / "market_telemetry.sqlite3",
        enabled=True,
    )
    provider = OptionsFlowProvider(
        telemetry_store=store,
        chain_cache=OptionChainCache(ttl_sec=60),
    )

    snapshot = provider.build_snapshot(symbol="AAPL", client=_ClientStub(), scope="paper", price_hint=100.0)

    assert snapshot["available"] is True
    assert snapshot["direction"] == "alcista"
    assert snapshot["reason"] == "options_flow_ok"
    assert snapshot["contracts_evaluated"] >= 6
    assert snapshot["telemetry_write"]["ok"] is True
    assert snapshot["telemetry_write"]["backend"] == "sqlite"

    with sqlite3.connect(tmp_path / "market_telemetry.sqlite3") as conn:
        rows = conn.execute("SELECT COUNT(*) FROM order_flow_snapshots").fetchone()[0]
    assert rows == 1


def test_options_flow_provider_rejects_symbols_without_listed_options(tmp_path: Path) -> None:
    provider = OptionsFlowProvider(
        telemetry_store=MarketTelemetryStore(
            backend="sqlite",
            sqlite_path=tmp_path / "market_telemetry.sqlite3",
            enabled=True,
        )
    )

    snapshot = provider.build_snapshot(symbol="BTC/USDT", client=_ClientStub(), scope="paper", price_hint=80000.0)

    assert snapshot["available"] is False
    assert snapshot["reason"] == "asset_without_listed_options"


def test_scanner_merge_order_flow_snapshots_prefers_hybrid_when_both_sources_exist() -> None:
    svc = OpportunityScannerService()
    intraday = {
        "available": True,
        "scope": "paper",
        "mode": "proxy_intradia",
        "direction": "alcista",
        "score_pct": 62.0,
        "confidence_pct": 38.0,
        "net_pressure_pct": 12.0,
        "price_vs_vwap_pct": 0.9,
        "spread_bps": 8.0,
        "quote_imbalance_pct": 14.0,
        "last_price": 100.0,
        "vwap": 99.1,
        "reason": "order_flow_proxy_ok",
    }
    options_flow = {
        "available": True,
        "scope": "paper",
        "mode": "options_flow",
        "direction": "alcista",
        "score_pct": 78.0,
        "confidence_pct": 66.0,
        "put_call_volume_ratio": 0.52,
        "put_call_oi_ratio": 0.74,
        "iv_term_structure_slope_pct": 8.4,
        "atm_skew_pct": 2.8,
        "gamma_bias_pct": 18.0,
        "front_dte": 12,
        "back_dte": 40,
        "expirations_used": ["2099-01-21", "2099-02-18"],
        "contracts_evaluated": 48,
        "reason": "options_flow_ok",
    }

    merged = svc._merge_order_flow_snapshots(intraday, options_flow)

    assert merged["available"] is True
    assert merged["mode"] == "hybrid_options_intradia"
    assert merged["direction"] == "alcista"
    assert merged["score_pct"] > intraday["score_pct"]
    assert merged["score_pct"] < options_flow["score_pct"]
    assert merged["put_call_volume_ratio"] == 0.52
    assert merged["options_flow"]["reason"] == "options_flow_ok"

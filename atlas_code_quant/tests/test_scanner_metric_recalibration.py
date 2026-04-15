from __future__ import annotations

import sys
from pathlib import Path

import pandas as pd


ROOT = Path(__file__).resolve().parents[2]
QUANT_ROOT = ROOT / "atlas_code_quant"
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from scanner.opportunity_scanner import OpportunityScannerService, _stable_symbol_mix  # noqa: E402


def _sample_df(rows: int = 260) -> pd.DataFrame:
    index = pd.date_range("2025-01-01", periods=rows, freq="D", tz="UTC")
    close = pd.Series([100.0 + (i * 0.4) for i in range(rows)], index=index)
    return pd.DataFrame(
        {
            "open": close - 0.2,
            "high": close + 0.5,
            "low": close - 0.5,
            "close": close,
            "volume": 1_000_000.0,
        },
        index=index,
    )


def test_backtest_method_recent_exposes_quality_metrics(monkeypatch) -> None:
    scanner = OpportunityScannerService()

    monkeypatch.setattr(
        scanner,
        "_method_signal",
        lambda method, df, symbol, timeframe: {
            "direction": 1,
            "strength": 0.8,
            "reasons": ["synthetic trend"],
            "price": float(df["close"].iloc[-1]),
        },
    )

    perf = scanner._backtest_method_recent("trend_ema_stack", _sample_df(), "AAPL", "1d")

    assert perf["sample"] > 0
    assert perf["win_rate_pct"] > 0
    assert "expectancy_pct" in perf
    assert "avg_win_pct" in perf
    assert "avg_loss_pct" in perf
    assert "payoff_ratio" in perf
    assert "quality_score_pct" in perf
    assert perf["quality_score_pct"] > 0


def test_evaluate_symbol_timeframes_rejects_low_profit_factor_even_with_signal(monkeypatch) -> None:
    scanner = OpportunityScannerService()
    monkeypatch.setattr(scanner.learning, "context", lambda **kwargs: {"total_bias": 0.0})
    monkeypatch.setattr(
        scanner,
        "_method_signal",
        lambda method, df, symbol, timeframe: {
            "direction": 1,
            "strength": 0.82,
            "reasons": ["synthetic trend"],
            "price": float(df["close"].iloc[-1]),
        },
    )
    monkeypatch.setattr(
        scanner,
        "_backtest_method_recent",
        lambda method, df, symbol, timeframe: {
            "sample": 40,
            "wins": 28,
            "losses": 12,
            "win_rate_pct": 70.0,
            "profit_factor": 0.92,
            "expectancy_pct": -0.08,
            "avg_win_pct": 0.22,
            "avg_loss_pct": 0.41,
            "payoff_ratio": 0.537,
            "sample_confidence_pct": 100.0,
            "profit_factor_score_pct": 8.57,
            "expectancy_score_pct": 24.44,
            "payoff_score_pct": 0.0,
            "quality_score_pct": 34.2,
        },
    )

    accepted, rejected = scanner._evaluate_symbol_timeframes(
        "AAPL",
        {"1d": _sample_df()},
        relative_strength_pct=88.0,
        order_flow={},
    )

    assert accepted == []
    assert rejected
    assert any("profit factor local" in reason for reason in rejected[0]["reasons"])


def test_report_does_not_deadlock_when_status_snapshot_is_embedded() -> None:
    scanner = OpportunityScannerService()

    report = scanner.report(activity_limit=5)

    assert "status" in report
    assert report["status"]["running"] is False
    assert isinstance(report["activity"], list)


def test_stable_symbol_mix_breaks_alphabetical_front_bias() -> None:
    grouped = [f"A{i:03d}" for i in range(120)] + [f"B{i:03d}" for i in range(120)]
    mixed = _stable_symbol_mix(grouped)
    first_window = mixed[:80]

    assert len(first_window) == 80
    assert any(symbol.startswith("A") for symbol in first_window)
    assert any(symbol.startswith("B") for symbol in first_window)

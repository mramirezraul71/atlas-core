from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from atlas_code_quant.iron_butterfly.iron_butterfly_complete import (  # noqa: E402
    DEFAULT_CONFIG,
    IronButterflyBacktester,
    StrikeMagnetDetector,
)


def _sample_intraday_df(n: int = 120, base: float = 5200.0) -> pd.DataFrame:
    rng = np.random.default_rng(21)
    idx = pd.date_range("2026-04-01 13:00:00+00:00", periods=n, freq="15min")
    close = base + np.cumsum(rng.normal(0, 1.6, n))
    open_ = np.roll(close, 1)
    open_[0] = close[0]
    high = np.maximum(open_, close) + rng.uniform(0.1, 2.0, n)
    low = np.minimum(open_, close) - rng.uniform(0.1, 2.0, n)
    vol = rng.integers(5000, 20000, n)
    return pd.DataFrame({"open": open_, "high": high, "low": low, "close": close, "volume": vol}, index=idx)


def _sample_hourly_days() -> pd.DataFrame:
    rng = np.random.default_rng(9)
    idx = pd.date_range("2026-04-01 13:00:00+00:00", periods=24 * 10, freq="1h")
    close = 5200 + np.cumsum(rng.normal(0, 2.2, len(idx)))
    open_ = np.roll(close, 1)
    open_[0] = close[0]
    high = np.maximum(open_, close) + rng.uniform(0.2, 2.2, len(idx))
    low = np.minimum(open_, close) - rng.uniform(0.2, 2.2, len(idx))
    vol = rng.integers(50_000, 250_000, len(idx))
    return pd.DataFrame({"open": open_, "high": high, "low": low, "close": close, "volume": vol}, index=idx)


def test_strike_magnet_detector() -> None:
    det = StrikeMagnetDetector()
    df = _sample_intraday_df()
    ts = df.index[-1]
    m = det.identify_magnet_strike("2026-04-01", ts, df)
    assert m.strike > 0
    assert 0 <= m.hybrid_score <= 1


def test_gex_analysis() -> None:
    det = StrikeMagnetDetector()
    df = _sample_intraday_df()
    g = det.analyze_gex_profiles(df, df.index[-1])
    assert g.profile in {"walls", "pillars", "slides", "pins"}
    assert 0 <= g.gex_score <= 1


def test_order_flow_concentration() -> None:
    det = StrikeMagnetDetector()
    df = _sample_intraday_df()
    of = det.detect_order_flow_concentration(df, df.index[-1])
    assert of.best_strike > 0
    assert 0 <= of.concentration_pct <= 1


def test_afternoon_pin_detection() -> None:
    det = StrikeMagnetDetector()
    ts_afternoon = pd.Timestamp("2026-04-01 18:30:00+00:00")  # 14:30 ET
    ts_morning = pd.Timestamp("2026-04-01 14:30:00+00:00")  # 10:30 ET
    assert det.is_afternoon_pin_window(ts_afternoon, mode="live") is True
    assert det.is_afternoon_pin_window(ts_morning, mode="live") is False
    assert det.is_afternoon_pin_window(ts_morning, mode="backtest") is True


def test_dealer_hedging_simulation() -> None:
    det = StrikeMagnetDetector()
    sim = det.simulate_dealer_hedging(spot=5200, strike=5200)
    assert isinstance(sim.pullback_force, float)
    assert sim.rebalance_size != 0 or sim.delta_after_move != sim.initial_delta


def test_iron_butterfly_construction() -> None:
    bt = IronButterflyBacktester("2026-04-01", "2026-04-15", capital=10_000)
    df = _sample_intraday_df()
    mag = bt.detector.identify_magnet_strike("2026-04-01", df.index[-1], df)
    t = bt._construct_fly_on_magnet(mag, spot=float(df["close"].iloc[-1]), ts=df.index[-1])
    assert t.short_strike > 0
    assert t.max_risk > 0
    assert t.contracts >= 1


def test_trade_management_tp_sl() -> None:
    bt = IronButterflyBacktester("2026-04-01", "2026-04-15", capital=10_000)
    df = _sample_intraday_df(30)
    mag = bt.detector.identify_magnet_strike("2026-04-01", df.index[0], df)
    t = bt._construct_fly_on_magnet(mag, spot=float(df["close"].iloc[0]), ts=df.index[0])
    out = bt._manage_trade(t, df)
    assert out.exit_reason in {"TP", "SL", "EOD", "PIN"}
    assert out.exit_time is not None


def test_backtester_complete_cycle(monkeypatch, tmp_path) -> None:
    bt = IronButterflyBacktester("2026-04-01", "2026-04-10", output_dir=tmp_path)
    sample = _sample_hourly_days()

    def _fake_load(interval="1h"):
        bt.spx_data = sample
        return sample

    monkeypatch.setattr(bt, "load_spx_data", _fake_load)
    out = bt.run_complete_backtest()
    assert "metrics" in out
    assert out["capital_final"] >= 0


def test_metrics_calculation(monkeypatch, tmp_path) -> None:
    bt = IronButterflyBacktester("2026-04-01", "2026-04-10", output_dir=tmp_path)
    bt.config["magnet_threshold"] = 0.2
    bt.config["orderflow_threshold"] = 0.0
    sample = _sample_hourly_days()
    monkeypatch.setattr(bt, "load_spx_data", lambda interval="1h": sample)
    bt.spx_data = sample
    bt.run_complete_backtest()
    m = bt.calculate_advanced_metrics()
    keys = {
        "total_trades",
        "win_rate_pct",
        "profit_factor",
        "sharpe_ratio",
        "max_drawdown_pct",
        "magnet_accuracy_pct",
    }
    assert keys.issubset(set(m.keys()))


def test_cli_script(monkeypatch, tmp_path) -> None:
    import importlib.util

    cli_path = ROOT / "atlas_code_quant" / "scripts" / "run_iron_butterfly.py"
    spec = importlib.util.spec_from_file_location("run_iron_butterfly", cli_path)
    assert spec is not None and spec.loader is not None
    cli = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cli)

    sample = _sample_hourly_days()

    def _fake_load(self, interval="1h"):
        self.spx_data = sample
        return sample

    monkeypatch.setattr(IronButterflyBacktester, "load_spx_data", _fake_load)
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "run_iron_butterfly.py",
            "--start-date",
            "2026-04-01",
            "--end-date",
            "2026-04-10",
            "--output-dir",
            str(tmp_path),
            "--dry-run",
        ],
    )
    assert cli.main() == 0


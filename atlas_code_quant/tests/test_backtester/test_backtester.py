from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
QUANT_ROOT = ROOT / "atlas_code_quant"
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from atlas_code_quant.backtester import AtlasBacktester, TradeRecord  # noqa: E402


def _sample_df(n: int = 300) -> pd.DataFrame:
    rng = np.random.default_rng(7)
    close = np.linspace(100, 120, n) + rng.normal(0, 0.2, n)
    df = pd.DataFrame(
        {
            "open": close * 0.999,
            "high": close * 1.004,
            "low": close * 0.996,
            "close": close,
            "volume": rng.uniform(1000, 5000, n),
        },
        index=pd.date_range("2026-04-01", periods=n, freq="1h", tz="UTC"),
    )
    return df


def test_load_historical_data_uses_cache(tmp_path) -> None:
    bt = AtlasBacktester("2026-04-01", "2026-04-15", search_mode=False)
    bt._data_dir = tmp_path
    df = _sample_df(100).reset_index().rename(columns={"index": "timestamp"})
    local = tmp_path / "AAPL_1h_2026-04-01_to_2026-04-15.csv"
    df.to_csv(local, index=False)
    out = bt.load_historical_data(["AAPL"], "1h")
    assert "AAPL" in out
    assert len(out["AAPL"]) == 100


def test_trade_execution_and_exit_logic() -> None:
    bt = AtlasBacktester("2026-04-01", "2026-04-15", search_mode=False)
    ts = pd.Timestamp("2026-04-01T10:00:00Z")
    cand = {"symbol": "AAPL", "direction": 1, "entry_price": 100.0}
    trade = bt._execute_entry(cand, ts)
    assert trade.entry_price > 0
    bt.open_trades["AAPL"] = trade
    cycle = {
        "AAPL": pd.DataFrame(
            [{"open": 100, "high": trade.take_profit + 0.1, "low": 99, "close": 101, "volume": 1000}],
            index=[ts + pd.Timedelta(hours=1)],
        )
    }
    closed = bt._evaluate_open_trades(ts + pd.Timedelta(hours=1), cycle)
    assert len(closed) == 1
    assert closed[0].status in {"closed_tp", "closed_sl", "closed_eod"}


def test_metrics_calculation_basic() -> None:
    bt = AtlasBacktester("2026-04-01", "2026-04-15", search_mode=False)
    t1 = TradeRecord("AAPL", 1, pd.Timestamp("2026-04-01T10:00:00Z"), 100, 10, 98, 104)
    t1.exit_time = pd.Timestamp("2026-04-01T12:00:00Z")
    t1.exit_price = 104
    t1.pnl = 38
    t1.pnl_pct = 3.8
    t1.status = "closed_tp"
    t2 = TradeRecord("MSFT", 1, pd.Timestamp("2026-04-01T13:00:00Z"), 100, 10, 98, 104)
    t2.exit_time = pd.Timestamp("2026-04-01T15:00:00Z")
    t2.exit_price = 98
    t2.pnl = -21
    t2.pnl_pct = -2.1
    t2.status = "closed_sl"
    bt.trades = [t1, t2]
    bt.equity_curve = [
        {"timestamp": pd.Timestamp("2026-04-01T00:00:00Z"), "equity": 10_000},
        {"timestamp": pd.Timestamp("2026-04-02T00:00:00Z"), "equity": 10_020},
        {"timestamp": pd.Timestamp("2026-04-03T00:00:00Z"), "equity": 10_017},
    ]
    m = bt.calculate_metrics()
    assert m["total_trades"] == 2
    assert 0 <= m["win_rate_pct"] <= 100
    assert "sharpe_ratio" in m
    assert "max_drawdown_pct" in m


def test_parameter_search_finds_best(monkeypatch) -> None:
    bt = AtlasBacktester("2026-04-01", "2026-04-15", search_mode=True)
    bt.data = {"AAPL": _sample_df(120)}

    def fake_sim(_data):
        return None

    def fake_calc():
        base = bt.signal_strength_threshold
        wr = 58.0 + (base - 0.55) * 100
        return {
            "win_rate_pct": wr,
            "sharpe_ratio": 1.1,
            "profit_factor": 1.3,
            "max_drawdown_pct": -10.0,
            "consecutive_losses": 2,
            "total_trades": 40,
        }

    monkeypatch.setattr(bt, "simulate_trading_cycles", fake_sim)
    monkeypatch.setattr(bt, "calculate_metrics", fake_calc)
    best = bt.search_optimal_parameters(target_win_rate=0.58, max_iterations=1)
    assert best is not None
    assert "combo" in best
    assert len(bt.parameter_search_history) > 0


def test_go_live_decision() -> None:
    bt = AtlasBacktester("2026-04-01", "2026-04-15", search_mode=False)
    bt.metrics = {
        "win_rate_pct": 60.0,
        "sharpe_ratio": 1.1,
        "max_drawdown_pct": -12.0,
        "profit_factor": 1.3,
        "consecutive_losses": 2,
        "total_trades": 35,
    }
    dec = bt.evaluate_go_live()
    assert dec.go_live_approved is True

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
QUANT_ROOT = ROOT / "atlas_code_quant"
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from atlas_code_quant.config.settings import settings
from atlas_code_quant.execution.signal_executor import SignalExecutor


def _configure_equity_kelly(monkeypatch, *, max_position_pct: float = 0.20, max_risk_pct: float = 0.01, min_qty: int = 1) -> None:
    monkeypatch.setenv("ATLAS_MODE", "paper")
    monkeypatch.setattr(settings, "equity_kelly_paper_enabled", True, raising=False)
    monkeypatch.setattr(settings, "equity_kelly_live_enabled", False, raising=False)
    monkeypatch.setattr(settings, "kelly_fraction", 0.25, raising=False)
    monkeypatch.setattr(settings, "kelly_max_position_pct", max_position_pct, raising=False)
    monkeypatch.setattr(settings, "kelly_min_samples", 6, raising=False)
    monkeypatch.setattr(settings, "equity_kelly_max_risk_per_trade_pct", max_risk_pct, raising=False)
    monkeypatch.setattr(settings, "equity_kelly_min_qty", min_qty, raising=False)
    monkeypatch.setattr(settings, "atr_sl_multiplier", 1.5, raising=False)
    monkeypatch.setattr(settings, "atr_tp_multiplier", 3.0, raising=False)


def _capture_send(monkeypatch):
    sent: list[dict[str, object]] = []

    def _fake_send(self, result, order_req, sig):
        sent.append({"qty": result.quantity, "order_req": order_req, "signal": dict(sig)})
        result.status = "simulated"
        result.order_id = "TEST-ORDER"
        result.fill_price = float(sig["entry_price"])
        return result

    monkeypatch.setattr(SignalExecutor, "_send_with_fallback", _fake_send)
    return sent


def _make_signal(**overrides):
    signal = {
        "symbol": "AAPL",
        "signal_type": "BUY",
        "entry_price": 100.0,
        "atr": 2.0,
        "strategy": "momentum",
        "strategy_type": "equity_long",
        "asset_class": "equity",
        "position_size": 50,
        "pnl_history": [100.0, 80.0, -40.0, 120.0, 90.0, -30.0, 110.0],
    }
    signal.update(overrides)
    return signal


def test_equity_kelly_sizing_applies_fraction(monkeypatch) -> None:
    _configure_equity_kelly(monkeypatch, max_position_pct=0.20, max_risk_pct=0.05)
    sent = _capture_send(monkeypatch)
    executor = SignalExecutor(mode="paper")

    result = executor.execute(_make_signal(), capital=10_000.0)

    assert result.status == "simulated"
    assert result.quantity == 15
    assert sent[0]["qty"] == 15


def test_equity_kelly_respects_max_position_pct(monkeypatch) -> None:
    _configure_equity_kelly(monkeypatch, max_position_pct=0.05, max_risk_pct=1.0)
    sent = _capture_send(monkeypatch)
    executor = SignalExecutor(mode="paper")

    result = executor.execute(_make_signal(position_size=100, atr=0.5), capital=10_000.0)

    assert result.status == "simulated"
    assert result.quantity == 5
    assert sent[0]["qty"] == 5


def test_equity_kelly_fallback_when_samples_insufficient(monkeypatch) -> None:
    _configure_equity_kelly(monkeypatch, max_position_pct=0.05, max_risk_pct=0.01)
    sent = _capture_send(monkeypatch)
    executor = SignalExecutor(mode="paper")

    result = executor.execute(
        _make_signal(
            position_size=10,
            atr=50.0,
            pnl_history=[100.0, -50.0, 80.0],
        ),
        capital=10_000.0,
    )

    assert result.status == "simulated"
    assert result.quantity == 1
    assert sent[0]["qty"] == 1


def test_equity_kelly_zero_qty_blocks_submit(monkeypatch) -> None:
    _configure_equity_kelly(monkeypatch, max_position_pct=0.05, max_risk_pct=0.01, min_qty=1)
    sent = _capture_send(monkeypatch)
    executor = SignalExecutor(mode="paper")

    result = executor.execute(
        _make_signal(
            entry_price=1_000.0,
            atr=10.0,
            position_size=1,
            pnl_history=[50.0, -25.0],
        ),
        capital=100.0,
    )

    assert result.status == "blocked"
    assert result.error == "quantity_zero_kelly_or_risk"
    assert sent == []


def test_equity_kelly_live_disabled_keeps_previous_behavior(monkeypatch) -> None:
    _configure_equity_kelly(monkeypatch, max_position_pct=0.20, max_risk_pct=0.05)
    monkeypatch.setenv("ATLAS_MODE", "live")
    monkeypatch.setattr(settings, "equity_kelly_live_enabled", False, raising=False)
    sent = _capture_send(monkeypatch)
    executor = SignalExecutor(mode="live")

    def _unexpected_call(*args, **kwargs):
        raise AssertionError("equity Kelly path should remain disabled in live")

    monkeypatch.setattr(executor, "_resolve_equity_kelly_quantity", _unexpected_call)

    result = executor.execute(_make_signal(position_size=7), capital=10_000.0)

    assert result.status == "simulated"
    assert result.quantity == 7
    assert sent[0]["qty"] == 7


def test_equity_kelly_fallback_without_seed_blocks_submit(monkeypatch) -> None:
    _configure_equity_kelly(monkeypatch, max_position_pct=0.05, max_risk_pct=0.01, min_qty=1)
    sent = _capture_send(monkeypatch)
    executor = SignalExecutor(mode="paper")

    result = executor.execute(
        _make_signal(
            position_size=0,
            pnl_history=[100.0, -50.0, 80.0],
            atr=5.0,
        ),
        capital=10_000.0,
    )

    assert result.status == "blocked"
    assert result.error == "quantity_zero_kelly_or_risk"
    assert sent == []

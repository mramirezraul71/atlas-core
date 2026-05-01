from __future__ import annotations

import json
from pathlib import Path

import pytest

from atlas_code_quant.data.signal_validator import SignalValidator
from atlas_code_quant.selector.strategy_selector import StrategySelectorService


class _TrackerStub:
    def build_summary(self, account_scope: str = "paper", account_id: str | None = None) -> dict:
        return {
            "balances": {
                "cash": 100000.0,
                "option_buying_power": 100000.0,
            },
            "account_session": {
                "scope": account_scope,
                "account_id": account_id,
            },
        }


class _LearningStub:
    def context(self, **_: object) -> dict:
        return {
            "symbol_bias": 0.0,
            "directional_symbol_bias": 0.0,
        }

    def strategy_bias(self, strategy_type: str, account_scope: str = "paper") -> float:
        return 0.0

    def risk_multiplier(self, account_scope: str = "paper") -> float:
        return 1.0


def _state_file(tmp_path: Path, *, enabled: bool = True) -> Path:
    path = tmp_path / "operation_center_state.json"
    path.write_text(json.dumps({"signal_validator_enabled": enabled}), encoding="utf-8")
    return path


def test_validate_price_data_rejects_negative_price(tmp_path: Path) -> None:
    validator = SignalValidator(state_path=_state_file(tmp_path))

    result = validator.validate_price_data("AAPL", -1.0, 1000, 99.0, 100.0)

    assert result.is_valid is False
    assert result.severity == "critical"
    assert "price <= 0" in result.reason


def test_validate_price_data_rejects_zero_volume_during_market_hours(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    validator = SignalValidator(state_path=_state_file(tmp_path))
    monkeypatch.setattr(validator, "_market_hours_active", lambda now=None: True)

    result = validator.validate_price_data("AAPL", 100.0, 0, 99.8, 100.2)

    assert result.is_valid is False
    assert result.severity == "warn"
    assert "volume = 0" in result.reason


def test_validate_price_data_rejects_crossed_market(tmp_path: Path) -> None:
    validator = SignalValidator(state_path=_state_file(tmp_path))

    result = validator.validate_price_data("AAPL", 100.0, 1200, 101.0, 100.0)

    assert result.is_valid is False
    assert result.severity == "critical"
    assert "crossed market" in result.reason


def test_validate_price_data_detects_large_gap_vs_previous_bar(tmp_path: Path) -> None:
    validator = SignalValidator(state_path=_state_file(tmp_path))

    first = validator.validate_price_data("AAPL", 100.0, 1000, 99.8, 100.2)
    second = validator.validate_price_data("AAPL", 121.0, 1000, 120.8, 121.2)

    assert first.is_valid is True
    assert second.is_valid is False
    assert second.severity == "warn"
    assert "gap" in second.reason


def test_validate_trade_entry_rejects_empty_symbol(tmp_path: Path) -> None:
    validator = SignalValidator(state_path=_state_file(tmp_path))

    result = validator.validate_trade_entry(
        {
            "symbol": "",
            "strategy_type": "equity_long",
            "signed_qty": 1,
            "entry_price": 100.0,
        }
    )

    assert result.is_valid is False
    assert result.severity == "error"
    assert "symbol vacío" in result.reason


def test_validate_trade_entry_rejects_unknown_strategy_type(tmp_path: Path) -> None:
    validator = SignalValidator(state_path=_state_file(tmp_path))

    result = validator.validate_trade_entry(
        {
            "symbol": "SPY",
            "strategy_type": "mystery_strategy",
            "signed_qty": 1,
            "entry_price": 100.0,
        }
    )

    assert result.is_valid is False
    assert result.severity == "error"
    assert "no reconocido" in result.reason


def test_validate_trade_entry_accepts_defined_risk_multileg_payload(tmp_path: Path) -> None:
    validator = SignalValidator(state_path=_state_file(tmp_path))

    result = validator.validate_trade_entry(
        {
            "symbol": "SPY",
            "strategy_type": "bull_call_debit_spread",
            "positions": [
                {"symbol": "SPY250620C00520000", "signed_qty": 1},
                {"symbol": "SPY250620C00530000", "signed_qty": -1},
            ],
            "entry_price": 2.35,
        }
    )

    assert result.is_valid is True


def test_validate_journal_write_rejects_negative_entry_price(tmp_path: Path) -> None:
    validator = SignalValidator(state_path=_state_file(tmp_path))

    result = validator.validate_journal_write({"symbol": "AAOI", "entry_price": -0.01})

    assert result.is_valid is False
    assert result.severity == "critical"
    assert "entry_price < 0" in result.reason


def test_validate_journal_write_rejects_missing_underlying(tmp_path: Path) -> None:
    validator = SignalValidator(state_path=_state_file(tmp_path))

    result = validator.validate_journal_write({"entry_price": 10.0})

    assert result.is_valid is False
    assert result.severity == "error"
    assert "underlying válido" in result.reason


def test_signal_validator_can_be_disabled_from_state(tmp_path: Path) -> None:
    validator = SignalValidator(state_path=_state_file(tmp_path, enabled=False))

    result = validator.validate_trade_entry(
        {
            "symbol": "",
            "strategy_type": "",
            "signed_qty": 0,
            "entry_price": -10.0,
        }
    )

    assert result.is_valid is True


def test_selector_proposal_rejects_invalid_candidate_price(tmp_path: Path) -> None:
    validator = SignalValidator(state_path=_state_file(tmp_path))
    service = StrategySelectorService(  # type: ignore[arg-type]
        _TrackerStub(),
        _LearningStub(),
        signal_validator=validator,
    )

    with pytest.raises(ValueError, match="price <= 0"):
        service.proposal(
            candidate={
                "symbol": "AAOI",
                "direction": "alcista",
                "timeframe": "15m",
                "price": -5.0,
                "selection_score": 83.17,
                "local_win_rate_pct": 56.41,
                "predicted_move_pct": 2.31,
                "confirmation": {"direction": "alcista", "higher_timeframe": "1h"},
            },
            account_scope="paper",
        )

from __future__ import annotations

import json
import sys
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

import pytest

ROOT = Path(__file__).resolve().parents[2]
QUANT_ROOT = ROOT / "atlas_code_quant"
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from atlas_code_quant.api import main
from atlas_code_quant.config.settings import TradingConfig
from atlas_code_quant.execution.kelly_sizer import KellySizer


def _write_config(tmp_path: Path, payload: dict) -> Path:
    p = tmp_path / "market_open_config.json"
    p.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return p


def test_market_open_config_loads_risk_fields(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("QUANT_MARKET_OPEN_MAX_POSITIONS", raising=False)
    monkeypatch.delenv("QUANT_KELLY_FRACTION", raising=False)
    monkeypatch.delenv("QUANT_EQUITY_KELLY_MAX_RISK_PCT", raising=False)
    monkeypatch.delenv("QUANT_KELLY_MAX_PCT", raising=False)
    monkeypatch.delenv("QUANT_MARKET_OPEN_DAILY_LOSS_LIMIT", raising=False)
    monkeypatch.delenv("QUANT_MARKET_OPEN_SCAN_INTERVAL_MIN", raising=False)
    monkeypatch.delenv("QUANT_MARKET_OPEN_SCHEDULE_OPEN_ET", raising=False)
    monkeypatch.delenv("QUANT_MARKET_OPEN_SCHEDULE_CLOSE_ET", raising=False)

    cfg_path = _write_config(
        tmp_path,
        {
            "schedule_et": {
                "market_open": "09:45",
                "market_close": "15:45",
                "scan_interval_min": 10,
            },
            "risk": {
                "max_positions": 7,
                "kelly_fraction": 0.18,
                "max_risk_per_trade": 0.015,
                "max_position_pct": 0.12,
                "daily_loss_limit": 0.03,
            },
        },
    )
    cfg = TradingConfig(market_open_config_path=cfg_path)

    assert cfg.market_open_config_loaded is True
    assert cfg.market_open_schedule_open_et == "09:45"
    assert cfg.market_open_schedule_close_et == "15:45"
    assert cfg.market_open_scan_interval_min == 10
    assert cfg.market_open_max_positions == 7
    assert cfg.kelly_fraction == pytest.approx(0.18)
    assert cfg.equity_kelly_max_risk_per_trade_pct == pytest.approx(0.015)
    assert cfg.kelly_max_position_pct == pytest.approx(0.12)
    assert cfg.market_open_daily_loss_limit_pct == pytest.approx(0.03)


def test_market_open_config_invalid_values_fall_back_safely(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("QUANT_MARKET_OPEN_MAX_POSITIONS", raising=False)
    monkeypatch.delenv("QUANT_KELLY_FRACTION", raising=False)
    monkeypatch.delenv("QUANT_EQUITY_KELLY_MAX_RISK_PCT", raising=False)
    monkeypatch.delenv("QUANT_KELLY_MAX_PCT", raising=False)
    monkeypatch.delenv("QUANT_MARKET_OPEN_DAILY_LOSS_LIMIT", raising=False)
    monkeypatch.delenv("QUANT_MARKET_OPEN_SCAN_INTERVAL_MIN", raising=False)

    cfg_path = _write_config(
        tmp_path,
        {
            "schedule_et": {"scan_interval_min": "not-an-int"},
            "risk": {
                "max_positions": -9,
                "kelly_fraction": "bad",
                "max_risk_per_trade": 9.0,
                "max_position_pct": -1,
                "daily_loss_limit": 99,
            },
        },
    )
    cfg = TradingConfig(market_open_config_path=cfg_path)

    assert cfg.market_open_config_loaded is True
    assert cfg.market_open_max_positions == 3
    assert cfg.kelly_fraction == pytest.approx(0.25)
    assert cfg.equity_kelly_max_risk_per_trade_pct == pytest.approx(0.01)
    assert cfg.kelly_max_position_pct == pytest.approx(0.20)
    assert cfg.market_open_daily_loss_limit_pct == pytest.approx(0.02)
    assert cfg.market_open_scan_interval_min == 5


def test_market_open_config_applies_max_positions_guard(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("QUANT_MARKET_OPEN_MAX_POSITIONS", raising=False)
    monkeypatch.delenv("QUANT_MARKET_OPEN_SCHEDULE_OPEN_ET", raising=False)
    monkeypatch.delenv("QUANT_MARKET_OPEN_SCHEDULE_CLOSE_ET", raising=False)

    cfg_path = _write_config(
        tmp_path,
        {"schedule_et": {"market_open": "09:30", "market_close": "16:00"}, "risk": {"max_positions": 2}},
    )
    cfg = TradingConfig(market_open_config_path=cfg_path)
    monkeypatch.setattr(main, "settings", cfg, raising=False)
    monkeypatch.setattr(main, "_broker_open_positions_count", lambda **kwargs: 2)

    payload = main._entry_pass_runtime_gate(
        action="submit",
        account_scope="paper",
        account_id=None,
        now_et=datetime(2026, 4, 14, 10, 0, tzinfo=ZoneInfo("America/New_York")),
    )

    assert payload["skip_entry_pass"] is True
    assert payload["reason"] == "max_open_positions"
    assert payload["max_open_positions"] == 2
    assert payload["open_positions"] == 2


def test_market_open_config_applies_kelly_fraction(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("QUANT_KELLY_FRACTION", raising=False)
    cfg_path = _write_config(
        tmp_path,
        {"schedule_et": {}, "risk": {"kelly_fraction": 0.33}},
    )
    cfg = TradingConfig(market_open_config_path=cfg_path)
    sizer = KellySizer(fraction=cfg.kelly_fraction, max_position_pct=cfg.kelly_max_position_pct)
    assert sizer.fraction == pytest.approx(0.33)


def test_market_open_config_missing_file_no_crash(tmp_path: Path) -> None:
    missing = tmp_path / "does_not_exist.json"
    cfg = TradingConfig(market_open_config_path=missing)
    assert cfg.market_open_config_loaded is False
    assert cfg.market_open_max_positions >= 1


def test_env_overrides_json_for_kelly(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.setenv("QUANT_KELLY_FRACTION", "0.4")
    cfg_path = _write_config(tmp_path, {"schedule_et": {}, "risk": {"kelly_fraction": 0.11}})
    cfg = TradingConfig(market_open_config_path=cfg_path)
    assert cfg.kelly_fraction == pytest.approx(0.4)

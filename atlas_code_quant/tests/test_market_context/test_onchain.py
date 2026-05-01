from __future__ import annotations

import sys
from pathlib import Path

QUANT_ROOT = Path(__file__).resolve().parents[2]
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from market_context.onchain_metrics import apply_test_fixture, onchain_gate  # noqa: E402


def test_sopr_bullish_under_one() -> None:
    fx = apply_test_fixture(sopr_current=0.95, mvrv_signal="neutral", age_spike=False)
    assert fx["sopr"]["signal"] == "bullish"


def test_mvrv_extreme_in_gate() -> None:
    sopr = {"sopr_current": 1.0, "sopr_ma7": 1.0, "signal": "neutral"}
    mvrv = {"mvrv_zscore": 8.0, "signal": "bearish_extreme"}
    age = {"age_consumed": 1.0, "spike": False, "signal": "normal"}
    g = onchain_gate("BTC", sopr, mvrv, age)
    assert g["gate_pass"] is False
    assert "MVRV extreme" in g["reasons"]


def test_age_spike_with_mvrv_bearish_rejects() -> None:
    sopr = {"sopr_current": 1.0, "sopr_ma7": 1.0, "signal": "neutral"}
    mvrv = {"mvrv_zscore": 1.0, "signal": "bearish"}
    age = {"age_consumed": 10.0, "spike": True, "signal": "cycle_change_potential"}
    g = onchain_gate("BTC", sopr, mvrv, age)
    assert g["gate_pass"] is False


def test_onchain_gate_passes_clean_fixture() -> None:
    fx = apply_test_fixture(sopr_current=1.0, mvrv_signal="neutral", age_spike=False)
    assert fx["gate"]["gate_pass"] is True

from __future__ import annotations

import sys
from pathlib import Path

QUANT_ROOT = Path(__file__).resolve().parents[2]
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from market_context.vix_handler import (  # noqa: E402
    calculate_vix_zscore,
    get_vix_gate,
    get_vix_multiplier_from_zscore,
    get_vix_zscore_multiplier,
)


def test_zscore_and_multipliers() -> None:
    assert calculate_vix_zscore(22.0, 20.0, 2.0) == 1.0
    assert get_vix_multiplier_from_zscore(2.0) == 0.5
    assert get_vix_multiplier_from_zscore(1.0) == 0.7
    assert get_vix_multiplier_from_zscore(0.0) == 1.0
    assert get_vix_zscore_multiplier(22.0, 20.0, 2.0) == 0.7
    assert get_vix_zscore_multiplier(24.0, 20.0, 2.0) == 0.5


def test_vix_gates() -> None:
    assert get_vix_gate(36.0) == "panic"
    assert get_vix_gate(32.0) == "caution"
    assert get_vix_gate(18.0) == "normal"

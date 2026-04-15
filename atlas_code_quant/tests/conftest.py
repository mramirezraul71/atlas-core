from __future__ import annotations

import importlib
import sys
from typing import Any
from unittest.mock import patch

import pytest

_API_MAIN_MODULE = "atlas_code_quant.api.main"
_LEARNING_STATUS_STUB: dict[str, Any] = {
    "enabled": True,
    "generated_at": "1970-01-01T00:00:00Z",
    "window_days": 0,
    "scope": "paper",
    "sample_count": 0,
    "win_rate_pct": 0.0,
    "profit_factor": 0.0,
    "expectancy_pct": 0.0,
    "risk_multiplier": 1.0,
    "top_positive": [],
    "top_negative": [],
    "status_mode": "mocked",
}


@pytest.fixture
def api_main_module():
    """
    Importa api.main con AdaptiveLearningService.status mockeado.

    Evita queries reales a trading_journal durante startup en collection/tests API.
    """
    with patch(
        "learning.adaptive_policy.AdaptiveLearningService.status",
        autospec=True,
        return_value=dict(_LEARNING_STATUS_STUB),
    ):
        if _API_MAIN_MODULE in sys.modules:
            return importlib.reload(sys.modules[_API_MAIN_MODULE])
        return importlib.import_module(_API_MAIN_MODULE)

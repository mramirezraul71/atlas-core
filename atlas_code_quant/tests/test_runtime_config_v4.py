from __future__ import annotations

import pytest

from atlas_code_quant.operations.runtime_config_v4 import validate_runtime_config_v4


def test_runtime_config_v4_blocks_full_live_in_dev() -> None:
    with pytest.raises(ValueError):
        validate_runtime_config_v4(
            {
                "deployment_mode": "dev",
                "allowed_runtime_modes": ["paper_baseline", "full_live"],
            }
        )

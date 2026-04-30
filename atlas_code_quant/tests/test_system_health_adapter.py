from __future__ import annotations

import sys
from pathlib import Path

QUANT_ROOT = Path(__file__).resolve().parents[1]
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from atlas_core.adapters.system_health_adapter import SystemHealthAdapter


def test_system_health_fallback_without_psutil(tmp_path: Path) -> None:
    ad = SystemHealthAdapter(psutil_module=False, root_path=tmp_path)
    st = ad.get_state()
    rk = ad.get_risks()
    assert st.name == "system_health"
    assert "fallback_mode" in st.details
    assert rk.level in {"low", "medium", "high", "critical"}


def test_system_health_derives_degraded_from_mock_metrics(tmp_path: Path) -> None:
    class _FakePsutil:
        @staticmethod
        def cpu_percent(interval=None):  # noqa: ANN001
            return 95.0

        class _VM:
            percent = 50.0

        @staticmethod
        def virtual_memory():
            return _FakePsutil._VM()

    ad = SystemHealthAdapter(psutil_module=_FakePsutil(), root_path=tmp_path)
    st = ad.get_state()
    rk = ad.get_risks()
    assert st.health in {"degraded", "critical"}
    assert rk.level in {"high", "critical", "medium"}


from __future__ import annotations

from pathlib import Path

from modules.atlas_radar_kalshi.control.calibration_kpis import compute_calibration_kpis
from modules.atlas_radar_kalshi.state.journal import Journal


def test_calibration_kpis_empty_journal(tmp_path: Path) -> None:
    j = Journal(tmp_path)
    k = compute_calibration_kpis(j)
    assert k.n_pairs == 0
    assert k.brier_score == 0.0


def test_calibration_kpis_brier_pair(tmp_path: Path) -> None:
    j = Journal(tmp_path)
    j.write(
        "decisions",
        {
            "ticker": "KX-T",
            "readout": {"p_ensemble": 0.7},
            "gate": {"side": "YES"},
        },
    )
    j.write("exits", {"ticker": "KX-T", "pnl_cents": 100, "entry": 50})
    k = compute_calibration_kpis(j)
    assert k.n_pairs == 1
    assert 0.0 <= k.brier_score <= 1.0

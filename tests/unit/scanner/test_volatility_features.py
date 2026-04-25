from __future__ import annotations

import pytest

from atlas_scanner.features.volatility import (
    compute_iv_percentile,
    compute_iv_rank,
    compute_vrp,
    compute_vrp_series,
)


def test_compute_iv_rank_simple_case() -> None:
    iv_history = [10.0, 20.0, 30.0, 40.0, 50.0]
    assert compute_iv_rank(iv_history, current_iv=40.0) == pytest.approx(75.0)


def test_compute_iv_rank_boundaries_and_empty_inputs() -> None:
    iv_history = [10.0, 20.0, 30.0, 40.0, 50.0]
    assert compute_iv_rank(iv_history, current_iv=10.0) == 0.0
    assert compute_iv_rank(iv_history, current_iv=50.0) == 100.0
    assert compute_iv_rank([12.0, 12.0, 12.0], current_iv=12.0) == 0.0
    assert compute_iv_rank([], current_iv=12.0) == 0.0


def test_compute_iv_percentile_cases() -> None:
    iv_history = [10.0, 20.0, 30.0, 40.0, 50.0]
    assert compute_iv_percentile(iv_history, current_iv=30.0) == pytest.approx(60.0)
    assert compute_iv_percentile(iv_history, current_iv=10.0) == pytest.approx(20.0)
    assert compute_iv_percentile(iv_history, current_iv=55.0) == pytest.approx(100.0)
    assert compute_iv_percentile([], current_iv=10.0) == 0.0


def test_compute_vrp_basic_and_negative_case() -> None:
    assert compute_vrp(iv_annualized=0.25, rv_annualized=0.20) == pytest.approx(0.05)
    assert compute_vrp(iv_annualized=0.15, rv_annualized=0.20) == pytest.approx(-0.05)


def test_compute_vrp_series_with_overlapping_keys() -> None:
    iv_series = {"5d": 0.25, "10d": 0.30, "20d": 0.35}
    rv_series = {"5d": 0.20, "10d": 0.28, "60d": 0.22}
    vrp_series = compute_vrp_series(iv_series=iv_series, rv_series=rv_series)

    assert vrp_series == {
        "5d": pytest.approx(0.05),
        "10d": pytest.approx(0.02),
    }


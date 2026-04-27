"""Tests F6 — reconcile()."""
from __future__ import annotations

from atlas_code_quant.execution.reconciler import reconcile


def test_no_drift_when_equal() -> None:
    rep = reconcile({"AAPL": 1, "MSFT": 2}, {"AAPL": 1, "MSFT": 2})
    assert rep.has_drift is False
    assert len(rep.items) == 2
    for item in rep.items:
        assert item.delta == 0


def test_drift_detected() -> None:
    rep = reconcile({"AAPL": 1}, {"AAPL": 2, "MSFT": 5})
    assert rep.has_drift is True
    by_sym = {i.symbol: i for i in rep.items}
    assert by_sym["AAPL"].delta == 1
    assert by_sym["MSFT"].delta == 5
    assert by_sym["MSFT"].internal_qty == 0


def test_internal_only_creates_negative_delta() -> None:
    rep = reconcile({"AAPL": 3}, {})
    assert rep.has_drift is True
    assert rep.items[0].delta == -3

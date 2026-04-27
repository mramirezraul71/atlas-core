"""Tests del :mod:`modules.atlas_radar_kalshi.exit_manager`.

Cubre TP / SL / time-stop / forced / data-degraded y ``build_targets``.
"""
from __future__ import annotations

import time

import pytest

from modules.atlas_radar_kalshi.exit_manager import (
    ExitConfig,
    ExitManager,
    Position,
)


def _pos(side: str = "YES", entry: int = 50, target: int = 60,
         stop: int = 46, age_s: float = 0.0) -> Position:
    return Position(
        ticker="DEMO", side=side, entry_price=entry, size=10,
        entry_ts=time.time() - age_s, edge_at_entry=0.10,
        target_price=target, stop_price=stop,
    )


# ---------------------------------------------------------------------------
class TestEvaluate:
    def test_no_exit_when_in_range(self) -> None:
        em = ExitManager(ExitConfig(sl_ticks=4, time_stop_seconds=999))
        em.open(_pos())
        sig = em.evaluate("DEMO", current_price=52, current_edge=0.05)
        assert sig.should_exit is False

    def test_take_profit_yes(self) -> None:
        em = ExitManager(ExitConfig(sl_ticks=10, time_stop_seconds=999))
        em.open(_pos(side="YES", entry=50, target=60))
        sig = em.evaluate("DEMO", current_price=60, current_edge=0.05)
        assert sig.should_exit is True
        assert sig.reason == "take_profit"
        assert sig.target_price == 60

    def test_take_profit_no(self) -> None:
        em = ExitManager(ExitConfig(sl_ticks=10, time_stop_seconds=999))
        em.open(_pos(side="NO", entry=50, target=40, stop=54))
        sig = em.evaluate("DEMO", current_price=40, current_edge=-0.05)
        assert sig.should_exit is True
        assert sig.reason == "take_profit"

    def test_stop_loss_ticks(self) -> None:
        em = ExitManager(ExitConfig(sl_ticks=4, time_stop_seconds=999))
        em.open(_pos(side="YES", entry=50, target=60, stop=46))
        # 50 - 46 = 4 ticks → stop_loss
        sig = em.evaluate("DEMO", current_price=46, current_edge=0.0)
        assert sig.should_exit is True
        assert sig.reason == "stop_loss_ticks"

    def test_edge_revert(self) -> None:
        em = ExitManager(ExitConfig(sl_ticks=99, sl_edge_revert=-0.02,
                                    time_stop_seconds=999))
        em.open(_pos(side="YES"))
        sig = em.evaluate("DEMO", current_price=49, current_edge=-0.05)
        assert sig.should_exit is True
        assert sig.reason == "edge_revert"

    def test_time_stop(self) -> None:
        em = ExitManager(ExitConfig(sl_ticks=99, time_stop_seconds=10))
        em.open(_pos(age_s=15))
        sig = em.evaluate("DEMO", current_price=50, current_edge=0.05)
        assert sig.should_exit is True
        assert sig.reason == "time_stop"

    def test_forced_overrides(self) -> None:
        em = ExitManager()
        em.open(_pos())
        sig = em.evaluate("DEMO", current_price=51, current_edge=0.05,
                          forced=True)
        assert sig.should_exit is True
        assert sig.reason == "forced"

    def test_data_degraded(self) -> None:
        em = ExitManager()
        em.open(_pos())
        sig = em.evaluate("DEMO", current_price=51, current_edge=0.05,
                          data_degraded=True)
        assert sig.should_exit is True
        assert sig.reason == "data_degraded"

    def test_no_position_returns_false(self) -> None:
        em = ExitManager()
        sig = em.evaluate("UNKNOWN", current_price=50, current_edge=0.0)
        assert sig.should_exit is False


# ---------------------------------------------------------------------------
class TestBuildTargets:
    def test_yes_targets_above_entry(self) -> None:
        tp, sl = ExitManager.build_targets(
            "YES", entry_price=50, p_fair=0.70,
            tp_capture_pct=0.5, sl_ticks=4,
        )
        assert tp > 50
        assert sl < 50
        assert tp <= 99 and sl >= 1

    def test_no_targets_below_entry(self) -> None:
        tp, sl = ExitManager.build_targets(
            "NO", entry_price=50, p_fair=0.30,
            tp_capture_pct=0.5, sl_ticks=4,
        )
        assert tp < 50
        assert sl > 50
        assert tp >= 1 and sl <= 99


# ---------------------------------------------------------------------------
class TestLifecycle:
    def test_open_close_listing(self) -> None:
        em = ExitManager()
        em.open(_pos())
        assert len(em.list_open()) == 1
        em.close("DEMO")
        assert len(em.list_open()) == 0

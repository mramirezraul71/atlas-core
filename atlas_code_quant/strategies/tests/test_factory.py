"""Tests F5 — Strategy Factory: registro, list, create_option."""
from __future__ import annotations

import pytest

from atlas_code_quant.strategies.factory import StrategyFactory


def test_list_options_includes_four() -> None:
    keys = StrategyFactory.list_options()
    assert "options.vertical_spread" in keys
    assert "options.iron_condor" in keys
    assert "options.iron_butterfly" in keys
    assert "options.straddle_strangle" in keys


def test_create_option_returns_instance() -> None:
    inst = StrategyFactory.create_option("options.vertical_spread")
    assert hasattr(inst, "build_plan")


def test_create_option_unknown_raises() -> None:
    with pytest.raises(KeyError):
        StrategyFactory.create_option("options.does_not_exist")


def test_register_option_and_create() -> None:
    class Dummy:
        name = "dummy"
        def build_plan(self, opp, cfg=None):
            return {"strategy": self.name}

    StrategyFactory.register_option("options.dummy_test", Dummy)
    inst = StrategyFactory.create_option("options.dummy_test")
    assert isinstance(inst, Dummy)


def test_list_all_has_legacy_and_options() -> None:
    all_ = StrategyFactory.list_all()
    assert "legacy" in all_
    assert "options" in all_
    assert isinstance(all_["legacy"], list)
    assert isinstance(all_["options"], list)

from atlas_code_quant.backtest.internal_gbm_simulator import LeanSimulator as NewLeanSimulator
from atlas_code_quant.backtest.lean_simulator import LeanSimulator as LegacyLeanSimulator


def test_internal_gbm_alias_keeps_lean_simulator_compat():
    assert NewLeanSimulator is LegacyLeanSimulator

from atlas_code_quant.autonomy import QuantAutonomyOrchestrator, QuantAutonomyState
from atlas_code_quant.intake import RadarOpportunityClient
from atlas_code_quant.lean import LeanConfig, LeanLauncher
from atlas_code_quant.telemetry import InMemoryCounter


def test_f1_import_scaffold_modules():
    assert RadarOpportunityClient(enabled=False).fetch_once() == []
    assert LeanConfig().enabled in {True, False}
    assert LeanLauncher().plan_backtest("AnyAlgo").command
    assert QuantAutonomyOrchestrator().state == QuantAutonomyState.BOOTING
    assert InMemoryCounter().inc("x") == 1

import pytest

from atlas_code_quant.strategies.factory import StrategyFactory
from atlas_code_quant.strategies.base import BaseStrategy, Signal, TradeSignal


class DummyStrategy(BaseStrategy):
    def __init__(self):
        super().__init__(name="dummy", symbols=["SPY"], timeframe="1m")

    def generate_signal(self, df, symbol):
        return TradeSignal(symbol=symbol, signal=Signal.HOLD, confidence=1.0, price=0.0)


def test_strategy_factory_register_and_create():
    StrategyFactory.register("dummy", DummyStrategy)
    strat = StrategyFactory.create("dummy")
    assert isinstance(strat, DummyStrategy)


def test_strategy_factory_unknown_key():
    with pytest.raises(KeyError):
        StrategyFactory.create("does_not_exist")

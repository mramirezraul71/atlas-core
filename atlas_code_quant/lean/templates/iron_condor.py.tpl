# LEAN algorithm template — Atlas Iron Condor
# F4 esqueleto. Reemplazar marcadores {{...}} antes de ejecutar con `lean backtest`.

from datetime import timedelta
from QuantConnect import Resolution
from QuantConnect.Algorithm import QCAlgorithm
from QuantConnect.Brokerages import BrokerageName
from QuantConnect.Securities import AccountType
from QuantConnect.Algorithm.Framework.Portfolio import EqualWeightingPortfolioConstructionModel
from QuantConnect.Algorithm.Framework.Execution import ImmediateExecutionModel


class AtlasIronCondor(QCAlgorithm):
    """Iron Condor delta-neutral con filtros DTE/strike inspirados en Radar."""

    def Initialize(self) -> None:
        self.SetStartDate({{start_year}}, {{start_month}}, {{start_day}})
        self.SetEndDate({{end_year}}, {{end_month}}, {{end_day}})
        self.SetCash({{cash}})
        self.SetBrokerageModel(BrokerageName.TradierBrokerage, AccountType.Margin)

        opt = self.AddOption("{{symbol}}", Resolution.Minute)
        opt.SetFilter(self._option_filter)
        self._underlying = opt.Symbol

        from .alpha_radar import RadarAlphaModel
        self.SetAlpha(RadarAlphaModel(min_score={{min_score}}))
        self.SetPortfolioConstruction(EqualWeightingPortfolioConstructionModel())
        self.SetExecution(ImmediateExecutionModel())

    def _option_filter(self, universe):
        # DTE 14-45, strikes amplios para cuatro patas
        return (universe.IncludeWeeklys()
                .Strikes(-6, +6)
                .Expiration(timedelta({{dte_min}}), timedelta({{dte_max}})))

    def OnData(self, data) -> None:
        pass

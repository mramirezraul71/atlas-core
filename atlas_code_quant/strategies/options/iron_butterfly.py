"""Iron Butterfly — F5.

Adapter conservador: reutiliza ``atlas_code_quant.iron_butterfly`` legacy y
añade el contrato ``StrategyPlan`` para integrarlo en la Strategy Factory.
"""
from __future__ import annotations

from dataclasses import dataclass

from atlas_code_quant.strategies.contracts import (
    OptionLeg,
    StrategyConfig,
    StrategyOpportunityRef,
    StrategyPlan,
)

try:
    from atlas_code_quant.iron_butterfly import IronButterflyBacktester
except Exception:  # pragma: no cover — paquete legacy siempre debería existir
    IronButterflyBacktester = None  # type: ignore[assignment]


@dataclass(slots=True)
class IronButterflyStrategy:
    name: str = "iron_butterfly"

    def create_backtester(self):
        if IronButterflyBacktester is None:
            raise RuntimeError("legacy IronButterflyBacktester unavailable")
        return IronButterflyBacktester()

    def build_plan(
        self,
        opportunity: StrategyOpportunityRef | dict,
        config: StrategyConfig | None = None,
    ) -> StrategyPlan:
        opp = (
            opportunity
            if isinstance(opportunity, StrategyOpportunityRef)
            else StrategyOpportunityRef.from_dict(opportunity)
        )
        cfg = config or StrategyConfig()

        if opp.direction != "neutral":
            return StrategyPlan(
                strategy=self.name,
                symbol=opp.symbol,
                direction=opp.direction,
                status="rejected",
                rationale="iron_butterfly assumes range-bound move",
                trace_id=opp.trace_id,
            )

        wing = cfg.width_usd
        legs = [
            # ATM short call + put (centro)
            OptionLeg(side="sell", right="call", strike_offset=0.0,
                      qty=cfg.qty, expiry_dte=cfg.dte_min),
            OptionLeg(side="sell", right="put",  strike_offset=0.0,
                      qty=cfg.qty, expiry_dte=cfg.dte_min),
            # alas
            OptionLeg(side="buy",  right="call", strike_offset=+wing,
                      qty=cfg.qty, expiry_dte=cfg.dte_min),
            OptionLeg(side="buy",  right="put",  strike_offset=-wing,
                      qty=cfg.qty, expiry_dte=cfg.dte_min),
        ]
        max_loss = min(cfg.max_loss_usd, wing * 100 * cfg.qty)
        notional = wing * 100 * cfg.qty * 2
        return StrategyPlan(
            strategy=self.name,
            symbol=opp.symbol,
            direction="neutral",
            legs=legs,
            notional_estimate_usd=notional,
            max_loss_estimate_usd=max_loss,
            horizon_min=opp.horizon_min,
            rationale=f"iron_butterfly wing={wing}",
            status="planned",
            trace_id=opp.trace_id,
            metadata={"legacy_backtester_available": IronButterflyBacktester is not None},
        )

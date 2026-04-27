"""Iron Condor esqueleto — F5.

Construye un Iron Condor delta-neutral (4 patas):
- short put OTM + long put deep OTM
- short call OTM + long call deep OTM
Solo construye plan si la dirección Radar es neutral (lo natural para Condor).
"""
from __future__ import annotations

from dataclasses import dataclass

from atlas_code_quant.strategies.contracts import (
    OptionLeg,
    StrategyConfig,
    StrategyOpportunityRef,
    StrategyPlan,
)


@dataclass(slots=True)
class IronCondorStrategy:
    name: str = "iron_condor"

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
                rationale="iron_condor optimal under neutral bias",
                trace_id=opp.trace_id,
            )

        wing = cfg.width_usd
        legs = [
            # Put side
            OptionLeg(side="sell", right="put",  strike_offset=-wing,
                      qty=cfg.qty, expiry_dte=cfg.dte_min),
            OptionLeg(side="buy",  right="put",  strike_offset=-2 * wing,
                      qty=cfg.qty, expiry_dte=cfg.dte_min),
            # Call side
            OptionLeg(side="sell", right="call", strike_offset=+wing,
                      qty=cfg.qty, expiry_dte=cfg.dte_min),
            OptionLeg(side="buy",  right="call", strike_offset=+2 * wing,
                      qty=cfg.qty, expiry_dte=cfg.dte_min),
        ]
        # Max loss aprox = wing * 100 * qty (- credit)
        max_loss = min(cfg.max_loss_usd, wing * 100 * cfg.qty)
        notional = wing * 100 * cfg.qty * 2  # dos lados
        return StrategyPlan(
            strategy=self.name,
            symbol=opp.symbol,
            direction="neutral",
            legs=legs,
            notional_estimate_usd=notional,
            max_loss_estimate_usd=max_loss,
            horizon_min=opp.horizon_min,
            rationale=f"iron_condor wing={wing} both_sides",
            status="planned",
            trace_id=opp.trace_id,
        )

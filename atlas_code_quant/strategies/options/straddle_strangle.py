"""Straddle / Strangle — F5.

Estrategias de volatilidad: straddle si la oportunidad indica volatilidad alta
y dirección neutral, strangle si se prefiere coste menor (strikes OTM).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from atlas_code_quant.strategies.contracts import (
    OptionLeg,
    StrategyConfig,
    StrategyOpportunityRef,
    StrategyPlan,
)


@dataclass(slots=True)
class StraddleStrangleStrategy:
    name: str = "straddle_strangle"
    variant: Literal["straddle", "strangle"] = "straddle"

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

        # Funciona mejor con bias neutral (busca volatilidad)
        if opp.direction != "neutral":
            return StrategyPlan(
                strategy=self.name,
                symbol=opp.symbol,
                direction=opp.direction,
                status="rejected",
                rationale="straddle/strangle requires neutral bias (volatility play)",
                trace_id=opp.trace_id,
            )

        offset = 0.0 if self.variant == "straddle" else cfg.width_usd
        legs = [
            OptionLeg(side="buy", right="call", strike_offset=+offset,
                      qty=cfg.qty, expiry_dte=cfg.dte_min),
            OptionLeg(side="buy", right="put",  strike_offset=-offset,
                      qty=cfg.qty, expiry_dte=cfg.dte_min),
        ]
        max_loss = cfg.cash_alloc_usd  # prima total pagada (debit)
        notional = max_loss
        return StrategyPlan(
            strategy=f"{self.name}.{self.variant}",
            symbol=opp.symbol,
            direction="neutral",
            legs=legs,
            notional_estimate_usd=notional,
            max_loss_estimate_usd=max_loss,
            horizon_min=opp.horizon_min,
            rationale=f"{self.variant} offset={offset}",
            status="planned",
            trace_id=opp.trace_id,
        )

"""Vertical spread esqueleto — F5.

Construye un plan de spread vertical (debit/credit) según la dirección del Radar:
- ``long`` → call debit spread (long ATM call + short OTM call).
- ``short`` → put debit spread (long ATM put + short OTM put).
- ``neutral`` → rechazado (use straddle/condor).
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
class VerticalSpreadStrategy:
    name: str = "vertical_spread"

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

        if opp.direction == "neutral":
            return StrategyPlan(
                strategy=self.name,
                symbol=opp.symbol,
                direction=opp.direction,
                status="rejected",
                rationale="vertical_spread requires directional bias",
                trace_id=opp.trace_id,
            )

        right = "call" if opp.direction == "long" else "put"
        # long ATM, short OTM al ancho configurado
        long_offset = 0.0
        short_offset = cfg.width_usd if right == "call" else -cfg.width_usd
        legs = [
            OptionLeg(side="buy", right=right, strike_offset=long_offset,
                      qty=cfg.qty, expiry_dte=cfg.dte_min),
            OptionLeg(side="sell", right=right, strike_offset=short_offset,
                      qty=cfg.qty, expiry_dte=cfg.dte_min),
        ]
        # max loss aprox: prima neta pagada (placeholder)
        max_loss = min(cfg.max_loss_usd, cfg.cash_alloc_usd)
        notional = cfg.width_usd * 100 * cfg.qty
        return StrategyPlan(
            strategy=self.name,
            symbol=opp.symbol,
            direction=opp.direction,
            legs=legs,
            notional_estimate_usd=notional,
            max_loss_estimate_usd=max_loss,
            horizon_min=opp.horizon_min,
            rationale=f"{right} debit spread width={cfg.width_usd}",
            status="planned",
            trace_id=opp.trace_id,
        )

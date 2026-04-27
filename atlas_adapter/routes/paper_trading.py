"""Endpoints paper trading — F9.4.

Permiten al experimento E2E abrir/cerrar posiciones paper a través de HTTP,
asegurando que las métricas en /api/system/metrics reflejen el estado real.

Todas las operaciones son paper-first: ATLAS_LIVE_TRADING_ENABLED=false y
ATLAS_TRADIER_DRY_RUN=true son obligatorios. Si live está habilitado, el
endpoint devuelve 403 inmediatamente.
"""
from __future__ import annotations

import os
from typing import Any

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field

from atlas_code_quant.execution.position_monitor import PositionMonitor
from atlas_code_quant.execution.runtime import (
    get_paper_broker,
    get_trade_journal,
)
from atlas_code_quant.strategies.contracts import (
    OptionLeg,
    StrategyPlan,
)


class _LegIn(BaseModel):
    side: str
    right: str
    strike_offset: float
    qty: int = 1
    expiry_dte: int = 14


class _OpenIn(BaseModel):
    strategy: str
    symbol: str
    direction: str = "long"
    legs: list[_LegIn] = Field(default_factory=list)
    notional_estimate_usd: float = 500.0
    max_loss_estimate_usd: float = 200.0
    horizon_min: int = 30
    rationale: str = "e2e"
    trace_id: str
    entry_price: float
    take_profit_pct: float = 0.50
    stop_loss_pct: float = 1.00
    time_stop_seconds: int | None = None


class _CloseIn(BaseModel):
    position_id: str
    exit_price: float
    reason: str = "manual"


class _MonitorIn(BaseModel):
    position_id: str
    last_price: float


def _check_paper_first() -> None:
    live = os.environ.get("ATLAS_LIVE_TRADING_ENABLED", "false").lower()
    if live in {"1", "true", "yes", "on"}:
        raise HTTPException(
            status_code=403,
            detail="live_trading_enabled_blocks_paper_endpoint",
        )


def build_paper_trading_router() -> APIRouter:
    router = APIRouter(tags=["PaperTrading"])

    @router.post("/api/paper/open")
    def paper_open(body: _OpenIn) -> dict[str, Any]:
        _check_paper_first()
        plan = StrategyPlan(
            strategy=body.strategy,
            symbol=body.symbol,
            direction=body.direction,
            legs=[
                OptionLeg(
                    side=leg.side,  # type: ignore[arg-type]
                    right=leg.right,  # type: ignore[arg-type]
                    strike_offset=leg.strike_offset,
                    qty=leg.qty,
                    expiry_dte=leg.expiry_dte,
                )
                for leg in body.legs
            ],
            notional_estimate_usd=body.notional_estimate_usd,
            max_loss_estimate_usd=body.max_loss_estimate_usd,
            horizon_min=body.horizon_min,
            rationale=body.rationale,
            status="planned",
            trace_id=body.trace_id,
        )
        broker = get_paper_broker()
        journal = get_trade_journal()
        pos = broker.open_position(
            plan,
            entry_price=body.entry_price,
            take_profit_pct=body.take_profit_pct,
            stop_loss_pct=body.stop_loss_pct,
            time_stop_seconds=body.time_stop_seconds,
            trace_id=body.trace_id,
        )
        journal.record_open(pos)
        return {"position": pos.to_dict()}

    @router.post("/api/paper/close")
    def paper_close(body: _CloseIn) -> dict[str, Any]:
        _check_paper_first()
        broker = get_paper_broker()
        journal = get_trade_journal()
        try:
            pos = broker.close_position(
                body.position_id,
                exit_price=body.exit_price,
                reason=body.reason,
            )
        except KeyError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc
        journal.record_close(pos)
        return {"position": pos.to_dict()}

    @router.post("/api/paper/monitor")
    def paper_monitor(body: _MonitorIn) -> dict[str, Any]:
        _check_paper_first()
        broker = get_paper_broker()
        pos = broker.get_open_position(body.position_id)
        if pos is None:
            raise HTTPException(status_code=404, detail="unknown_position")
        decision = PositionMonitor().evaluate(pos, last_price=body.last_price)
        return {
            "position_id": body.position_id,
            "action": decision.action,
            "reason": decision.reason,
            "suggested_exit_price": decision.suggested_exit_price,
        }

    return router

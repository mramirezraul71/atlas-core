"""Endpoint /api/system/metrics consolidado — F9.4.

Expone métricas runtime de los subsistemas paper-first:
- radar: símbolos en universo, opportunities cacheadas (best-effort).
- strategy: contadores de planes generados (best-effort).
- execution: cash, posiciones abiertas/cerradas, PnL realizado total.
- risk: límites configurados vs uso (best-effort).
- journal: número de entradas y ciclos completos.

Diseño paper-first: si un subsistema no está disponible en este proceso,
el campo correspondiente reporta ``available=false`` con ``degraded=true``
en lugar de fallar.
"""
from __future__ import annotations

import os
import time
from typing import Any

from fastapi import APIRouter

from atlas_code_quant.execution.runtime import (
    get_paper_broker,
    get_trade_journal,
)


def _radar_block() -> dict[str, Any]:
    try:
        from atlas_adapter.services.universe_provider import (  # type: ignore
            UniverseProvider,
        )
        univ = UniverseProvider().refresh()
        symbols = [getattr(e, "symbol", str(e)) for e in univ]
        return {
            "available": True,
            "universe_size": len(symbols),
            "symbols_sample": symbols[:10],
        }
    except Exception as exc:  # pragma: no cover - defensivo
        return {"available": False, "degraded": True, "error": str(exc)[:200]}


def _strategy_block() -> dict[str, Any]:
    try:
        from atlas_code_quant.strategies.factory import StrategyFactory  # noqa: F401
        return {
            "available": True,
            "factory_strategies": [
                "vertical_spread",
                "iron_condor",
                "iron_butterfly",
                "straddle_strangle",
            ],
        }
    except Exception as exc:
        return {"available": False, "degraded": True, "error": str(exc)[:200]}


def _execution_block() -> dict[str, Any]:
    broker = get_paper_broker()
    open_pos = broker.open_positions()
    closed_pos = broker.closed_positions()
    realized_pnl = round(
        sum((p.realized_pnl_usd or 0.0) for p in closed_pos), 2
    )
    return {
        "available": True,
        "cash_usd": round(broker.cash_usd, 2),
        "positions_open": len(open_pos),
        "positions_closed": len(closed_pos),
        "realized_pnl_usd_total": realized_pnl,
        "live_trading_enabled": os.environ.get(
            "ATLAS_LIVE_TRADING_ENABLED", "false"
        ).lower() in {"1", "true", "yes", "on"},
        "tradier_dry_run": os.environ.get(
            "ATLAS_TRADIER_DRY_RUN", "true"
        ).lower() in {"1", "true", "yes", "on"},
    }


def _risk_block() -> dict[str, Any]:
    try:
        from atlas_code_quant.risk.policy import RiskPolicy  # type: ignore
        rp = RiskPolicy()
        return {
            "available": True,
            "max_loss_per_trade_usd": getattr(rp, "max_loss_per_trade_usd", None),
            "max_daily_loss_usd": getattr(rp, "max_daily_loss_usd", None),
            "max_concurrent_positions": getattr(
                rp, "max_concurrent_positions", None
            ),
        }
    except Exception as exc:
        return {"available": False, "degraded": True, "error": str(exc)[:200]}


def _journal_block() -> dict[str, Any]:
    journal = get_trade_journal()
    entries = journal.entries()
    opens = sum(1 for e in entries if e.event == "trade_open")
    closes = sum(1 for e in entries if e.event == "trade_close")
    return {
        "available": True,
        "entries_total": len(entries),
        "trade_opens": opens,
        "trade_closes": closes,
        "complete_cycles": closes,  # cada close empareja con su open
    }


def build_system_metrics_router() -> APIRouter:
    router = APIRouter(tags=["System"])

    @router.get("/api/system/metrics")
    def system_metrics() -> dict[str, Any]:
        return {
            "ts": time.time(),
            "radar": _radar_block(),
            "strategy": _strategy_block(),
            "execution": _execution_block(),
            "risk": _risk_block(),
            "journal": _journal_block(),
        }

    return router

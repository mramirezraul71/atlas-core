"""Rutas FastAPI para el módulo OptionStrat interno de ATLAS-Quant.

Endpoints
---------
POST /options/strategies/preview    — Calcular payoff/greeks de una estrategia
GET  /options/templates             — Lista de plantillas disponibles
POST /options/templates/build       — Construir estrategia desde plantilla
POST /options/scenario              — Heatmap escenario precio×tiempo
POST /options/portfolio/add         — Añadir estrategia a cartera
DELETE /options/portfolio/{name}    — Cerrar estrategia
GET  /options/portfolio/summary     — Resumen de cartera + greeks agregadas
POST /options/portfolio/greeks      — Griegas totales con precios actuales
GET  /options/iv-rank               — IV Rank proxy (paper): ATM IV vs rango RV histórico
POST /options/briefing             — Briefing de sesión Fase A (JSON, paper/test)
GET  /options/close-candidates     — Reporte paper de candidatos a cierre (sin órdenes)
POST /options/intent               — Intent operativo pre-selector (paper/test)
POST /options/paper-session-plan   — Pipeline paper: briefing + intent + plan de entrada (sin órdenes)
"""
from __future__ import annotations

import logging
import time
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field
from atlas_code_quant.config.settings import settings
from atlas_code_quant.monitoring.strategy_tracker import StrategyTracker

try:
    from options.strategy_engine import (
        MarketSnapshot, Strategy, breakeven_points, greeks,
        payoff_at_expiry, pnl_today, scenario_pnl,
        strategy_from_dict, strategy_risk_summary, strategy_to_dict,
    )
    from options.strategy_templates import TEMPLATE_REGISTRY, list_templates
    from options.portfolio_analyzer import PortfolioAnalyzer
except ModuleNotFoundError:
    from atlas_code_quant.options.strategy_engine import (
        MarketSnapshot, Strategy, breakeven_points, greeks,
        payoff_at_expiry, pnl_today, scenario_pnl,
        strategy_from_dict, strategy_risk_summary, strategy_to_dict,
    )
    from atlas_code_quant.options.strategy_templates import TEMPLATE_REGISTRY, list_templates
    from atlas_code_quant.options.portfolio_analyzer import PortfolioAnalyzer

logger = logging.getLogger("atlas.options.api")

router = APIRouter(prefix="/options", tags=["OptionStrat"])

# Instancia global del analizador de cartera
_PORTFOLIO_PATH = Path("data/options_portfolio.json")
_portfolio = PortfolioAnalyzer(storage_path=_PORTFOLIO_PATH)
_tracker = StrategyTracker()


# ---------------------------------------------------------------------------
# Schemas Pydantic
# ---------------------------------------------------------------------------

class LegSchema(BaseModel):
    leg_type: str = "option"       # "option" | "linear"
    # Campos opción
    underlying_symbol: Optional[str] = None
    expiry: Optional[str] = None   # ISO date string
    strike: Optional[float] = None
    option_type: Optional[str] = None   # "call" | "put"
    side: str = "long"             # "long" | "short"
    quantity: int = 1
    premium: float = 0.0
    multiplier: int = 100
    iv: float = 0.25
    underlying_price: float = 0.0
    # Campos lineales
    symbol: Optional[str] = None
    asset_class: Optional[str] = "stock"
    entry_price: float = 0.0


class StrategySchema(BaseModel):
    name: str
    underlying: str = ""
    legs: List[LegSchema]
    metadata: Dict = Field(default_factory=dict)


class MarketSnapshotSchema(BaseModel):
    underlying_price: float
    risk_free_rate: float = 0.05
    iv_overrides: Dict[str, float] = Field(default_factory=dict)
    dte_override: Optional[int] = None


class PreviewRequest(BaseModel):
    strategy: StrategySchema
    market: MarketSnapshotSchema
    n_points: int = 300      # puntos en la curva
    spot_range_pct: float = 0.35  # ±35% del spot


class TemplateRequest(BaseModel):
    template_name: str
    symbol: str
    spot: float
    dte: int = 30
    # Strikes como precios absolutos o como offsets relativos al spot
    strikes: Dict[str, float] = Field(default_factory=dict)
    quantity: int = 1
    iv: float = 0.25
    extra: Dict = Field(default_factory=dict)


class ScenarioRequest(BaseModel):
    strategy: StrategySchema
    spot_range: List[float]       # [min, max] del subyacente
    dte_values: List[int]         # días hasta vencimiento para columnas
    iv_multiplier: float = 1.0
    risk_free_rate: float = 0.05
    n_spots: int = 20


class AddToPortfolioRequest(BaseModel):
    strategy: StrategySchema
    market: MarketSnapshotSchema
    quantity: int = 1
    notes: str = ""
    tags: List[str] = Field(default_factory=list)


class PortfolioSummaryRequest(BaseModel):
    market_prices: Dict[str, float]   # {symbol: price}
    risk_free_rate: float = 0.05


class SessionBriefingRequest(BaseModel):
    """Briefing cuantitativo mínimo (paper); no ejecuta órdenes."""

    symbol: str
    direction: str = "neutral"
    regime: str = "ranging"
    gamma_regime: Optional[str] = None
    dte_mode: Optional[str] = None
    days_to_event: Optional[int] = None
    event_near: bool = False
    spot: Optional[float] = None
    support_levels: Optional[List[float]] = None
    resistance_levels: Optional[List[float]] = None
    breach_context: Optional[Dict] = None


class OptionsIntentRequest(BaseModel):
    briefing: Dict
    manual_overrides: Optional[Dict] = None


class PaperSessionPlanRequest(BaseModel):
    """Plan de sesión paper-only (briefing → intent → entry plan). Sin ejecución ni broker live."""

    symbol: str
    direction: Optional[str] = None
    regime: Optional[str] = None
    gamma_regime: Optional[str] = None
    dte_mode: Optional[str] = None
    days_to_event: Optional[int] = None
    event_near: Optional[bool] = None
    spot: Optional[float] = None
    support_levels: Optional[List[float]] = None
    resistance_levels: Optional[List[float]] = None
    breach_context: Optional[Dict] = None
    capital: Optional[float] = None
    manual_intent_overrides: Optional[Dict] = None
    manual_entry_overrides: Optional[Dict] = None


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _leg_to_dict(leg: LegSchema) -> dict:
    return leg.model_dump()


def _build_strategy(s: StrategySchema) -> Strategy:
    d = {
        "name": s.name,
        "underlying": s.underlying,
        "legs": [l.model_dump() for l in s.legs],
        "metadata": s.metadata,
    }
    return strategy_from_dict(d)


def _build_market(m: MarketSnapshotSchema) -> MarketSnapshot:
    iv_overrides = {float(k): v for k, v in m.iv_overrides.items()}
    return MarketSnapshot(
        underlying_price=m.underlying_price,
        risk_free_rate=m.risk_free_rate,
        iv_overrides=iv_overrides,
        dte_override=m.dte_override,
    )


def _make_grid(spot: float, pct: float, n: int) -> np.ndarray:
    lo = max(0.01, spot * (1 - pct))
    hi = spot * (1 + pct)
    return np.linspace(lo, hi, n)


def _sync_optionstrat_from_broker() -> Dict[str, int]:
    """Sincroniza OptionStrat con posiciones reales del broker (best-effort)."""
    try:
        snapshot = _tracker.snapshot(account_scope=settings.tradier_default_scope)
        return _portfolio.sync_from_broker_groups(
            groups=snapshot.groups,
            quote_index=snapshot.quote_index,
            source=f"tradier_{settings.tradier_default_scope}",
        )
    except Exception as exc:
        logger.debug("OptionStrat broker sync skipped: %s", exc)
        return {"added": 0, "updated": 0, "closed": 0, "active": 0}


def _mirror_only_enabled() -> bool:
    return bool(settings.optionstrat_broker_mirror_only)


# ---------------------------------------------------------------------------
# Endpoints
# ---------------------------------------------------------------------------

@router.post("/strategies/preview")
async def preview_strategy(req: PreviewRequest):
    """Calcular payoff, PnL T+0, breakevens, griegas y resumen de riesgo.

    Devuelve arrays listos para graficar en el frontend.
    """
    t0 = time.perf_counter()
    try:
        strategy = _build_strategy(req.strategy)
        market   = _build_market(req.market)
        S_grid   = _make_grid(market.underlying_price, req.spot_range_pct, req.n_points)

        payoff   = payoff_at_expiry(strategy, S_grid).tolist()
        pnl_0    = pnl_today(strategy, market, S_grid).tolist()
        bk       = breakeven_points(strategy, S_grid)
        g        = greeks(strategy, market)
        risk     = strategy_risk_summary(strategy, market, S_grid)

        ms = round((time.perf_counter() - t0) * 1000, 1)
        return {
            "ok": True,
            "ms": ms,
            "data": {
                "strategy": strategy_to_dict(strategy),
                "s_grid": [round(s, 4) for s in S_grid.tolist()],
                "payoff_expiry": [round(p, 2) for p in payoff],
                "pnl_today": [round(p, 2) for p in pnl_0],
                "breakevens": [round(b, 2) for b in bk],
                "greeks": g.to_dict(),
                "risk_summary": risk.to_dict(),
            },
        }
    except Exception as exc:
        logger.exception("preview_strategy error")
        raise HTTPException(status_code=400, detail=str(exc))


@router.get("/templates")
async def get_templates():
    """Lista todas las plantillas de estrategia disponibles."""
    return {"ok": True, "data": list_templates()}


@router.post("/templates/build")
async def build_from_template(req: TemplateRequest):
    """Construye una estrategia desde una plantilla por nombre.

    Parámetros básicos: symbol, spot, dte, strikes, iv.
    Los strikes en `strikes` se mapean a los parámetros de cada plantilla.
    """
    fn = TEMPLATE_REGISTRY.get(req.template_name)
    if fn is None:
        raise HTTPException(status_code=404, detail=f"Template '{req.template_name}' not found")

    try:
        # Construir kwargs combinando strikes y extra
        kwargs = {
            "symbol": req.symbol,
            "spot": req.spot,
            "dte": req.dte,
            "quantity": req.quantity,
            "iv": req.iv,
            **{k: float(v) for k, v in req.strikes.items()},
            **req.extra,
        }
        strategy = fn(**kwargs)
        return {"ok": True, "data": strategy_to_dict(strategy)}
    except TypeError as exc:
        raise HTTPException(status_code=400, detail=f"Parámetros incorrectos para '{req.template_name}': {exc}")
    except Exception as exc:
        logger.exception("build_from_template error")
        raise HTTPException(status_code=400, detail=str(exc))


@router.post("/scenario")
async def compute_scenario(req: ScenarioRequest):
    """Heatmap de PnL: filas = precio subyacente, columnas = DTE.

    Devuelve matriz lista para visualizar como heatmap o superficie 3D.
    """
    try:
        strategy = _build_strategy(req.strategy)
        lo, hi = req.spot_range[0], req.spot_range[1]
        S_values = list(np.linspace(lo, hi, req.n_spots))

        matrix = scenario_pnl(
            strategy=strategy,
            S_values=[float(s) for s in S_values],
            dte_values=req.dte_values,
            iv_multiplier=req.iv_multiplier,
            r=req.risk_free_rate,
        )

        return {
            "ok": True,
            "data": {
                "s_values": [round(s, 2) for s in S_values],
                "dte_values": req.dte_values,
                "pnl_matrix": [[round(v, 2) for v in row] for row in matrix.tolist()],
            },
        }
    except Exception as exc:
        logger.exception("compute_scenario error")
        raise HTTPException(status_code=400, detail=str(exc))


@router.post("/portfolio/add")
async def add_to_portfolio(req: AddToPortfolioRequest):
    """Añade una estrategia a la cartera activa."""
    if _mirror_only_enabled():
        raise HTTPException(
            status_code=403,
            detail="OptionStrat en modo espejo broker: alta manual deshabilitada",
        )
    try:
        strategy = _build_strategy(req.strategy)
        market   = _build_market(req.market)
        entry = _portfolio.add_strategy(
            strategy=strategy,
            market=market,
            quantity=req.quantity,
            notes=req.notes,
            tags=req.tags,
        )
        return {
            "ok": True,
            "data": {
                "name": entry.name,
                "opened_at": entry.opened_at.isoformat(),
                "status": entry.status,
            },
        }
    except Exception as exc:
        logger.exception("add_to_portfolio error")
        raise HTTPException(status_code=400, detail=str(exc))


@router.delete("/portfolio/{strategy_name}")
async def close_strategy(strategy_name: str):
    """Cierra (marca como closed) una estrategia en la cartera."""
    if _mirror_only_enabled():
        raise HTTPException(
            status_code=403,
            detail="OptionStrat en modo espejo broker: cierre manual deshabilitado",
        )
    removed = _portfolio.remove_strategy(strategy_name)
    if not removed:
        raise HTTPException(status_code=404, detail=f"Estrategia '{strategy_name}' no encontrada o ya cerrada")
    return {"ok": True, "data": {"closed": strategy_name}}


@router.post("/portfolio/summary")
async def portfolio_summary(req: PortfolioSummaryRequest):
    """Resumen completo de la cartera: PnL, griegas, distribución, risk flags."""
    try:
        sync_stats = _sync_optionstrat_from_broker()
        summary = _portfolio.compute_summary(
            market_prices=req.market_prices,
            risk_free_rate=req.risk_free_rate,
        )
        payload = summary.to_dict()
        payload["history"] = _portfolio.history(limit=200)
        payload["sync"] = sync_stats
        payload["mirror_only"] = _mirror_only_enabled()
        return {"ok": True, "data": payload}
    except Exception as exc:
        logger.exception("portfolio_summary error")
        raise HTTPException(status_code=400, detail=str(exc))


@router.post("/portfolio/greeks")
async def portfolio_greeks(req: PortfolioSummaryRequest):
    """Griegas totales de la cartera (respuesta rápida sin PnL completo)."""
    try:
        _sync_optionstrat_from_broker()
        g = _portfolio.compute_aggregate_greeks(
            market_prices=req.market_prices,
            risk_free_rate=req.risk_free_rate,
        )
        return {"ok": True, "data": g.to_dict()}
    except Exception as exc:
        raise HTTPException(status_code=400, detail=str(exc))


@router.get("/portfolio/status")
async def portfolio_status():
    """Estado rápido de la cartera (sin cálculos pesados)."""
    _sync_optionstrat_from_broker()
    payload = _portfolio.status()
    payload["mirror_only"] = _mirror_only_enabled()
    payload["tradier_scope"] = settings.tradier_default_scope
    return {"ok": True, "data": payload}


@router.get("/portfolio/history")
async def portfolio_history(limit: int = 200):
    """Histórico de estrategias cerradas (persistente)."""
    return {"ok": True, "data": _portfolio.history(limit=limit)}


@router.get("/config")
async def optionstrat_config():
    return {
        "ok": True,
        "data": {
            "mirror_only": _mirror_only_enabled(),
            "tradier_scope": settings.tradier_default_scope,
        },
    }


@router.post("/briefing")
async def session_briefing_endpoint(req: SessionBriefingRequest):
    """Briefing de sesión Fase A: IV, expected move 1σ, estrategia sugerida (``pick_strategy``). Paper/test."""
    try:
        from backtesting.winning_probability import TradierClient
        from execution.option_chain_cache import OptionChainCache
        from options.iv_rank_calculator import IVRankCalculator
        from options.session_briefing import SessionBriefingEngine
    except ModuleNotFoundError:
        from atlas_code_quant.backtesting.winning_probability import TradierClient
        from atlas_code_quant.execution.option_chain_cache import OptionChainCache
        from atlas_code_quant.options.iv_rank_calculator import IVRankCalculator
        from atlas_code_quant.options.session_briefing import SessionBriefingEngine

    sym = (req.symbol or "").strip().upper()
    if not sym:
        raise HTTPException(status_code=400, detail="symbol is required")

    client = TradierClient(scope="paper")
    calc = IVRankCalculator(client, chain_cache=OptionChainCache(), scope="paper")
    engine = SessionBriefingEngine(calc)
    try:
        payload = engine.build_briefing(
            sym,
            direction=req.direction,
            regime=req.regime,
            gamma_regime=req.gamma_regime,
            dte_mode=req.dte_mode,
            days_to_event=req.days_to_event,
            event_near=req.event_near,
            spot=req.spot,
            support_levels=req.support_levels,
            resistance_levels=req.resistance_levels,
            breach_context=req.breach_context,
        )
    except Exception as exc:
        logger.exception("session_briefing error")
        raise HTTPException(status_code=502, detail=str(exc)) from exc
    return {"ok": True, "data": payload}


@router.get("/close-candidates")
async def close_candidates_report(include_hold: bool = False):
    """Reporte paper-only de candidatos a cierre. No ejecuta órdenes ni muta broker."""
    try:
        from execution.auto_close_engine import AutoCloseEngine
        from options.auto_close_adapter import AutoCloseReportBuilder
    except ModuleNotFoundError:
        from atlas_code_quant.execution.auto_close_engine import AutoCloseEngine
        from atlas_code_quant.options.auto_close_adapter import AutoCloseReportBuilder

    builder = AutoCloseReportBuilder(_portfolio, AutoCloseEngine())
    payload = builder.build_report(include_hold=include_hold)
    return {"ok": True, "data": payload}


@router.post("/intent")
async def options_intent_endpoint(req: OptionsIntentRequest):
    """Intent operativo pre-selector. Solo analítico (paper), sin ejecución."""
    try:
        from options.options_intent_router import OptionsIntentRouter
    except ModuleNotFoundError:
        from atlas_code_quant.options.options_intent_router import OptionsIntentRouter

    router_intent = OptionsIntentRouter()
    payload = router_intent.build_intent(
        req.briefing,
        manual_overrides=req.manual_overrides,
    )
    return {"ok": True, "data": payload}


@router.post("/paper-session-plan")
async def paper_session_plan_endpoint(req: PaperSessionPlanRequest):
    """Pipeline único paper-only: briefing enriquecido, intent y plan de entrada. Sin órdenes ni live."""
    try:
        from backtesting.winning_probability import TradierClient
        from execution.option_chain_cache import OptionChainCache
        from options.iv_rank_calculator import IVRankCalculator
        from options.session_briefing import SessionBriefingEngine
        from options.paper_entry_planner import PaperEntryPlanner
        from options.paper_session_orchestrator import PaperSessionOrchestrator
        from options.options_intent_router import OptionsIntentRouter
    except ModuleNotFoundError:
        from atlas_code_quant.backtesting.winning_probability import TradierClient
        from atlas_code_quant.execution.option_chain_cache import OptionChainCache
        from atlas_code_quant.options.iv_rank_calculator import IVRankCalculator
        from atlas_code_quant.options.session_briefing import SessionBriefingEngine
        from atlas_code_quant.options.paper_entry_planner import PaperEntryPlanner
        from atlas_code_quant.options.paper_session_orchestrator import PaperSessionOrchestrator
        from atlas_code_quant.options.options_intent_router import OptionsIntentRouter

    sym = (req.symbol or "").strip().upper()
    if not sym:
        raise HTTPException(status_code=400, detail="symbol is required")

    client = TradierClient(scope="paper")
    calc = IVRankCalculator(client, chain_cache=OptionChainCache(), scope="paper")
    engine = SessionBriefingEngine(calc)
    orchestrator = PaperSessionOrchestrator(
        engine,
        OptionsIntentRouter(),
        PaperEntryPlanner(),
    )
    try:
        payload = orchestrator.build_session_plan(
            symbol=sym,
            direction=req.direction,
            regime=req.regime,
            gamma_regime=req.gamma_regime,
            dte_mode=req.dte_mode,
            days_to_event=req.days_to_event,
            event_near=req.event_near,
            spot=req.spot,
            support_levels=req.support_levels,
            resistance_levels=req.resistance_levels,
            breach_context=req.breach_context,
            capital=req.capital,
            manual_intent_overrides=req.manual_intent_overrides,
            manual_entry_overrides=req.manual_entry_overrides,
        )
    except Exception as exc:
        logger.exception("paper_session_plan error")
        raise HTTPException(status_code=502, detail=str(exc)) from exc
    return {"ok": True, "data": payload}


@router.get("/iv-rank")
async def iv_rank_query(symbol: str, dte_target: Optional[int] = None):
    """IV Rank proxy (ATM IV vs rango histórico de vol realizada). Solo paper/sandbox."""
    try:
        from backtesting.winning_probability import TradierClient
        from execution.option_chain_cache import OptionChainCache
        from options.iv_rank_calculator import IVRankCalculator
    except ModuleNotFoundError:
        from atlas_code_quant.backtesting.winning_probability import TradierClient
        from atlas_code_quant.execution.option_chain_cache import OptionChainCache
        from atlas_code_quant.options.iv_rank_calculator import IVRankCalculator

    sym = (symbol or "").strip().upper()
    if not sym:
        raise HTTPException(status_code=400, detail="symbol is required")

    # Fase A: solo lectura paper — no exponer a cuentas live desde este endpoint.
    client = TradierClient(scope="paper")
    calc = IVRankCalculator(client, chain_cache=OptionChainCache(), scope="paper")
    try:
        payload = calc.get_iv_rank(sym, dte_target=dte_target)
    except Exception as exc:
        logger.exception("iv_rank_query error")
        raise HTTPException(status_code=502, detail=str(exc)) from exc
    return {"ok": True, "data": payload}

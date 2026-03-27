"""Atlas Code-Quant — API REST para integración con Atlas/ROS2.

Endpoints:
    GET  /health              — Estado del sistema
    POST /signal              — Evaluar señal para un símbolo
    POST /order               — Enviar orden de trading
    GET  /positions           — Posiciones abiertas + PnL
    POST /strategy/activate   — Activar estrategia
    POST /strategy/deactivate — Desactivar estrategia
    GET  /strategies          — Lista de estrategias registradas
    POST /backtest            — Ejecutar backtest completo
    POST /backtest/quick      — Backtest rápido con datos locales
    GET  /backtest/reports    — Lista de reportes generados

Puerto por defecto: 8792
"""
from __future__ import annotations

import asyncio
import math
import time
import logging
from datetime import datetime

from pathlib import Path as _PathLib

from fastapi import FastAPI, HTTPException, Header, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from api.decorators import require_live_confirmation
from api.schemas import (
    EvalSignalRequest, ActivateStrategyRequest, OrderRequest, WinningProbabilityRequest,
    SignalResponse, PortfolioResponse, HealthResponse, StdResponse, QuantStatusPayload,
    EmergencyStopRequest, JournalEntriesPayload, JournalStatsPayload, OperationConfigPayload,
    OperationCyclePayload, OperationCycleRequest, OperationStatusPayload,
    VisionCalibrationMovePosePayload, VisionCalibrationResetPayload, VisionCalibrationSamplePayload,
    VisionCalibrationStartPayload, VisionCalibrationStatusPayload,
    StrategySelectorPayload, StrategySelectorRequest,
    ScannerConfigPayload, ScannerControlRequest, ScannerReportPayload, ScannerStatusPayload,
    StatusEnum, SignalEnum,
    LoopStartRequest, VisionProviderRequest,
)
from backtesting.winning_probability import SUPPORTED_STRATEGIES, get_winning_probability
from config.settings import settings
from execution.account_manager import AccountManager
from execution.tradier_execution import (
    TradierOrderBlocked,
    is_opening_order as tradier_is_opening_order,
    route_order_to_tradier,
    should_route_to_tradier,
)
from journal.service import TradingJournalService
from journal.sync import JournalSyncService
from learning.adaptive_policy import AdaptiveLearningService
from monitoring.canonical_snapshot import CanonicalSnapshotService
from monitoring.strategy_tracker import StrategyTracker
from operations.auton_executor import AutonExecutorService
from operations.brain_bridge import QuantBrainBridge
from operations.journal_pro import JournalProService
from operations.operation_center import OperationCenter
from operations.sensor_vision import SensorVisionService
from operations.vision_calibration import VisionCalibrationService
from scanner.opportunity_scanner import OpportunityScannerService
from selector.strategy_selector import StrategySelectorService

# ── OptionStrat ───────────────────────────────────────────────────────────────
from api.routes.options import router as options_router

# ── Fase 3: alertas, visión, retraining ──────────────────────────────────────
from operations.alert_dispatcher import get_alert_dispatcher
from learning.visual_state_builder import get_visual_state_builder
from learning.retraining_scheduler import get_retraining_scheduler

logger = logging.getLogger("quant.api")
_START_TIME = time.time()
_ACCOUNT_MANAGER = AccountManager()
_BRAIN_BRIDGE = QuantBrainBridge()
_ADAPTIVE_LEARNING = AdaptiveLearningService()
_STRATEGY_TRACKER = StrategyTracker()
_CANONICAL_SNAPSHOT = CanonicalSnapshotService(_STRATEGY_TRACKER)
_JOURNAL = TradingJournalService(_STRATEGY_TRACKER, _BRAIN_BRIDGE)
_JOURNAL_SYNC = JournalSyncService(_JOURNAL)
_VISION = SensorVisionService()
_AUTON_EXECUTOR = AutonExecutorService()
_JOURNAL_PRO = JournalProService(_JOURNAL)
_SCANNER = OpportunityScannerService(_ADAPTIVE_LEARNING)
_VISION_CALIBRATION = VisionCalibrationService()
_SELECTOR = StrategySelectorService(_STRATEGY_TRACKER, _ADAPTIVE_LEARNING)
_OPERATION_CENTER = OperationCenter(
    tracker=_STRATEGY_TRACKER,
    journal=_JOURNAL_PRO,
    vision=_VISION,
    executor=_AUTON_EXECUTOR,
    brain=_BRAIN_BRIDGE,
    learning=_ADAPTIVE_LEARNING,
)

app = FastAPI(
    title="Atlas Code-Quant API",
    description="Trading algorítmico con IA — Interfaz para Atlas/ROS2",
    version="0.1.0",
    docs_url="/docs",
    redoc_url="/redoc",
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# ── Dashboard estático ────────────────────────────────────────────
_STATIC_DIR = _PathLib(__file__).resolve().parent / "static"
if _STATIC_DIR.exists():
    app.mount("/static", StaticFiles(directory=str(_STATIC_DIR)), name="static")

@app.get("/ui", include_in_schema=False)
async def dashboard_ui():
    """Atlas Code-Quant Dashboard v1.0.0"""
    idx = _STATIC_DIR / "index.html"
    if idx.exists():
        return FileResponse(str(idx))
    raise HTTPException(status_code=404, detail="Dashboard not found")


@app.get("/options/ui", include_in_schema=False)
async def options_ui():
    """OptionStrat UI — constructor y analizador de estrategias de opciones"""
    idx = _STATIC_DIR / "options" / "index.html"
    if idx.exists():
        return FileResponse(str(idx))
    raise HTTPException(status_code=404, detail="OptionStrat UI not found")

app.include_router(options_router)


@app.on_event("startup")
async def preload_tradier_sessions() -> None:
    _JOURNAL.init_db()
    for scope, token in (("paper", settings.tradier_paper_token), ("live", settings.tradier_live_token)):
        if not token:
            continue
        try:
            _ACCOUNT_MANAGER.resolve(account_scope=scope)  # type: ignore[arg-type]
            logger.info("Tradier %s session preloaded", scope)
        except Exception:
            logger.exception("Unable to preload Tradier %s session", scope)
    await _JOURNAL_SYNC.start()
    if settings.scanner_auto_start and settings.scanner_enabled:
        await _SCANNER.start()
    # Fase 3 — inicializar singletons
    try:
        await get_alert_dispatcher().start()
        logger.info("AlertDispatcher iniciado")
    except Exception:
        logger.exception("AlertDispatcher: fallo en startup (no crítico)")


@app.on_event("shutdown")
async def stop_background_services() -> None:
    global _auto_cycle_task
    await _JOURNAL_SYNC.stop()
    await _SCANNER.stop()
    if _auto_cycle_task and not _auto_cycle_task.done():
        _AUTO_CYCLE_STATE["running"] = False
        _auto_cycle_task.cancel()
        try:
            await _auto_cycle_task
        except (asyncio.CancelledError, Exception):
            pass
    # Fase 3 — apagar dispatcher
    try:
        await get_alert_dispatcher().stop()
    except Exception:
        pass
    # Fase 3 — apagar retraining schedulers activos
    try:
        await get_retraining_scheduler().stop()
    except Exception:
        pass

# ── Registry global (se popula al iniciar el motor) ──────────────────────────
_strategies: dict = {}
_portfolio = None

# ── Auto-cycle loop (scanner → operation_center bridge) ──────────────────────
_AUTO_CYCLE_STATE: dict = {
    "running": False,
    "cycle_count": 0,
    "last_cycle_at": None,
    "last_candidate_symbol": None,
    "last_action": None,
    "last_result": None,
    "loop_interval_sec": 120,
    "max_per_cycle": 1,
    "started_at": None,
    "stopped_at": None,
    "error": None,
}
_auto_cycle_task: asyncio.Task | None = None


def _auth(x_api_key: str | None) -> None:
    if x_api_key != settings.api_key:
        raise HTTPException(status_code=401, detail="API key inválida")


def _probability_payload(body: WinningProbabilityRequest) -> dict:
    result = get_winning_probability(
        symbol=body.symbol,
        strategy_type=body.strategy_type,
        account_scope=body.account_scope,
        tradier_token=body.tradier_token,
        tradier_base_url=body.tradier_base_url,
        history_days=body.history_days,
        min_dte=body.min_dte,
        max_dte=body.max_dte,
        n_paths=body.n_paths,
        random_seed=body.random_seed,
    )
    payload = result.to_dict()
    payload["supported_strategies"] = list(SUPPORTED_STRATEGIES)
    return payload


def _resolve_request_session(
    *,
    account_scope: str | None,
    account_id: str | None,
    tradier_token: str | None = None,
    tradier_base_url: str | None = None,
) -> dict | None:
    try:
        _, session = _ACCOUNT_MANAGER.resolve(
            account_scope=account_scope,  # type: ignore[arg-type]
            account_id=account_id,
            tradier_token=tradier_token,
            tradier_base_url=tradier_base_url,
        )
        return session.to_dict()
    except Exception:
        logger.exception("Unable to resolve Tradier account session")
        return None


def _is_opening_request(body: OrderRequest) -> bool:
    return tradier_is_opening_order(body)


def _inherit_probability_context(
    body: WinningProbabilityRequest | None,
    *,
    account_scope: str | None,
    account_id: str | None,
) -> WinningProbabilityRequest | None:
    if body is None:
        return None
    updates = {}
    if body.account_scope is None and account_scope is not None:
        updates["account_scope"] = account_scope
    if body.account_id is None and account_id is not None:
        updates["account_id"] = account_id
    return body.model_copy(update=updates) if updates else body


def _active_strategy_ids() -> list[str]:
    return [k for k, v in _strategies.items() if getattr(v, "active", False)]


def _canonical_snapshot_payload(
    *,
    account_scope: str | None,
    account_id: str | None,
) -> dict[str, object]:
    return _CANONICAL_SNAPSHOT.build_snapshot(
        account_scope=account_scope,
        account_id=account_id,
        internal_portfolio=_portfolio,
    )


def _broker_open_positions_count(
    *,
    account_scope: str | None,
    account_id: str | None,
) -> int:
    scope = str(account_scope or "").strip().lower()
    if scope not in {"paper", "live"}:
        return len(_portfolio.positions) if _portfolio else 0
    snapshot = _canonical_snapshot_payload(account_scope=account_scope, account_id=account_id)
    return int((snapshot.get("totals") or {}).get("positions") or 0)


def _tradier_positions_payload(
    *,
    account_scope: str | None,
    account_id: str | None,
) -> dict[str, object]:
    return _CANONICAL_SNAPSHOT.build_positions_payload(
        account_scope=account_scope,
        account_id=account_id,
        internal_portfolio=_portfolio,
    )


def _build_status_payload(
    *,
    account_scope: str | None,
    account_id: str | None,
) -> QuantStatusPayload:
    account_status = _ACCOUNT_MANAGER.status(
        account_scope=account_scope,  # type: ignore[arg-type]
        account_id=account_id,
    )
    payload = _CANONICAL_SNAPSHOT.build_status_payload(
        account_scope=account_scope,
        account_id=account_id,
        uptime_sec=round(time.time() - _START_TIME, 1),
        active_strategies=_active_strategy_ids(),
        internal_portfolio=_portfolio,
        pdt_status=account_status.pdt_status,
    )
    if payload.get("account_session") is None:
        payload["account_session"] = account_status.session.to_dict()
    return QuantStatusPayload.model_validate(payload)


def _compact_monitor_summary(summary: dict[str, object]) -> dict[str, object]:
    strategies = []
    for item in list(summary.get("strategies") or [])[:8]:
        if not isinstance(item, dict):
            continue
        strategies.append(
            {
                "strategy_id": item.get("strategy_id"),
                "underlying": item.get("underlying"),
                "strategy_type": item.get("strategy_type"),
                "open_pnl": item.get("open_pnl"),
                "spot": item.get("spot"),
                "win_rate_pct": item.get("win_rate_pct"),
                "probability_updated_at": item.get("probability_updated_at"),
                "net_delta": item.get("net_delta"),
                "theta_daily": item.get("theta_daily"),
                "positions": item.get("positions"),
                "alert": item.get("alert"),
                "payoff_curve": list(item.get("payoff_curve") or [])[::2],
                "attribution": item.get("attribution") or {},
            }
        )
    return {
        "generated_at": summary.get("generated_at"),
        "refresh_interval_sec": summary.get("refresh_interval_sec"),
        "source": summary.get("source"),
        "source_label": summary.get("source_label"),
        "account_session": summary.get("account_session") or {},
        "balances": summary.get("balances") or {},
        "pdt_status": summary.get("pdt_status") or {},
        "totals": summary.get("totals") or {},
        "alerts": list(summary.get("alerts") or [])[:8],
        "strategies": strategies,
        "reconciliation": summary.get("reconciliation") or {},
        "simulators": summary.get("simulators") or {},
    }


def _compact_scanner_report(report: dict[str, object]) -> dict[str, object]:
    candidates = []
    for item in list(report.get("candidates") or report.get("opportunities") or [])[:6]:
        if not isinstance(item, dict):
            continue
        candidates.append(
            {
                "symbol": item.get("symbol"),
                "strategy_type": item.get("strategy_type") or item.get("strategy"),
                "selection_score": item.get("selection_score") or item.get("score"),
                "win_rate_pct": item.get("win_rate_pct") or item.get("win_probability"),
                "var_95": item.get("var_95") or item.get("risk_pct"),
                "direction": item.get("direction") or item.get("signal") or item.get("side"),
                "generated_at": item.get("generated_at") or item.get("timestamp"),
            }
        )
    return {
        "generated_at": report.get("generated_at"),
        "status": report.get("status"),
        "summary": report.get("summary") or {},
        "candidates": candidates,
        "candidate_count": len(candidates),
    }


# ── Endpoints ────────────────────────────────────────────────────────────────

@app.get("/health", response_model=HealthResponse, tags=["Sistema"])
async def health():
    """Estado general del sistema de trading."""
    return HealthResponse(
        status=StatusEnum.OK,
        uptime_sec=round(time.time() - _START_TIME, 1),
        active_strategies=[k for k, v in _strategies.items() if getattr(v, "active", False)],
        open_positions=len(_portfolio.positions) if _portfolio else 0,
        timestamp=datetime.now(),
    )


@app.get("/status", response_model=StdResponse, tags=["Sistema"])
async def status(
    x_api_key: str | None = Header(None),
    account_scope: str | None = None,
    account_id: str | None = None,
):
    """Estado operativo del motor + control PDT de la cuenta activa."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        payload = _build_status_payload(account_scope=account_scope, account_id=account_id)
        return StdResponse(ok=True, data=payload.model_dump(), ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as exc:
        logger.exception("Error building quant status")
        return StdResponse(ok=False, error=str(exc), ms=round((time.perf_counter() - t0) * 1000, 2))


@app.get("/canonical/snapshot", response_model=StdResponse, tags=["Sistema"])
async def canonical_snapshot(
    x_api_key: str | None = Header(None),
    account_scope: str | None = None,
    account_id: str | None = None,
):
    """Snapshot canónico broker-first para UI, WS y observabilidad."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        payload = _canonical_snapshot_payload(account_scope=account_scope, account_id=account_id)
        return StdResponse(ok=True, data=payload, ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as exc:
        logger.exception("Error building canonical snapshot")
        return StdResponse(ok=False, error=str(exc), ms=round((time.perf_counter() - t0) * 1000, 2))


@app.post("/signal", response_model=StdResponse, tags=["Trading"])
async def eval_signal(body: EvalSignalRequest, x_api_key: str | None = Header(None)):
    """Evalúa señales para un símbolo. Llamado por Atlas/ROS2."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    strategy_id = body.strategy or next(
        (k for k, v in _strategies.items() if getattr(v, "active", False)), None
    )
    if not strategy_id or strategy_id not in _strategies:
        return StdResponse(ok=False, error="No hay estrategia activa", ms=0)
    strategy = _strategies[strategy_id]
    try:
        from data.feed import MarketFeed
        feed = MarketFeed(source="ccxt")
        df = feed.ohlcv(body.symbol, body.timeframe, limit=200)
        signal = strategy.generate_signal(df, body.symbol)
        strategy.on_signal(signal)
        signal_data = signal.to_dict()
        if body.options_probability:
            probability_request = _inherit_probability_context(
                body.options_probability,
                account_scope=body.options_probability.account_scope,
                account_id=body.options_probability.account_id,
            ) or body.options_probability
            signal_data.setdefault("metadata", {})
            signal_data["metadata"]["options_probability"] = _probability_payload(probability_request)
            signal_data["metadata"]["account_session"] = _resolve_request_session(
                account_scope=probability_request.account_scope,
                account_id=probability_request.account_id,
                tradier_token=probability_request.tradier_token,
                tradier_base_url=probability_request.tradier_base_url,
            )
        return StdResponse(
            ok=True,
            data=signal_data,
            ms=round((time.perf_counter() - t0) * 1000, 2),
        )
    except Exception as e:
        logger.exception("Error evaluando señal")
        return StdResponse(ok=False, error=str(e))


@app.post("/probability/options", response_model=StdResponse, tags=["Opciones"])
async def probability_options(body: WinningProbabilityRequest, x_api_key: str | None = Header(None)):
    """Calcula probabilidad de victoria para una estrategia de opciones."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        payload = _probability_payload(body)
        payload["account_session"] = _resolve_request_session(
            account_scope=body.account_scope,
            account_id=body.account_id,
            tradier_token=body.tradier_token,
            tradier_base_url=body.tradier_base_url,
        )
        return StdResponse(
            ok=True,
            data=payload,
            ms=round((time.perf_counter() - t0) * 1000, 2),
        )
    except Exception as e:
        logger.exception("Error calculando winning probability")
        return StdResponse(
            ok=False,
            error=str(e),
            data={
                "symbol": body.symbol,
                "strategy_type": body.strategy_type,
                "supported_strategies": list(SUPPORTED_STRATEGIES),
            },
            ms=round((time.perf_counter() - t0) * 1000, 2),
        )


@app.post("/order", response_model=StdResponse, tags=["Trading"])
@require_live_confirmation
async def place_order(body: OrderRequest, x_api_key: str | None = Header(None)):
    """Coloca una orden de trading (paper o live segun configuracion)."""
    _auth(x_api_key)
    opening_request = _is_opening_request(body)
    effective_account_scope = body.account_scope or settings.tradier_default_scope
    tradier_route = should_route_to_tradier(body)
    account_session_payload = (
        _resolve_request_session(
            account_scope=effective_account_scope,
            account_id=body.account_id,
        )
        if tradier_route or body.account_scope is not None or body.account_id is not None
        else None
    )
    pdt_status = None
    gate_payload = None

    if opening_request and body.probability_gate and body.probability_gate.enabled:
        gate_request = _inherit_probability_context(
            body.probability_gate,
            account_scope=effective_account_scope,
            account_id=body.account_id,
        ) or body.probability_gate
        gate_payload = _probability_payload(gate_request)
        win_rate_pct = float(gate_payload.get("win_rate_pct") or 0.0)
        min_win_rate_pct = float(gate_request.min_win_rate_pct)
        if win_rate_pct < min_win_rate_pct:
            return StdResponse(
                ok=False,
                error=(
                    f"Probability gate bloqueo la orden: {win_rate_pct:.2f}% < "
                    f"minimo requerido {min_win_rate_pct:.2f}%"
                ),
                data={
                    "probability_gate": gate_payload,
                    "account_session": account_session_payload,
                    "pdt_status": pdt_status,
                },
            )

    if tradier_route:
        try:
            payload = route_order_to_tradier(body)
            if gate_payload is not None:
                payload["probability_gate"] = gate_payload
            return StdResponse(ok=True, data=payload)
        except TradierOrderBlocked as exc:
            payload = {
                "account_session": account_session_payload,
                **exc.payload,
                **({"probability_gate": gate_payload} if gate_payload is not None else {}),
            }
            return StdResponse(ok=False, error=str(exc), data=payload)
        except Exception as exc:
            logger.exception("Error routing order to Tradier")
            payload = {
                "account_session": account_session_payload,
                **({"probability_gate": gate_payload} if gate_payload is not None else {}),
            }
            return StdResponse(ok=False, error=str(exc), data=payload)

    if not settings.paper_trading:
        raise HTTPException(status_code=403, detail="Live trading no habilitado")
    if not _portfolio:
        return StdResponse(ok=False, error="Portfolio no inicializado")

    if opening_request:
        pos = _portfolio.open_position(
            body.symbol, "long", body.size, body.price or 0,
            body.stop_loss, body.take_profit,
        )
        payload = pos.to_dict() if pos else None
        if payload is not None:
            payload = {
                **payload,
                "account_session": account_session_payload,
                "pdt_status": pdt_status,
                **({"probability_gate": gate_payload} if gate_payload is not None else {}),
            }
        return StdResponse(ok=pos is not None, data=payload)
    pnl = _portfolio.close_position(body.symbol, body.price or 0)
    return StdResponse(ok=True, data={"pnl": pnl, "account_session": account_session_payload, "pdt_status": pdt_status})


@app.get("/positions", response_model=StdResponse, tags=["Trading"])
async def get_positions(
    x_api_key: str | None = Header(None),
    account_scope: str | None = None,
    account_id: str | None = None,
):
    """Retorna posiciones abiertas y resumen del portfolio."""
    _auth(x_api_key)
    scope = str(account_scope or "").strip().lower()
    if scope in {"paper", "live"}:
        try:
            return StdResponse(ok=True, data=_tradier_positions_payload(account_scope=scope, account_id=account_id))
        except Exception as exc:
            logger.exception("Error building Tradier positions payload")
            return StdResponse(ok=False, error=str(exc))
    if not _portfolio:
        return StdResponse(ok=False, error="Portfolio no inicializado")
    return StdResponse(ok=True, data=_portfolio.summary())


@app.get("/monitor/summary", response_model=StdResponse, tags=["Monitor"])
async def monitor_summary(
    x_api_key: str | None = Header(None),
    account_scope: str | None = None,
    account_id: str | None = None,
):
    """Entrega seguimiento avanzado numerico y grafico para posiciones Tradier."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        snapshot = _canonical_snapshot_payload(account_scope=account_scope, account_id=account_id)
        data = dict(snapshot.get("monitor_summary") or {})
        data["source"] = snapshot.get("source")
        data["source_label"] = snapshot.get("source_label")
        data["reconciliation"] = snapshot.get("reconciliation")
        data["simulators"] = snapshot.get("simulators")
        return StdResponse(ok=True, data=data, ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as e:
        logger.exception("Error building advanced monitor summary")
        return StdResponse(ok=False, error=str(e), ms=round((time.perf_counter() - t0) * 1000, 2))


@app.get("/monitor/payoff/{strategy_id}", response_model=StdResponse, tags=["Monitor"])
async def monitor_payoff(
    strategy_id: str,
    x_api_key: str | None = Header(None),
    account_scope: str | None = None,
    account_id: str | None = None,
):
    """Curva de riesgo detallada para una estrategia activa."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        payload = _STRATEGY_TRACKER.payoff(
            strategy_id,
            account_scope=account_scope,  # type: ignore[arg-type]
            account_id=account_id,
        )
        return StdResponse(ok=True, data=payload.model_dump(), ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as exc:
        logger.exception("Error building payoff curve")
        return StdResponse(ok=False, error=str(exc), ms=round((time.perf_counter() - t0) * 1000, 2))


@app.get("/journal/stats", response_model=StdResponse, tags=["Journal"])
async def journal_stats(x_api_key: str | None = Header(None)):
    """Resumen comparativo del diario Live vs Paper."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        payload = JournalStatsPayload.model_validate(_JOURNAL.stats())
        return StdResponse(ok=True, data=payload.model_dump(), ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as exc:
        logger.exception("Error building journal stats")
        return StdResponse(ok=False, error=str(exc), ms=round((time.perf_counter() - t0) * 1000, 2))


@app.get("/journal/entries", response_model=StdResponse, tags=["Journal"])
async def journal_entries(
    x_api_key: str | None = Header(None),
    limit: int = 24,
    status: str | None = None,
):
    """Entradas recientes del diario con tesis, patas y post-mortem."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        payload = JournalEntriesPayload.model_validate(_JOURNAL.entries(limit=limit, status=status))
        return StdResponse(ok=True, data=payload.model_dump(), ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as exc:
        logger.exception("Error building journal entries")
        return StdResponse(ok=False, error=str(exc), ms=round((time.perf_counter() - t0) * 1000, 2))


@app.post("/journal/sync/refresh", response_model=StdResponse, tags=["Journal"])
async def journal_refresh(x_api_key: str | None = Header(None)):
    """Ejecuta una sincronización inmediata del diario contra Tradier."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        payload = await _JOURNAL_SYNC.run_once()
        return StdResponse(ok=True, data=payload, ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as exc:
        logger.exception("Error running journal refresh")
        return StdResponse(ok=False, error=str(exc), ms=round((time.perf_counter() - t0) * 1000, 2))


@app.get("/operation/status", response_model=StdResponse, tags=["Operation"])
async def operation_status(x_api_key: str | None = Header(None)):
    """Unified operational status for paper-first ATLAS tests."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        payload = OperationStatusPayload.model_validate(_OPERATION_CENTER.status())
        return StdResponse(ok=True, data=payload.model_dump(), ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as exc:
        logger.exception("Error building operation status")
        return StdResponse(ok=False, error=str(exc), ms=round((time.perf_counter() - t0) * 1000, 2))


@app.post("/operation/config", response_model=StdResponse, tags=["Operation"])
async def operation_config(
    body: OperationConfigPayload,
    x_api_key: str | None = Header(None),
):
    """Update paper-first operational control-plane config."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        _OPERATION_CENTER.update_config(body.model_dump())
        payload = OperationStatusPayload.model_validate(_OPERATION_CENTER.status())
        return StdResponse(ok=True, data=payload.model_dump(), ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as exc:
        logger.exception("Error updating operation config")
        return StdResponse(ok=False, error=str(exc), ms=round((time.perf_counter() - t0) * 1000, 2))


@app.post("/operation/test-cycle", response_model=StdResponse, tags=["Operation"])
async def operation_test_cycle(
    body: OperationCycleRequest,
    x_api_key: str | None = Header(None),
):
    """Evaluate, preview or submit a paper-first autonomous cycle."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        payload = OperationCyclePayload.model_validate(
            _OPERATION_CENTER.evaluate_candidate(
                order=body.order,
                action=body.action,
                capture_context=body.capture_context,
            )
        )
        return StdResponse(ok=True, data=payload.model_dump(), ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as exc:
        logger.exception("Error running operation cycle")
        return StdResponse(ok=False, error=str(exc), ms=round((time.perf_counter() - t0) * 1000, 2))


async def _auto_cycle_loop(interval_sec: int, max_per_cycle: int) -> None:
    """Loop background: toma candidatos del scanner y los evalúa en OperationCenter."""
    global _AUTO_CYCLE_STATE
    _AUTO_CYCLE_STATE.update({
        "running": True, "cycle_count": 0, "error": None,
        "started_at": datetime.utcnow().isoformat(), "stopped_at": None,
        "loop_interval_sec": interval_sec, "max_per_cycle": max_per_cycle,
    })
    logger.info("[auto-cycle] Iniciado. interval=%ds max_per_cycle=%d", interval_sec, max_per_cycle)
    try:
        while _AUTO_CYCLE_STATE["running"]:
            await asyncio.sleep(interval_sec)
            if not _AUTO_CYCLE_STATE["running"]:
                break
            try:
                report = await asyncio.to_thread(_SCANNER.report, 24)
                candidates = list(report.get("candidates") or [])
                if not candidates:
                    _AUTO_CYCLE_STATE["last_cycle_at"] = datetime.utcnow().isoformat()
                    _AUTO_CYCLE_STATE["cycle_count"] += 1
                    logger.debug("[auto-cycle] Ciclo %d — sin candidatos", _AUTO_CYCLE_STATE["cycle_count"])
                    continue

                sorted_cands = sorted(
                    candidates,
                    key=lambda c: float(c.get("selection_score") or 0),
                    reverse=True,
                )[:max_per_cycle]

                op_status = await asyncio.to_thread(_OPERATION_CENTER.status)
                auton_mode = str(op_status.get("auton_mode") or "paper_supervised")
                action = "submit" if auton_mode not in {"off", "paper_supervised"} else "preview"

                last_result = None
                for cand in sorted_cands:
                    symbol = str(cand.get("symbol") or "").strip()
                    if not symbol:
                        continue
                    direction = str(cand.get("direction") or "long").lower()
                    side = "buy" if direction in {"long", "bull", "up"} else "sell_short"
                    ac_raw = str(cand.get("asset_class") or "equity").lower()
                    asset_class = ac_raw if ac_raw in {"equity", "option", "multileg", "combo"} else "equity"
                    order = OrderRequest(
                        symbol=symbol, side=side, size=1,
                        order_type="market", asset_class=asset_class,  # type: ignore[arg-type]
                        account_scope="paper", preview=True,
                    )
                    result = await asyncio.to_thread(
                        _OPERATION_CENTER.evaluate_candidate,
                        order=order, action=action, capture_context=False,  # type: ignore[call-arg]
                    )
                    last_result = {
                        "symbol": symbol, "action": action,
                        "blocked": bool(result.get("blocked")),
                        "reasons": (result.get("reasons") or []),
                        "selection_score": cand.get("selection_score"),
                        "win_rate_pct": (result.get("probability") or {}).get("win_rate_pct"),
                    }
                    logger.info("[auto-cycle] symbol=%s action=%s blocked=%s score=%.1f",
                                symbol, action, last_result["blocked"],
                                float(cand.get("selection_score") or 0))

                _AUTO_CYCLE_STATE.update({
                    "cycle_count": _AUTO_CYCLE_STATE["cycle_count"] + 1,
                    "last_cycle_at": datetime.utcnow().isoformat(),
                    "last_candidate_symbol": sorted_cands[0].get("symbol") if sorted_cands else None,
                    "last_action": action,
                    "last_result": last_result,
                })
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                _AUTO_CYCLE_STATE["error"] = str(exc)
                _AUTO_CYCLE_STATE["last_cycle_at"] = datetime.utcnow().isoformat()
                logger.exception("[auto-cycle] Error en ciclo: %s", exc)
    except asyncio.CancelledError:
        pass
    finally:
        _AUTO_CYCLE_STATE.update({"running": False, "stopped_at": datetime.utcnow().isoformat()})
        logger.info("[auto-cycle] Detenido. ciclos=%d", _AUTO_CYCLE_STATE["cycle_count"])


@app.post("/operation/loop/start", response_model=StdResponse, tags=["Operation"])
async def operation_loop_start(
    body: LoopStartRequest,
    x_api_key: str | None = Header(None),
):
    """Inicia el ciclo autónomo: toma candidatos del scanner y los envía a OperationCenter.
    En modo paper_supervised ejecuta preview (evalúa sin enviar orden).
    En modo paper_autonomous ejecuta submit (envía a paper trading).
    """
    _auth(x_api_key)
    global _auto_cycle_task
    t0 = time.perf_counter()
    if _AUTO_CYCLE_STATE.get("running") and _auto_cycle_task and not _auto_cycle_task.done():
        return StdResponse(
            ok=False, error="auto_cycle_already_running",
            data=dict(_AUTO_CYCLE_STATE),
            ms=round((time.perf_counter() - t0) * 1000, 2),
        )
    if _auto_cycle_task and not _auto_cycle_task.done():
        _auto_cycle_task.cancel()
    _auto_cycle_task = asyncio.create_task(
        _auto_cycle_loop(body.interval_sec, body.max_per_cycle)
    )
    return StdResponse(
        ok=True,
        data={"started": True, "interval_sec": body.interval_sec, "max_per_cycle": body.max_per_cycle},
        ms=round((time.perf_counter() - t0) * 1000, 2),
    )


@app.post("/operation/loop/stop", response_model=StdResponse, tags=["Operation"])
async def operation_loop_stop(x_api_key: str | None = Header(None)):
    """Detiene el ciclo autónomo de evaluación."""
    _auth(x_api_key)
    global _auto_cycle_task
    t0 = time.perf_counter()
    _AUTO_CYCLE_STATE["running"] = False
    if _auto_cycle_task and not _auto_cycle_task.done():
        _auto_cycle_task.cancel()
        try:
            await asyncio.wait_for(asyncio.shield(_auto_cycle_task), timeout=3.0)
        except (asyncio.CancelledError, asyncio.TimeoutError, Exception):
            pass
    return StdResponse(
        ok=True,
        data={"stopped": True, "cycles_completed": _AUTO_CYCLE_STATE.get("cycle_count", 0)},
        ms=round((time.perf_counter() - t0) * 1000, 2),
    )


@app.get("/operation/loop/status", response_model=StdResponse, tags=["Operation"])
async def operation_loop_status(x_api_key: str | None = Header(None)):
    """Estado actual del ciclo autónomo scanner→operación."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    state = dict(_AUTO_CYCLE_STATE)
    state["task_alive"] = bool(_auto_cycle_task and not _auto_cycle_task.done())
    return StdResponse(ok=True, data=state, ms=round((time.perf_counter() - t0) * 1000, 2))


# ── Vision management endpoints ───────────────────────────────────────────────

@app.get("/operation/vision", response_model=StdResponse, tags=["Operation"])
async def operation_vision_status_endpoint(x_api_key: str | None = Header(None)):
    """Estado completo del proveedor de visión incluyendo diagnóstico de conectividad."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        status = await asyncio.to_thread(_VISION.status)
        diagnose = await asyncio.to_thread(_VISION.diagnose)
        return StdResponse(
            ok=True,
            data={"status": status, "diagnose": diagnose},
            ms=round((time.perf_counter() - t0) * 1000, 2),
        )
    except Exception as exc:
        logger.exception("Error en vision status")
        return StdResponse(ok=False, error=str(exc), ms=round((time.perf_counter() - t0) * 1000, 2))


@app.post("/operation/vision/provider", response_model=StdResponse, tags=["Operation"])
async def operation_vision_provider(
    body: VisionProviderRequest,
    x_api_key: str | None = Header(None),
):
    """Cambia el proveedor de visión activo (ej: direct_nexus → desktop_capture → off)."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    supported = {"off", "manual", "desktop_capture", "direct_nexus", "atlas_push_bridge", "insta360"}
    if body.provider not in supported:
        return StdResponse(
            ok=False,
            error=f"Proveedor no soportado. Opciones: {sorted(supported)}",
            ms=round((time.perf_counter() - t0) * 1000, 2),
        )
    try:
        updated = await asyncio.to_thread(
            _VISION.update,
            provider=body.provider,
            notes=body.notes,
        )
        diagnose = await asyncio.to_thread(_VISION.diagnose)
        return StdResponse(
            ok=True,
            data={"updated": updated, "diagnose": diagnose},
            ms=round((time.perf_counter() - t0) * 1000, 2),
        )
    except Exception as exc:
        logger.exception("Error cambiando proveedor de vision")
        return StdResponse(ok=False, error=str(exc), ms=round((time.perf_counter() - t0) * 1000, 2))


# ── Vision chart analyze (Insta360 / desktop OCR) ─────────────────────────────

@app.get("/vision/chart/analyze", response_model=StdResponse, tags=["Operation"])
async def vision_chart_analyze(
    max_age_sec: float = 30.0,
    x_api_key: str | None = Header(None),
):
    """Captura un frame (Insta360/desktop) y ejecuta OCR sobre el grafico.

    Devuelve: chart_color, prices detectados, patron, confidence y fuente.
    """
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        from atlas_code_quant.vision.visual_pipeline import VisualPipeline
        pipeline = await asyncio.to_thread(VisualPipeline.get_instance)
        result = await asyncio.to_thread(pipeline.analyze, max_age_sec)
        return StdResponse(
            ok=result.error is None,
            data={
                "chart_color": result.chart_color,
                "prices": result.prices[:10],
                "pattern_detected": result.pattern_detected,
                "confidence": result.confidence,
                "source": result.source,
                "raw_texts_sample": result.raw_texts[:5] if result.raw_texts else [],
                "error": result.error,
            },
            ms=round((time.perf_counter() - t0) * 1000, 2),
        )
    except Exception as exc:
        logger.exception("Error en vision chart analyze")
        return StdResponse(ok=False, error=str(exc), ms=round((time.perf_counter() - t0) * 1000, 2))


@app.get("/vision/chart/status", response_model=StdResponse, tags=["Operation"])
async def vision_chart_status(x_api_key: str | None = Header(None)):
    """Estado del pipeline visual: fuente activa, OCR disponible, ultimo resultado."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        from atlas_code_quant.vision.visual_pipeline import VisualPipeline
        pipeline = await asyncio.to_thread(VisualPipeline.get_instance)
        status = await asyncio.to_thread(pipeline.status)
        return StdResponse(ok=True, data=status, ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as exc:
        return StdResponse(ok=False, error=str(exc), ms=round((time.perf_counter() - t0) * 1000, 2))


@app.get("/api/v2/quant/vision/chart/analyze", response_model=StdResponse, tags=["V2"])
async def vision_chart_analyze_v2(max_age_sec: float = 30.0, x_api_key: str | None = Header(None)):
    return await vision_chart_analyze(max_age_sec=max_age_sec, x_api_key=x_api_key)


@app.get("/api/v2/quant/vision/chart/status", response_model=StdResponse, tags=["V2"])
async def vision_chart_status_v2(x_api_key: str | None = Header(None)):
    return await vision_chart_status(x_api_key=x_api_key)


# ── V2 aliases — loop & vision ────────────────────────────────────────────────

@app.post("/api/v2/quant/operation/loop/start", response_model=StdResponse, tags=["V2"])
async def operation_loop_start_v2(body: LoopStartRequest, x_api_key: str | None = Header(None)):
    return await operation_loop_start(body=body, x_api_key=x_api_key)


@app.post("/api/v2/quant/operation/loop/stop", response_model=StdResponse, tags=["V2"])
async def operation_loop_stop_v2(x_api_key: str | None = Header(None)):
    return await operation_loop_stop(x_api_key=x_api_key)


@app.get("/api/v2/quant/operation/loop/status", response_model=StdResponse, tags=["V2"])
async def operation_loop_status_v2(x_api_key: str | None = Header(None)):
    return await operation_loop_status(x_api_key=x_api_key)


@app.get("/api/v2/quant/operation/vision", response_model=StdResponse, tags=["V2"])
async def operation_vision_v2(x_api_key: str | None = Header(None)):
    return await operation_vision_status_endpoint(x_api_key=x_api_key)


@app.post("/api/v2/quant/operation/vision/provider", response_model=StdResponse, tags=["V2"])
async def operation_vision_provider_v2(body: VisionProviderRequest, x_api_key: str | None = Header(None)):
    return await operation_vision_provider(body=body, x_api_key=x_api_key)


@app.get("/vision/calibration/status", response_model=StdResponse, tags=["Operation"])
async def vision_calibration_status(x_api_key: str | None = Header(None)):
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        payload = VisionCalibrationStatusPayload.model_validate(_VISION_CALIBRATION.status())
        return StdResponse(ok=True, data=payload.model_dump(), ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as exc:
        logger.exception("Error reading vision calibration status")
        return StdResponse(ok=False, error=str(exc), ms=round((time.perf_counter() - t0) * 1000, 2))


@app.post("/vision/calibration/start", response_model=StdResponse, tags=["Operation"])
async def vision_calibration_start(
    body: VisionCalibrationStartPayload,
    x_api_key: str | None = Header(None),
):
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        payload = VisionCalibrationStatusPayload.model_validate(_VISION_CALIBRATION.start_session(**body.model_dump()))
        return StdResponse(ok=True, data=payload.model_dump(), ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as exc:
        logger.exception("Error starting vision calibration")
        return StdResponse(ok=False, error=str(exc), ms=round((time.perf_counter() - t0) * 1000, 2))


@app.post("/vision/calibration/move-pose", response_model=StdResponse, tags=["Operation"])
async def vision_calibration_move_pose(
    body: VisionCalibrationMovePosePayload,
    x_api_key: str | None = Header(None),
):
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        payload = VisionCalibrationStatusPayload.model_validate(_VISION_CALIBRATION.move_pose(**body.model_dump()))
        return StdResponse(ok=True, data=payload.model_dump(), ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as exc:
        logger.exception("Error moving vision pose")
        return StdResponse(ok=False, error=str(exc), ms=round((time.perf_counter() - t0) * 1000, 2))


@app.post("/vision/calibration/sample", response_model=StdResponse, tags=["Operation"])
async def vision_calibration_sample(
    body: VisionCalibrationSamplePayload,
    x_api_key: str | None = Header(None),
):
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        payload = VisionCalibrationStatusPayload.model_validate(_VISION_CALIBRATION.record_sample(**body.model_dump()))
        return StdResponse(ok=True, data=payload.model_dump(), ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as exc:
        logger.exception("Error recording vision calibration sample")
        return StdResponse(ok=False, error=str(exc), ms=round((time.perf_counter() - t0) * 1000, 2))


@app.post("/vision/calibration/fit", response_model=StdResponse, tags=["Operation"])
async def vision_calibration_fit(
    body: VisionCalibrationStartPayload | None = None,
    x_api_key: str | None = Header(None),
):
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        save_path = body.save_path if body else None
        payload = VisionCalibrationStatusPayload.model_validate(_VISION_CALIBRATION.fit_and_save(save_path=save_path))
        return StdResponse(ok=True, data=payload.model_dump(), ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as exc:
        logger.exception("Error fitting vision calibration")
        return StdResponse(ok=False, error=str(exc), ms=round((time.perf_counter() - t0) * 1000, 2))


@app.post("/vision/calibration/reset", response_model=StdResponse, tags=["Operation"])
async def vision_calibration_reset(
    body: VisionCalibrationResetPayload,
    x_api_key: str | None = Header(None),
):
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        payload = VisionCalibrationStatusPayload.model_validate(_VISION_CALIBRATION.reset_session(**body.model_dump()))
        return StdResponse(ok=True, data=payload.model_dump(), ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as exc:
        logger.exception("Error resetting vision calibration")
        return StdResponse(ok=False, error=str(exc), ms=round((time.perf_counter() - t0) * 1000, 2))


@app.post("/emergency/stop", response_model=StdResponse, tags=["Operation"])
async def emergency_stop(
    body: EmergencyStopRequest,
    x_api_key: str | None = Header(None),
):
    """Immediate kill switch for autonomous execution."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        payload = OperationStatusPayload.model_validate(_OPERATION_CENTER.emergency_stop(reason=body.reason))
        return StdResponse(ok=True, data=payload.model_dump(), ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as exc:
        logger.exception("Error triggering emergency stop")
        return StdResponse(ok=False, error=str(exc), ms=round((time.perf_counter() - t0) * 1000, 2))


@app.post("/emergency/reset", response_model=StdResponse, tags=["Operation"])
async def emergency_reset(x_api_key: str | None = Header(None)):
    """Reset the autonomous execution kill switch."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        payload = OperationStatusPayload.model_validate(_OPERATION_CENTER.clear_emergency_stop())
        return StdResponse(ok=True, data=payload.model_dump(), ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as exc:
        logger.exception("Error resetting emergency stop")
        return StdResponse(ok=False, error=str(exc), ms=round((time.perf_counter() - t0) * 1000, 2))


@app.get("/scanner/status", response_model=StdResponse, tags=["Scanner"])
async def scanner_status(x_api_key: str | None = Header(None)):
    """Estado operativo del escáner permanente de oportunidades."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        payload = ScannerStatusPayload.model_validate(_SCANNER.status())
        return StdResponse(ok=True, data=payload.model_dump(), ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as exc:
        logger.exception("Error reading scanner status")
        return StdResponse(ok=False, error=str(exc), ms=round((time.perf_counter() - t0) * 1000, 2))


@app.get("/scanner/report", response_model=StdResponse, tags=["Scanner"])
async def scanner_report(
    x_api_key: str | None = Header(None),
    activity_limit: int = 60,
):
    """Reporte completo del escáner: criterios, candidatos, rechazos y actividad."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        payload = ScannerReportPayload.model_validate(_SCANNER.report(activity_limit=activity_limit))
        return StdResponse(ok=True, data=payload.model_dump(), ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as exc:
        logger.exception("Error reading scanner report")
        return StdResponse(ok=False, error=str(exc), ms=round((time.perf_counter() - t0) * 1000, 2))


@app.post("/scanner/config", response_model=StdResponse, tags=["Scanner"])
async def scanner_config(
    body: ScannerConfigPayload,
    x_api_key: str | None = Header(None),
):
    """Actualiza la configuración del escáner."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        _SCANNER.update_config(body.model_dump(exclude_none=True))
        if body.enabled:
            await _SCANNER.start()
        else:
            await _SCANNER.stop()
        payload = ScannerReportPayload.model_validate(_SCANNER.report())
        return StdResponse(ok=True, data=payload.model_dump(), ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as exc:
        logger.exception("Error updating scanner config")
        return StdResponse(ok=False, error=str(exc), ms=round((time.perf_counter() - t0) * 1000, 2))


@app.post("/scanner/control", response_model=StdResponse, tags=["Scanner"])
async def scanner_control(
    body: ScannerControlRequest,
    x_api_key: str | None = Header(None),
):
    """Inicia, detiene o fuerza un ciclo del escáner."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        payload = ScannerReportPayload.model_validate(await _SCANNER.control(body.action))
        return StdResponse(ok=True, data=payload.model_dump(), ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as exc:
        logger.exception("Error controlling scanner")
        return StdResponse(ok=False, error=str(exc), ms=round((time.perf_counter() - t0) * 1000, 2))


@app.post("/selector/proposal", response_model=StdResponse, tags=["Selector"])
async def selector_proposal(
    body: StrategySelectorRequest,
    x_api_key: str | None = Header(None),
):
    """Convierte una oportunidad del escaner en una propuesta operativa lista para validar."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        payload = StrategySelectorPayload.model_validate(
            _SELECTOR.proposal(
                candidate=body.candidate.model_dump(),
                account_scope=body.account_scope,
                account_id=body.account_id,
                chart_provider=body.chart_provider,
                prefer_defined_risk=body.prefer_defined_risk,
                allow_equity=body.allow_equity,
                allow_credit=body.allow_credit,
                risk_budget_pct=body.risk_budget_pct,
            )
        )
        await asyncio.to_thread(_BRAIN_BRIDGE.record_selector_proposal, payload.model_dump())
        return StdResponse(ok=True, data=payload.model_dump(), ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as exc:
        logger.exception("Error building selector proposal")
        await asyncio.to_thread(
            _BRAIN_BRIDGE.record_error,
            kind="selector_error",
            source="selector_proposal",
            error_text=str(exc),
            context={"candidate": body.candidate.model_dump(), "account_scope": body.account_scope},
        )
        return StdResponse(ok=False, error=str(exc), ms=round((time.perf_counter() - t0) * 1000, 2))


@app.get("/api/v2/quant/health", response_model=HealthResponse, tags=["V2"])
async def health_v2():
    return await health()


@app.get("/api/v2/quant/status", response_model=StdResponse, tags=["V2"])
async def status_v2(
    x_api_key: str | None = Header(None),
    account_scope: str | None = None,
    account_id: str | None = None,
):
    return await status(x_api_key=x_api_key, account_scope=account_scope, account_id=account_id)


@app.post("/api/v2/quant/signal", response_model=StdResponse, tags=["V2"])
async def eval_signal_v2(body: EvalSignalRequest, x_api_key: str | None = Header(None)):
    return await eval_signal(body=body, x_api_key=x_api_key)


@app.post("/api/v2/quant/probability/options", response_model=StdResponse, tags=["V2"])
async def probability_options_v2(body: WinningProbabilityRequest, x_api_key: str | None = Header(None)):
    return await probability_options(body=body, x_api_key=x_api_key)


@app.post("/api/v2/quant/order", response_model=StdResponse, tags=["V2"])
async def place_order_v2(body: OrderRequest, x_api_key: str | None = Header(None)):
    return await place_order(body=body, x_api_key=x_api_key)


@app.get("/api/v2/quant/positions", response_model=StdResponse, tags=["V2"])
async def get_positions_v2(
    x_api_key: str | None = Header(None),
    account_scope: str | None = None,
    account_id: str | None = None,
):
    return await get_positions(x_api_key=x_api_key, account_scope=account_scope, account_id=account_id)


@app.get("/api/v2/quant/canonical/snapshot", response_model=StdResponse, tags=["V2"])
async def canonical_snapshot_v2(
    x_api_key: str | None = Header(None),
    account_scope: str | None = None,
    account_id: str | None = None,
):
    return await canonical_snapshot(x_api_key=x_api_key, account_scope=account_scope, account_id=account_id)


@app.get("/api/v2/quant/monitor/summary", response_model=StdResponse, tags=["V2"])
async def monitor_summary_v2(
    x_api_key: str | None = Header(None),
    account_scope: str | None = None,
    account_id: str | None = None,
):
    return await monitor_summary(x_api_key=x_api_key, account_scope=account_scope, account_id=account_id)


@app.get("/api/v2/quant/monitor/payoff/{strategy_id}", response_model=StdResponse, tags=["V2"])
async def monitor_payoff_v2(
    strategy_id: str,
    x_api_key: str | None = Header(None),
    account_scope: str | None = None,
    account_id: str | None = None,
):
    return await monitor_payoff(
        strategy_id=strategy_id,
        x_api_key=x_api_key,
        account_scope=account_scope,
        account_id=account_id,
    )


@app.get("/api/v2/quant/journal/stats", response_model=StdResponse, tags=["V2"])
async def journal_stats_v2(x_api_key: str | None = Header(None)):
    return await journal_stats(x_api_key=x_api_key)


@app.get("/api/v2/quant/journal/entries", response_model=StdResponse, tags=["V2"])
async def journal_entries_v2(
    x_api_key: str | None = Header(None),
    limit: int = 24,
    status: str | None = None,
):
    return await journal_entries(x_api_key=x_api_key, limit=limit, status=status)


@app.post("/api/v2/quant/journal/sync/refresh", response_model=StdResponse, tags=["V2"])
async def journal_refresh_v2(x_api_key: str | None = Header(None)):
    return await journal_refresh(x_api_key=x_api_key)


@app.get("/api/v2/quant/operation/status", response_model=StdResponse, tags=["V2"])
async def operation_status_v2(x_api_key: str | None = Header(None)):
    return await operation_status(x_api_key=x_api_key)


@app.post("/api/v2/quant/operation/config", response_model=StdResponse, tags=["V2"])
async def operation_config_v2(
    body: OperationConfigPayload,
    x_api_key: str | None = Header(None),
):
    return await operation_config(body=body, x_api_key=x_api_key)


@app.post("/api/v2/quant/operation/test-cycle", response_model=StdResponse, tags=["V2"])
async def operation_test_cycle_v2(
    body: OperationCycleRequest,
    x_api_key: str | None = Header(None),
):
    return await operation_test_cycle(body=body, x_api_key=x_api_key)


@app.get("/api/v2/quant/vision/calibration/status", response_model=StdResponse, tags=["V2"])
async def vision_calibration_status_v2(x_api_key: str | None = Header(None)):
    return await vision_calibration_status(x_api_key=x_api_key)


@app.post("/api/v2/quant/vision/calibration/start", response_model=StdResponse, tags=["V2"])
async def vision_calibration_start_v2(
    body: VisionCalibrationStartPayload,
    x_api_key: str | None = Header(None),
):
    return await vision_calibration_start(body=body, x_api_key=x_api_key)


@app.post("/api/v2/quant/vision/calibration/move-pose", response_model=StdResponse, tags=["V2"])
async def vision_calibration_move_pose_v2(
    body: VisionCalibrationMovePosePayload,
    x_api_key: str | None = Header(None),
):
    return await vision_calibration_move_pose(body=body, x_api_key=x_api_key)


@app.post("/api/v2/quant/vision/calibration/sample", response_model=StdResponse, tags=["V2"])
async def vision_calibration_sample_v2(
    body: VisionCalibrationSamplePayload,
    x_api_key: str | None = Header(None),
):
    return await vision_calibration_sample(body=body, x_api_key=x_api_key)


@app.post("/api/v2/quant/vision/calibration/fit", response_model=StdResponse, tags=["V2"])
async def vision_calibration_fit_v2(
    body: VisionCalibrationStartPayload | None = None,
    x_api_key: str | None = Header(None),
):
    return await vision_calibration_fit(body=body, x_api_key=x_api_key)


@app.post("/api/v2/quant/vision/calibration/reset", response_model=StdResponse, tags=["V2"])
async def vision_calibration_reset_v2(
    body: VisionCalibrationResetPayload,
    x_api_key: str | None = Header(None),
):
    return await vision_calibration_reset(body=body, x_api_key=x_api_key)


@app.post("/api/v2/quant/emergency/stop", response_model=StdResponse, tags=["V2"])
async def emergency_stop_v2(
    body: EmergencyStopRequest,
    x_api_key: str | None = Header(None),
):
    return await emergency_stop(body=body, x_api_key=x_api_key)


@app.post("/api/v2/quant/emergency/reset", response_model=StdResponse, tags=["V2"])
async def emergency_reset_v2(x_api_key: str | None = Header(None)):
    return await emergency_reset(x_api_key=x_api_key)


@app.get("/api/v2/quant/scanner/status", response_model=StdResponse, tags=["V2"])
async def scanner_status_v2(x_api_key: str | None = Header(None)):
    return await scanner_status(x_api_key=x_api_key)


@app.get("/api/v2/quant/scanner/report", response_model=StdResponse, tags=["V2"])
async def scanner_report_v2(
    x_api_key: str | None = Header(None),
    activity_limit: int = 60,
):
    return await scanner_report(x_api_key=x_api_key, activity_limit=activity_limit)


@app.post("/api/v2/quant/scanner/config", response_model=StdResponse, tags=["V2"])
async def scanner_config_v2(
    body: ScannerConfigPayload,
    x_api_key: str | None = Header(None),
):
    return await scanner_config(body=body, x_api_key=x_api_key)


@app.post("/api/v2/quant/scanner/control", response_model=StdResponse, tags=["V2"])
async def scanner_control_v2(
    body: ScannerControlRequest,
    x_api_key: str | None = Header(None),
):
    return await scanner_control(body=body, x_api_key=x_api_key)


@app.post("/api/v2/quant/selector/proposal", response_model=StdResponse, tags=["V2"])
async def selector_proposal_v2(
    body: StrategySelectorRequest,
    x_api_key: str | None = Header(None),
):
    return await selector_proposal(body=body, x_api_key=x_api_key)


async def _quant_live_updates_socket(websocket: WebSocket) -> None:
    api_key = websocket.query_params.get("api_key") or websocket.headers.get("x-api-key")
    if api_key != settings.api_key:
        await websocket.close(code=4401, reason="Invalid API key")
        return
    account_scope = websocket.query_params.get("account_scope")
    account_id = websocket.query_params.get("account_id")
    await websocket.accept()
    try:
        while True:
            try:
                canonical_snapshot_payload = await asyncio.to_thread(
                    _canonical_snapshot_payload,
                    account_scope=account_scope,
                    account_id=account_id,
                )
                status_payload = _build_status_payload(account_scope=account_scope, account_id=account_id)
                operation_status_payload = await asyncio.to_thread(_OPERATION_CENTER.status)
                scanner_report_payload = _compact_scanner_report(await asyncio.to_thread(_SCANNER.report, 24))
                compact_monitor_summary = _compact_monitor_summary(canonical_snapshot_payload["monitor_summary"])
                await websocket.send_json(
                    {
                        "type": "quant.live_update",
                        "generated_at": datetime.utcnow().isoformat(),
                        "status": status_payload.model_dump(),
                        "canonical_snapshot": canonical_snapshot_payload,
                        "monitor_summary": compact_monitor_summary,
                        "operation_status": operation_status_payload,
                        "scanner_report": scanner_report_payload,
                    }
                )
            except Exception as exc:
                logger.exception("Error producing live update payload")
                await websocket.send_json(
                    {
                        "type": "quant.live_error",
                        "generated_at": datetime.utcnow().isoformat(),
                        "error": str(exc),
                    }
                )
            await asyncio.sleep(max(settings.tradier_live_update_interval_sec, 2))
    except WebSocketDisconnect:
        return


@app.websocket("/ws/live-updates")
async def ws_live_updates(websocket: WebSocket):
    await _quant_live_updates_socket(websocket)


@app.websocket("/api/v2/quant/ws/live-updates")
async def ws_live_updates_v2(websocket: WebSocket):
    await _quant_live_updates_socket(websocket)


async def _operation_stream_socket(websocket: WebSocket) -> None:
    api_key = websocket.query_params.get("api_key") or websocket.headers.get("x-api-key")
    if api_key != settings.api_key:
        await websocket.close(code=4401, reason="Invalid API key")
        return
    await websocket.accept()
    try:
        while True:
            payload = await asyncio.to_thread(_OPERATION_CENTER.status)
            await websocket.send_json(
                {
                    "type": "quant.operation_update",
                    "generated_at": datetime.utcnow().isoformat(),
                    "operation_status": payload,
                }
            )
            await asyncio.sleep(max(settings.tradier_live_update_interval_sec, 2))
    except WebSocketDisconnect:
        return
    except Exception:
        logger.exception("Operation websocket loop failed")
        try:
            await websocket.close(code=1011, reason="Operation stream failure")
        except RuntimeError:
            pass


@app.websocket("/ws/operation-stream")
async def ws_operation_stream(websocket: WebSocket):
    await _operation_stream_socket(websocket)


@app.websocket("/api/v2/quant/ws/operation-stream")
async def ws_operation_stream_v2(websocket: WebSocket):
    await _operation_stream_socket(websocket)


@app.post("/strategy/activate", response_model=StdResponse, tags=["Estrategias"])
async def activate_strategy(body: ActivateStrategyRequest, x_api_key: str | None = Header(None)):
    """Activa una estrategia registrada."""
    _auth(x_api_key)
    if body.strategy not in _strategies:
        return StdResponse(ok=False, error=f"Estrategia '{body.strategy}' no encontrada")
    _strategies[body.strategy].activate()
    return StdResponse(ok=True, data={"strategy": body.strategy, "active": True})


@app.post("/strategy/deactivate", response_model=StdResponse, tags=["Estrategias"])
async def deactivate_strategy(body: ActivateStrategyRequest, x_api_key: str | None = Header(None)):
    """Desactiva una estrategia."""
    _auth(x_api_key)
    if body.strategy not in _strategies:
        return StdResponse(ok=False, error=f"Estrategia '{body.strategy}' no encontrada")
    _strategies[body.strategy].deactivate()
    return StdResponse(ok=True, data={"strategy": body.strategy, "active": False})


@app.get("/strategies", response_model=StdResponse, tags=["Estrategias"])
async def list_strategies(x_api_key: str | None = Header(None)):
    """Lista todas las estrategias registradas y su estado."""
    _auth(x_api_key)
    return StdResponse(ok=True, data={
        k: {"active": getattr(v, "active", False), "symbols": getattr(v, "symbols", [])}
        for k, v in _strategies.items()
    })


# ── Backtesting ───────────────────────────────────────────────────────────────

class BacktestRequest(BaseModel if False else object):
    pass


from pydantic import BaseModel as _BM

class BacktestRunRequest(_BM):
    symbol:      str   = "BTC/USDT"
    source:      str   = "ccxt"         # ccxt | yfinance
    exchange:    str   = "binance"
    timeframe:   str   = "1h"
    limit:       int   = 500
    period:      str   = "1y"           # para yfinance
    strategy:    str   = "ma_cross"
    fast_period: int   = 10
    slow_period: int   = 50
    capital:     float = 10_000.0
    commission:  float = 0.001
    generate_html: bool = False


@app.post("/backtest", response_model=StdResponse, tags=["Backtesting"])
async def run_backtest(body: BacktestRunRequest, x_api_key: str | None = Header(None)):
    """Ejecuta un backtest completo y retorna métricas + trades."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        from data.feed import MarketFeed
        from backtesting.engine import BacktestConfig, BacktestEngine
        from backtesting.reporter import export_json

        feed = MarketFeed(exchange_id=body.exchange)
        if body.source == "ccxt":
            df = feed.ohlcv_ccxt(body.symbol, body.timeframe, limit=body.limit)
        else:
            df = feed.ohlcv_yfinance(body.symbol, period=body.period, interval=body.timeframe)

        if df is None or df.empty:
            return StdResponse(ok=False, error="No se pudo obtener datos de mercado")

        if body.strategy == "ma_cross":
            from strategies.ma_cross import MACrossStrategy
            strategy = MACrossStrategy(
                "ma_cross", [body.symbol], timeframe=body.timeframe,
                fast_period=body.fast_period, slow_period=body.slow_period,
            )
        else:
            from models.signals import MLSignalStrategy
            model_name = body.strategy.replace("ml_", "") or "rf"
            strategy = MLSignalStrategy(f"ml_{model_name}", [body.symbol], model_name=model_name)

        config = BacktestConfig(
            initial_capital=body.capital,
            commission_pct=body.commission,
        )
        result = BacktestEngine(strategy=strategy, config=config).run(df, body.symbol)

        # Exportar JSON
        import re
        from pathlib import Path as _P
        reports_dir = _P(__file__).resolve().parent.parent / "reports"
        safe_sym = re.sub(r"[^a-zA-Z0-9]", "-", body.symbol)
        json_path = reports_dir / f"backtest_{safe_sym}_{body.strategy}.json"
        export_json(result, json_path)

        if body.generate_html:
            from backtesting.reporter import generate_html_report
            html_path = reports_dir / f"backtest_{safe_sym}_{body.strategy}.html"
            generate_html_report(result, html_path)

        return StdResponse(
            ok=True,
            data={
                "metrics": result.metrics,
                "trades": [t.to_dict() for t in result.trades[-20:]],  # últimos 20
                "total_trades": len(result.trades),
                "report_path": str(json_path),
                "bars_processed": len(df),
            },
            ms=round((time.perf_counter() - t0) * 1000, 2),
        )
    except Exception as e:
        logger.exception("Error en backtest")
        return StdResponse(ok=False, error=str(e), ms=round((time.perf_counter() - t0) * 1000, 2))


@app.get("/backtest/reports", response_model=StdResponse, tags=["Backtesting"])
async def list_reports(x_api_key: str | None = Header(None)):
    """Lista los reportes de backtest generados."""
    _auth(x_api_key)
    from pathlib import Path as _P
    reports_dir = _P(__file__).resolve().parent.parent / "reports"
    if not reports_dir.exists():
        return StdResponse(ok=True, data=[])
    files = sorted(reports_dir.glob("backtest_*.json"), key=lambda f: f.stat().st_mtime, reverse=True)
    return StdResponse(ok=True, data=[
        {"name": f.name, "size_kb": round(f.stat().st_size / 1024, 1),
         "mtime": datetime.fromtimestamp(f.stat().st_mtime).isoformat()}
        for f in files[:20]
    ])


@app.post("/api/v2/quant/backtest", response_model=StdResponse, tags=["V2"])
async def run_backtest_v2(body: BacktestRunRequest, x_api_key: str | None = Header(None)):
    return await run_backtest(body=body, x_api_key=x_api_key)


@app.get("/api/v2/quant/backtest/reports", response_model=StdResponse, tags=["V2"])
async def list_reports_v2(x_api_key: str | None = Header(None)):
    return await list_reports(x_api_key=x_api_key)


# ── Dashboard Overview (agregador para gráficos) ─────────────────────────────

@app.get("/api/v2/quant/dashboard/overview", response_model=StdResponse, tags=["Dashboard"])
async def dashboard_overview(
    account_scope: str | None = None,
    account_id: str | None = None,
    x_api_key: str | None = Header(None),
):
    """Agrega métricas de journal + posiciones para el dashboard de análisis gráfico."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    import math as _math
    import statistics as _stats
    try:
        scope = account_scope or "paper"
        canonical_snapshot = _canonical_snapshot_payload(account_scope=scope, account_id=account_id)
        canonical_balances = canonical_snapshot.get("balances") or {}
        canonical_totals = canonical_snapshot.get("totals") or {}

        # Estadísticas del journal — estructura: {accounts: {live: {...}, paper: {...}}}
        j_stats = _JOURNAL.stats()
        accounts = j_stats.get("accounts", {}) if isinstance(j_stats, dict) else {}
        acct_stats = accounts.get(scope) or accounts.get("paper") or accounts.get("live") or {}

        # Equity curve → formato {time: unix_sec, value}
        raw_curve = acct_stats.get("equity_curve") or []
        equity_curve = []
        initial_capital = 10_000.0
        for pt in raw_curve:
            ts = pt.get("timestamp")
            try:
                import dateutil.parser as _dp
                unix = int(_dp.parse(ts).timestamp()) if ts else None
            except Exception:
                unix = None
            if unix:
                equity_curve.append({"time": unix, "value": round(initial_capital + float(pt.get("equity", 0)), 2)})

        # Drawdown desde equity curve
        drawdown_curve = []
        peak = initial_capital
        for pt in equity_curve:
            v = pt["value"]
            if v > peak:
                peak = v
            dd_pct = ((v - peak) / peak * 100) if peak > 0 else 0
            drawdown_curve.append({"time": pt["time"], "value": round(dd_pct, 4)})

        # PnL list para histograma
        trade_pnls = [float(pt.get("pnl", 0)) for pt in raw_curve if pt.get("pnl") is not None]

        # Calmar ratio
        realized_pnl = float(acct_stats.get("realized_pnl") or 0)
        max_dd = min((pt["value"] for pt in drawdown_curve), default=0)
        calmar = round(realized_pnl / abs(max_dd), 3) if max_dd < -0.001 else None

        # Monte Carlo básico (bootstrap 200 paths de 50 steps)
        mc_data = None
        if len(trade_pnls) >= 5:
            import random as _rand
            _rand.seed(42)
            n_paths, n_steps = 200, min(50, len(trade_pnls))
            paths = []
            for _ in range(n_paths):
                sample = _rand.choices(trade_pnls, k=n_steps)
                cum = 0.0
                path = []
                for pnl in sample:
                    cum += pnl
                    path.append(round(cum, 2))
                paths.append(path)
            transposed = list(zip(*paths))
            labels = list(range(1, n_steps + 1))
            median = [round(_stats.median(step), 2) for step in transposed]
            p05 = [round(sorted(step)[int(len(step) * 0.05)], 2) for step in transposed]
            p95 = [round(sorted(step)[int(len(step) * 0.95)], 2) for step in transposed]
            mc_var_95 = p05[-1] if p05 else None
            mc_cvar_95 = round(sum(p for p in [pt[-1] for pt in paths] if p <= (mc_var_95 or 0)) / max(1, sum(1 for p in [pt[-1] for pt in paths] if p <= (mc_var_95 or 0))), 2) if mc_var_95 else None
            mc_prob_profit = round(sum(1 for pt in paths if pt[-1] > 0) / n_paths * 100, 1)
            mc_data = {"labels": labels, "median": median, "p05": p05, "p95": p95,
                       "var_95": mc_var_95, "cvar_95": mc_cvar_95, "prob_profit": mc_prob_profit}

        # Trades recientes para streak chart
        recent_trades = [{"pnl": pt.get("pnl", 0)} for pt in raw_curve[-30:]]

        # Heatmap de performance
        heatmap = acct_stats.get("heatmap") or []

        payload = {
            "equity": round(float(canonical_balances.get("total_equity") or (initial_capital + realized_pnl)), 2),
            "realized_pnl": round(realized_pnl, 2),
            "unrealized_pnl": round(float(canonical_totals.get("open_pnl") or acct_stats.get("unrealized_pnl") or 0), 2),
            "sharpe_ratio": acct_stats.get("sharpe_ratio"),
            "win_rate_pct": acct_stats.get("win_rate_pct"),
            "profit_factor": acct_stats.get("profit_factor"),
            "expectancy": acct_stats.get("expectancy"),
            "total_trades": acct_stats.get("trades_closed", 0),
            "open_positions": int(canonical_totals.get("positions") or 0),
            "max_drawdown_pct": round(max_dd, 3) if drawdown_curve else None,
            "calmar_ratio": calmar,
            "equity_curve": equity_curve,
            "drawdown_curve": drawdown_curve,
            "trade_pnls": trade_pnls,
            "monte_carlo": mc_data,
            "recent_trades": recent_trades,
            "heatmap": heatmap,
            "daily_pnl_pct": None,
            "source": canonical_snapshot.get("source"),
            "source_label": canonical_snapshot.get("source_label"),
            "account_scope": canonical_snapshot.get("account_scope"),
            "account_id": canonical_snapshot.get("account_id"),
            "balances": canonical_balances,
            "totals": canonical_totals,
            "reconciliation": canonical_snapshot.get("reconciliation"),
            "simulators": canonical_snapshot.get("simulators"),
            "historical_source": {
                "label": f"Journal {scope}",
                "kind": "journal",
            },
        }
        return StdResponse(ok=True, data=payload, ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as e:
        logger.exception("Error en dashboard/overview")
        return StdResponse(ok=False, error=str(e), ms=round((time.perf_counter() - t0) * 1000, 2))


# ── Fase 3: Alertas ───────────────────────────────────────────────────────────

@app.get("/api/v2/quant/alerts/status", response_model=StdResponse, tags=["Alertas v3"])
async def alerts_status(x_api_key: str | None = Header(None)):
    """Estado del despachador de alertas (Telegram + WhatsApp)."""
    _auth(x_api_key)
    dispatcher = get_alert_dispatcher()
    return StdResponse(ok=True, data=dispatcher.status())


@app.post("/api/v2/quant/alerts/test", response_model=StdResponse, tags=["Alertas v3"])
async def alerts_test(
    message: str = "Test desde Atlas Code-Quant",
    x_api_key: str | None = Header(None),
):
    """Envía una alerta de prueba por Telegram y/o WhatsApp."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    dispatcher = get_alert_dispatcher()
    try:
        await dispatcher.system_error(
            component="dashboard_test",
            error=message,
            critical=False,
        )
        return StdResponse(
            ok=True,
            data={"sent": True, "message": message},
            ms=round((time.perf_counter() - t0) * 1000, 2),
        )
    except Exception as e:
        return StdResponse(ok=False, error=str(e), ms=round((time.perf_counter() - t0) * 1000, 2))


# ── Fase 3: Visión ────────────────────────────────────────────────────────────

@app.get("/api/v2/quant/visual/state", response_model=StdResponse, tags=["Visual v3"])
async def visual_state(x_api_key: str | None = Header(None)):
    """Captura frame actual y devuelve estado visual (features OCR, patrones, safety)."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        builder = get_visual_state_builder()
        state = await asyncio.get_event_loop().run_in_executor(None, builder.capture_and_build)
        data = state.to_dict()
        # Añadir feature_vector serializable
        data["feature_vector"] = state.feature_vector.tolist() if state.feature_vector is not None else []
        return StdResponse(
            ok=True,
            data=data,
            ms=round((time.perf_counter() - t0) * 1000, 2),
        )
    except Exception as e:
        logger.exception("Error en visual/state")
        return StdResponse(ok=False, error=str(e), ms=round((time.perf_counter() - t0) * 1000, 2))


# ── Fase 3: Retraining ────────────────────────────────────────────────────────

@app.get("/api/v2/quant/retraining/status", response_model=StdResponse, tags=["Retraining v3"])
async def retraining_status(
    symbol: str = "BTC/USDT",
    x_api_key: str | None = Header(None),
):
    """Estado del scheduler de retraining (Optuna + PPO) para un símbolo."""
    _auth(x_api_key)
    scheduler = get_retraining_scheduler(symbol=symbol)
    return StdResponse(ok=True, data=scheduler.status())


@app.post("/api/v2/quant/retraining/trigger", response_model=StdResponse, tags=["Retraining v3"])
async def retraining_trigger(
    symbol: str = "BTC/USDT",
    x_api_key: str | None = Header(None),
):
    """Fuerza un ciclo de retraining inmediato (ignora intervalo programado)."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        scheduler = get_retraining_scheduler(symbol=symbol)
        scheduler.force_retrain()
        if not scheduler.status().get("running", False):
            asyncio.ensure_future(scheduler.start())
        return StdResponse(
            ok=True,
            data={"triggered": True, "symbol": symbol, "status": scheduler.status()},
            ms=round((time.perf_counter() - t0) * 1000, 2),
        )
    except Exception as e:
        logger.exception("Error al disparar retraining")
        return StdResponse(ok=False, error=str(e), ms=round((time.perf_counter() - t0) * 1000, 2))


# ═══════════════════════════════════════════════════════════════════════════════
# PAPER TRADING PLATFORM — endpoints /paper/*
# Capital virtual, fills simulados, equity curve, blotter
# ═══════════════════════════════════════════════════════════════════════════════

def _get_pb():
    """Singleton lazy del PaperBroker."""
    from paper.paper_broker import get_paper_broker
    return get_paper_broker()


@app.get("/paper/account", response_model=StdResponse, tags=["Paper Trading"])
async def paper_account(x_api_key: str | None = Header(None)):
    """Resumen completo de la cuenta paper: equity, P&L, win rate, drawdown."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        summary = _get_pb().get_account_summary()
        return StdResponse(ok=True, data=summary.to_dict(),
                           ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as e:
        return StdResponse(ok=False, error=str(e),
                           ms=round((time.perf_counter() - t0) * 1000, 2))


@app.get("/paper/positions", response_model=StdResponse, tags=["Paper Trading"])
async def paper_positions(x_api_key: str | None = Header(None)):
    """Posiciones abiertas con P&L no realizado."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        return StdResponse(ok=True, data=_get_pb().get_positions(),
                           ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as e:
        return StdResponse(ok=False, error=str(e),
                           ms=round((time.perf_counter() - t0) * 1000, 2))


@app.get("/paper/orders", response_model=StdResponse, tags=["Paper Trading"])
async def paper_orders(
    limit: int = 100,
    x_api_key: str | None = Header(None),
):
    """Blotter: historial de órdenes paper (fills + rechazos)."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        return StdResponse(ok=True, data=_get_pb().get_orders(limit=limit),
                           ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as e:
        return StdResponse(ok=False, error=str(e),
                           ms=round((time.perf_counter() - t0) * 1000, 2))


@app.get("/paper/equity-curve", response_model=StdResponse, tags=["Paper Trading"])
async def paper_equity_curve(
    limit: int = 500,
    x_api_key: str | None = Header(None),
):
    """Curva de equity: lista de {ts, equity, cash, realized_pnl, event}."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        return StdResponse(ok=True, data=_get_pb().get_equity_curve_recent(limit=limit),
                           ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as e:
        return StdResponse(ok=False, error=str(e),
                           ms=round((time.perf_counter() - t0) * 1000, 2))


class PaperFillRequest(BaseModel):
    symbol:         str
    side:           str              # buy | sell | sell_short | buy_to_cover
    qty:            int
    price:          float
    strategy:       str   = ""
    signal_score:   float = 0.0
    score_tier:     str   = ""
    option_strategy:str   = ""


@app.post("/paper/fill", response_model=StdResponse, tags=["Paper Trading"])
async def paper_fill(body: PaperFillRequest, x_api_key: str | None = Header(None)):
    """Registra un fill paper manualmente o desde autonomous_loop."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        result = _get_pb().fill(
            symbol=body.symbol,
            side=body.side,
            qty=body.qty,
            price=body.price,
            strategy=body.strategy,
            signal_score=body.signal_score,
            score_tier=body.score_tier,
            option_strategy=body.option_strategy,
        )
        return StdResponse(
            ok=result.status == "filled",
            data={
                "order_id":     result.order_id,
                "status":       result.status,
                "fill_price":   result.fill_price,
                "commission":   result.commission,
                "pnl":          result.pnl,
                "reject_reason":result.reject_reason,
            },
            ms=round((time.perf_counter() - t0) * 1000, 2),
        )
    except Exception as e:
        return StdResponse(ok=False, error=str(e),
                           ms=round((time.perf_counter() - t0) * 1000, 2))


class PaperCloseRequest(BaseModel):
    symbol: str
    price:  float
    qty:    int = 0    # 0 = cerrar todo


@app.post("/paper/close", response_model=StdResponse, tags=["Paper Trading"])
async def paper_close(body: PaperCloseRequest, x_api_key: str | None = Header(None)):
    """Cierra una posición paper (total o parcial)."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        pb = _get_pb()
        positions = pb.get_positions()
        pos = next((p for p in positions if p["symbol"] == body.symbol), None)
        if not pos:
            return StdResponse(ok=False, error=f"No open position for {body.symbol}",
                               ms=round((time.perf_counter() - t0) * 1000, 2))
        qty   = body.qty if body.qty > 0 else pos["qty"]
        side  = "sell" if pos["side"] == "long" else "buy_to_cover"
        result = pb.fill(symbol=body.symbol, side=side, qty=qty, price=body.price)
        return StdResponse(
            ok=result.status == "filled",
            data={"order_id": result.order_id, "pnl": result.pnl, "status": result.status},
            ms=round((time.perf_counter() - t0) * 1000, 2),
        )
    except Exception as e:
        return StdResponse(ok=False, error=str(e),
                           ms=round((time.perf_counter() - t0) * 1000, 2))


class PaperResetRequest(BaseModel):
    initial_capital: float = 0.0    # 0 = usar capital original


@app.post("/paper/reset", response_model=StdResponse, tags=["Paper Trading"])
async def paper_reset(body: PaperResetRequest, x_api_key: str | None = Header(None)):
    """Resetea la cuenta paper al capital inicial (historial de órdenes se conserva)."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        result = _get_pb().reset(new_capital=body.initial_capital or None)
        return StdResponse(ok=True, data=result,
                           ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as e:
        return StdResponse(ok=False, error=str(e),
                           ms=round((time.perf_counter() - t0) * 1000, 2))


# ── V2 aliases ────────────────────────────────────────────────────────────────
@app.get("/api/v2/quant/paper/account",      response_model=StdResponse, tags=["V2"])
async def paper_account_v2(x_api_key: str | None = Header(None)):
    return await paper_account(x_api_key=x_api_key)

@app.get("/api/v2/quant/paper/positions",    response_model=StdResponse, tags=["V2"])
async def paper_positions_v2(x_api_key: str | None = Header(None)):
    return await paper_positions(x_api_key=x_api_key)

@app.get("/api/v2/quant/paper/orders",       response_model=StdResponse, tags=["V2"])
async def paper_orders_v2(limit: int = 100, x_api_key: str | None = Header(None)):
    return await paper_orders(limit=limit, x_api_key=x_api_key)

@app.get("/api/v2/quant/paper/equity-curve", response_model=StdResponse, tags=["V2"])
async def paper_equity_curve_v2(limit: int = 500, x_api_key: str | None = Header(None)):
    return await paper_equity_curve(limit=limit, x_api_key=x_api_key)

@app.post("/api/v2/quant/paper/fill",        response_model=StdResponse, tags=["V2"])
async def paper_fill_v2(body: PaperFillRequest, x_api_key: str | None = Header(None)):
    return await paper_fill(body=body, x_api_key=x_api_key)

@app.post("/api/v2/quant/paper/reset",       response_model=StdResponse, tags=["V2"])
async def paper_reset_v2(body: PaperResetRequest, x_api_key: str | None = Header(None)):
    return await paper_reset(body=body, x_api_key=x_api_key)


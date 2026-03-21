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
import time
import logging
from datetime import datetime

from fastapi import FastAPI, HTTPException, Header, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

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
from monitoring.strategy_tracker import StrategyTracker
from operations.auton_executor import AutonExecutorService
from operations.brain_bridge import QuantBrainBridge
from operations.journal_pro import JournalProService
from operations.operation_center import OperationCenter
from operations.sensor_vision import SensorVisionService
from operations.vision_calibration import VisionCalibrationService
from scanner.opportunity_scanner import OpportunityScannerService
from selector.strategy_selector import StrategySelectorService

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
    await _JOURNAL_SYNC.stop()
    await _SCANNER.stop()
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


def _build_status_payload(
    *,
    account_scope: str | None,
    account_id: str | None,
) -> QuantStatusPayload:
    account_status = _ACCOUNT_MANAGER.status(
        account_scope=account_scope,  # type: ignore[arg-type]
        account_id=account_id,
    )
    return QuantStatusPayload(
        generated_at=datetime.utcnow().isoformat(),
        service_status="ok",
        uptime_sec=round(time.time() - _START_TIME, 1),
        account_session=account_status.session.to_dict(),
        pdt_status=account_status.pdt_status,
        days_trades_used=min(int(account_status.pdt_status.get("day_trades_last_window") or 0), 3),
        active_strategies=_active_strategy_ids(),
        open_positions=len(_portfolio.positions) if _portfolio else 0,
    )


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
async def get_positions(x_api_key: str | None = Header(None)):
    """Retorna posiciones abiertas y resumen del portfolio."""
    _auth(x_api_key)
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
        data = _STRATEGY_TRACKER.build_summary(account_scope=account_scope, account_id=account_id)  # type: ignore[arg-type]
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
async def get_positions_v2(x_api_key: str | None = Header(None)):
    return await get_positions(x_api_key=x_api_key)


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
                monitor_summary_payload = await _STRATEGY_TRACKER.live_update(
                    account_scope=account_scope,  # type: ignore[arg-type]
                    account_id=account_id,
                )
                status_payload = _build_status_payload(account_scope=account_scope, account_id=account_id)
                operation_status_payload = await asyncio.to_thread(_OPERATION_CENTER.status)
                scanner_report_payload = await asyncio.to_thread(_SCANNER.report, 24)
                await websocket.send_json(
                    {
                        "type": "quant.live_update",
                        "generated_at": datetime.utcnow().isoformat(),
                        "status": status_payload.model_dump(),
                        "monitor_summary": monitor_summary_payload["monitor_summary"],
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
            error_type="test",
            details=message,
            symbol=None,
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
        return StdResponse(
            ok=True,
            data={
                "safe_mode": state.safe_mode,
                "brightness": round(state.brightness, 2),
                "sharpness": round(state.sharpness, 2),
                "screens_detected": state.screens_detected,
                "dominant_color": state.dominant_color,
                "prices_detected": state.prices_detected,
                "pattern": state.pattern,
                "feature_vector": state.feature_vector.tolist() if state.feature_vector is not None else [],
                "timestamp": state.timestamp,
            },
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
    scheduler = get_retraining_scheduler(symbol=symbol, brain_bridge=_BRAIN_BRIDGE)
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
        scheduler = get_retraining_scheduler(symbol=symbol, brain_bridge=_BRAIN_BRIDGE)
        scheduler.force_retrain()
        if not scheduler.running:
            asyncio.ensure_future(scheduler.start())
        return StdResponse(
            ok=True,
            data={"triggered": True, "symbol": symbol, "status": scheduler.status()},
            ms=round((time.perf_counter() - t0) * 1000, 2),
        )
    except Exception as e:
        logger.exception("Error al disparar retraining")
        return StdResponse(ok=False, error=str(e), ms=round((time.perf_counter() - t0) * 1000, 2))


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
import concurrent.futures
import hashlib
import math
import os
import time
import logging
import sys
import threading
from collections import defaultdict
from copy import deepcopy
from datetime import datetime
from typing import Any
from zoneinfo import ZoneInfo

# Ampliar thread pool: el default (min(32, cpu+4)) se satura cuando
# yfinance/Yahoo Finance hace múltiples HTTP requests bloqueantes.
# Con 64 threads, el event loop puede servir requests HTTP mientras
# el scanner descarga datos en background.
try:
    _loop = asyncio.get_running_loop()
except RuntimeError:
    _loop = asyncio.new_event_loop()
    asyncio.set_event_loop(_loop)
_loop.set_default_executor(
    concurrent.futures.ThreadPoolExecutor(max_workers=64, thread_name_prefix="quant-io")
)

from pathlib import Path as _PathLib

_API_FILE = _PathLib(__file__).resolve()
_QUANT_ROOT = _API_FILE.parents[1]
_REPO_ROOT = _API_FILE.parents[2]
for _path in (str(_REPO_ROOT), str(_QUANT_ROOT)):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from fastapi import FastAPI, HTTPException, Header, WebSocket, WebSocketDisconnect, Response
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
from starlette.websockets import WebSocketState

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
from atlas_code_quant.config.feature_flags import AtlasFeatureFlags
from atlas_code_quant.intake import RadarOpportunityClient
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
from learning.learning_orchestrator import (
    run_learning_loop,
    stop_learning_loop,
    get_orchestrator_status,
    reconcile_closed_positions,
    apply_ic_to_policy,
    run_daily_analysis as _run_daily_analysis,
)
from monitoring.canonical_snapshot import CanonicalSnapshotService
from monitoring.strategy_tracker import StrategyTracker
from operations.auton_executor import AutonExecutorService
from operations.brain_bridge import QuantBrainBridge
from operations.journal_pro import JournalProService
from operations.operation_center import OperationCenter
from operations.sensor_vision import SensorVisionService
from operations.chart_plan_builder import chart_plan_probe_ok
from operations.decision_shadow import (
    build_advisory_shadow,
    build_baseline_decision,
    record_shadow_triplet,
    resolve_final_paper_decision,
)
from operations.paper_visual_pipeline import collect_visual_evidence
from operations.readiness_eval import evaluate_operational_readiness, startup_warmup_gate_satisfied
from operations.readiness_payload_builder import build_readiness_fast_payload, quant_readiness_flags
from operations.camera_health_payload import build_quant_camera_health_payload
from operations.startup_visual_connect import (
    apply_startup_camera_autoconfigure,
    apply_startup_visual_connections,
)
from operations.vision_calibration import VisionCalibrationService
from selector.strategy_selector import StrategySelectorService
from scanner import OpportunityScannerService
from atlas_code_quant.options.paper_runtime_loop import (
    build_default_options_runtime_loop,
    options_runtime_loop_enabled,
)
from atlas_code_quant.options.options_pipeline_runtime import (
    build_default_options_pipeline_scheduler,
    options_pipeline_runtime_enabled,
)
from atlas_code_quant.options.options_engine_metrics import record_paper_aggressive_decision

# ── OptionStrat ───────────────────────────────────────────────────────────────
from api.routes.options import router as options_router
_XGBOOST_ROUTER_IMPORT_ERROR: Exception | None = None
xgboost_router = None
_XGBOOST_ENABLED_FOR_ROUTER = os.getenv("QUANT_XGBOOST_ENABLED", "false").strip().lower() not in {"0", "false", "no"}
if _XGBOOST_ENABLED_FOR_ROUTER:
    try:
        from api.routers.xgboost_router import router as xgboost_router
    except ImportError as exc:
        _XGBOOST_ROUTER_IMPORT_ERROR = exc
        logging.getLogger("quant.api").warning("XGBoost router disabled (import failed): %s", exc)
from production.grafana_dashboard import GrafanaDashboard

try:
    from prometheus_client import REGISTRY, generate_latest
    _PROM_EXPORT_OK = True
except Exception:
    REGISTRY = None
    generate_latest = None
    _PROM_EXPORT_OK = False

# ── Fase 3: alertas, visión, retraining ──────────────────────────────────────
from operations.alert_dispatcher import get_alert_dispatcher
from learning.visual_state_builder import get_visual_state_builder
from learning.retraining_scheduler import get_retraining_scheduler
from learning.ic_signal_tracker import get_ic_tracker
from learning.duckdb_analytics import get_analytics_engine
from learning.event_store import get_event_store
from learning.strategy_evolver import StrategyEvolver
from knowledge.knowledge_base import get_knowledge_base

logger = logging.getLogger("quant.api")
_NEW_YORK_TZ = ZoneInfo("America/New_York")
_FLAGS = AtlasFeatureFlags()


def _legacy_scanner_enabled() -> bool:
    return bool(_FLAGS.legacy_scanner_enabled)


def _radar_intake_enabled() -> bool:
    return bool(_FLAGS.radar_intake_enabled)


class RadarScannerAdapter:
    """Adapter para mantener contrato scanner interno usando intake Radar."""

    def __init__(self, client: RadarOpportunityClient) -> None:
        self._client = client
        self._running = False
        self._last_error = ""
        self._last_trace_id = ""
        self._last_report: dict[str, Any] = {
            "generated_at": datetime.utcnow().isoformat(),
            "status": {"running": False, "cycle_count": 0, "last_error": ""},
            "summary": {"candidate_count": 0, "source": "radar_intake"},
            "candidates": [],
            "rejections": [],
            "activity": [],
            "order_flow_system": {"provider_ready": False},
        }

    async def start(self) -> None:
        self._running = True

    async def stop(self) -> None:
        self._running = False

    def status(self) -> dict[str, Any]:
        return {
            "running": self._running and _radar_intake_enabled(),
            "mode": "radar_intake",
            "legacy_enabled": _legacy_scanner_enabled(),
            "radar_intake_enabled": _radar_intake_enabled(),
            "last_error": self._last_error,
            "last_trace_id": self._last_trace_id,
            "candidate_count": int((self._last_report.get("summary") or {}).get("candidate_count") or 0),
            "source": "radar",
        }

    def update_config(self, _cfg: dict[str, Any]) -> None:
        # F3: el control fino va por Radar; se mantiene método para compatibilidad.
        return

    async def control(self, action: str) -> dict[str, Any]:
        a = str(action or "").strip().lower()
        if a == "start":
            await self.start()
        elif a == "stop":
            await self.stop()
        elif a in {"cycle", "refresh"}:
            self.report(activity_limit=24)
        return self.report(activity_limit=24)

    @staticmethod
    def _normalize_direction(value: Any) -> str:
        direction = str(value or "").strip().lower()
        if direction in {"alcista", "bullish", "bull", "long", "buy", "up"}:
            return "alcista"
        if direction in {"bajista", "bearish", "bear", "short", "sell", "down"}:
            return "bajista"
        return "neutral"

    @staticmethod
    def _to_candidate(row: dict[str, Any]) -> dict[str, Any]:
        payload = row.get("payload") if isinstance(row.get("payload"), dict) else {}
        merged = {**payload, **row}
        score = row.get("score")
        try:
            selection_score = float(score)
        except (TypeError, ValueError):
            selection_score = 0.0
        horizon_min = row.get("horizon_min")
        try:
            hmin = int(horizon_min)
        except (TypeError, ValueError):
            hmin = 60
        timeframe = "1d" if hmin >= 720 else "1h" if hmin >= 60 else "15m" if hmin >= 15 else "5m"
        direction = RadarScannerAdapter._normalize_direction(merged.get("direction"))
        snapshot = merged.get("snapshot") if isinstance(merged.get("snapshot"), dict) else {}
        price = merged.get("price")
        if price is None:
            price = merged.get("last") or merged.get("close") or snapshot.get("price")
        volume = merged.get("volume")
        if volume is None:
            volume = merged.get("bar_volume") or snapshot.get("volume")
        return {
            "symbol": str(row.get("symbol") or "").strip().upper(),
            "strategy_type": str(merged.get("strategy_type") or row.get("classification") or "radar_intake"),
            "selection_score": selection_score,
            "ml_score": merged.get("ml_score"),
            "local_win_rate_pct": min(100.0, max(0.0, selection_score)),
            "timeframe": timeframe,
            "direction": direction,
            "signal_strength_pct": selection_score,
            "has_options": True,
            "source": str(row.get("source") or "stub"),
            "trace_id": str(row.get("trace_id") or ""),
            "asset_class": str(row.get("asset_class") or "unknown"),
            "timestamp": str(row.get("timestamp") or datetime.utcnow().isoformat()),
            "snapshot_classification": str(row.get("classification") or "watchlist"),
            "degradations_active": list(row.get("degradations_active") or []),
            "price": price,
            "bid": merged.get("bid"),
            "ask": merged.get("ask"),
            "volume": volume,
            "bar_volume": volume,
            "predicted_move_pct": merged.get("predicted_move_pct"),
            "confirmation": merged.get("confirmation") or {},
            "market_regime": merged.get("market_regime") or merged.get("regime"),
            "iv_rank": merged.get("iv_rank") or merged.get("iv_rank_pct"),
            "iv_hv_ratio": merged.get("iv_hv_ratio"),
            "liquidity_score": merged.get("liquidity_score"),
            "skew_pct": merged.get("skew_pct"),
            "term_structure_slope": merged.get("term_structure_slope"),
            "options_thesis": merged.get("options_thesis") or merged.get("thesis"),
            "order_flow": merged.get("order_flow") or {},
        }

    def report(self, activity_limit: int = 24) -> dict[str, Any]:
        if not _radar_intake_enabled():
            self._last_error = "radar_intake_disabled"
            self._last_report = {
                "generated_at": datetime.utcnow().isoformat(),
                "status": {"running": False, "cycle_count": 0, "last_error": self._last_error},
                "summary": {"candidate_count": 0, "source": "radar_intake_disabled"},
                "candidates": [],
                "rejections": [],
                "activity": [{"timestamp": datetime.utcnow().isoformat(), "level": "warn", "message": self._last_error}],
                "order_flow_system": {"provider_ready": False},
            }
            return self._last_report
        res = self._client.fetch_opportunities(
            limit=max(1, int(activity_limit or _FLAGS.radar_intake_limit)),
            min_score=float(_FLAGS.radar_min_score),
        )
        self._last_trace_id = res.trace_id or self._last_trace_id
        candidates = [self._to_candidate(o.to_dict()) for o in res.batch.items]
        self._last_error = str(res.error or "")
        self._last_report = {
            "generated_at": datetime.utcnow().isoformat(),
            "status": {
                "running": self._running,
                "cycle_count": int((self._last_report.get("status") or {}).get("cycle_count") or 0) + 1,
                "last_error": self._last_error,
                "source": "radar",
                "trace_id": self._last_trace_id,
            },
            "summary": {
                "candidate_count": len(candidates),
                "source": "radar_intake",
                "truncated": bool(res.batch.truncated),
                "universe_size": int(res.batch.universe_size),
                "degraded_globally": bool(res.batch.degraded_globally),
            },
            "candidates": candidates,
            "rejections": [],
            "activity": [
                {
                    "timestamp": datetime.utcnow().isoformat(),
                    "level": "warn" if res.degraded else "info",
                    "message": (res.error or "radar_intake_ok"),
                }
            ],
            "order_flow_system": {"provider_ready": bool(res.ok)},
        }
        return self._last_report


_RADAR_CLIENT = RadarOpportunityClient(
    opportunities_url=_FLAGS.radar_opportunities_url,
    stream_url=_FLAGS.radar_stream_url,
    enabled=bool(_FLAGS.radar_intake_enabled),
    timeout_sec=float(_FLAGS.radar_intake_timeout_sec),
    default_limit=int(_FLAGS.radar_intake_limit),
    default_min_score=float(_FLAGS.radar_min_score),
)
_START_TIME = time.time()
_ACCOUNT_MANAGER = AccountManager()
_BRAIN_BRIDGE = QuantBrainBridge()
_ADAPTIVE_LEARNING = AdaptiveLearningService()
_STRATEGY_TRACKER = StrategyTracker()
_CANONICAL_SNAPSHOT = CanonicalSnapshotService(_STRATEGY_TRACKER)
_JOURNAL = TradingJournalService(_STRATEGY_TRACKER, _BRAIN_BRIDGE, _ADAPTIVE_LEARNING, get_ic_tracker())
_JOURNAL_SYNC = JournalSyncService(_JOURNAL)
_VISION = SensorVisionService()
_AUTON_EXECUTOR = AutonExecutorService()
_JOURNAL_PRO = JournalProService(_JOURNAL)
_SCANNER = RadarScannerAdapter(_RADAR_CLIENT) if _radar_intake_enabled() else OpportunityScannerService(_ADAPTIVE_LEARNING)
_VISION_CALIBRATION = VisionCalibrationService()
_SELECTOR = StrategySelectorService(_STRATEGY_TRACKER, _ADAPTIVE_LEARNING)
_GRAFANA_METRICS = GrafanaDashboard() if _PROM_EXPORT_OK else None
_OPERATION_CENTER = OperationCenter(
    tracker=_STRATEGY_TRACKER,
    journal=_JOURNAL_PRO,
    vision=_VISION,
    executor=_AUTON_EXECUTOR,
    brain=_BRAIN_BRIDGE,
    learning=_ADAPTIVE_LEARNING,
    reconciliation_provider=lambda *, account_scope, account_id=None: _CANONICAL_SNAPSHOT.build_snapshot(
        account_scope=account_scope,
        account_id=account_id,
        internal_portfolio=_portfolio,
    ),
)

_SNAPSHOT_CACHE_LOCK = threading.Lock()
_SNAPSHOT_CACHE: dict[str, dict[str, object]] = {}
_SNAPSHOT_CACHE_AT: dict[str, float] = {}
_SNAPSHOT_CACHE_WORKERS: dict[str, threading.Thread] = {}
_SNAPSHOT_CACHE_ERRORS: dict[str, str] = {}
_SNAPSHOT_CACHE_TTL_SEC = 5.0
_SNAPSHOT_CACHE_MAX_STALE_SEC = 180.0
_WS_CLIENT_LIMIT = max(2, min(int(settings.quant_dashboard_ws_limit), 64))
_WS_CLIENT_CONNECTIONS: dict[str, set[WebSocket]] = defaultdict(set)
_WS_CLIENT_LOCK = threading.Lock()

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


@app.get("/metrics", include_in_schema=False)
async def prometheus_metrics() -> Response:
    """Expone el registro Prometheus del proceso Quant en el propio puerto 8795."""
    if not _PROM_EXPORT_OK or REGISTRY is None or generate_latest is None:
        raise HTTPException(status_code=503, detail="prometheus_export_unavailable")
    return Response(
        content=generate_latest(REGISTRY),
        media_type="text/plain; version=0.0.4; charset=utf-8",
    )


@app.get("/options/ui", include_in_schema=False)
async def options_ui():
    """OptionStrat UI — constructor y analizador de estrategias de opciones"""
    idx = _STATIC_DIR / "options" / "index.html"
    if idx.exists():
        return FileResponse(str(idx))
    raise HTTPException(status_code=404, detail="OptionStrat UI not found")


@app.get("/options/runtime-status")
async def options_runtime_status() -> dict[str, Any]:
    """Estado operativo resumido del runtime paper de opciones.

    Expone únicamente el wiring del runtime paper y del scheduler multi-activo
    para facilitar observabilidad y validación por API, sin tocar lógica live.
    """
    return {
        "ok": True,
        "paper_runtime": {
            "enabled": bool(options_runtime_loop_enabled()),
            "running": bool(_options_runtime_task and not _options_runtime_task.done()),
            "task_name": getattr(_options_runtime_task, "get_name", lambda: None)(),
        },
        "multi_asset_pipeline_runtime": {
            "enabled": bool(options_pipeline_runtime_enabled()),
            "running": bool(_options_pipeline_runtime_task and not _options_pipeline_runtime_task.done()),
            "task_name": getattr(_options_pipeline_runtime_task, "get_name", lambda: None)(),
        },
        "paper_performance_endpoint": "/options/paper-performance",
    }

app.include_router(options_router)
if xgboost_router is not None:
    app.include_router(xgboost_router)

_startup_services_task: asyncio.Task | None = None
_metrics_sync_task: asyncio.Task | None = None
_options_runtime_task: asyncio.Task | None = None
_options_pipeline_runtime_task: asyncio.Task | None = None
_STARTUP_BACKGROUND_STATE: dict[str, object] = {
    "running": False,
    "stage": "idle",
    "started_at": None,
    "finished_at": None,
    "last_error": None,
}


def _mark_startup_background(stage: str, *, error: str | None = None) -> None:
    timestamp = datetime.utcnow().isoformat()
    _STARTUP_BACKGROUND_STATE["stage"] = stage
    if stage == "starting":
        _STARTUP_BACKGROUND_STATE["running"] = True
        _STARTUP_BACKGROUND_STATE["started_at"] = timestamp
        _STARTUP_BACKGROUND_STATE["finished_at"] = None
    elif stage in {"completed", "cancelled", "failed"}:
        _STARTUP_BACKGROUND_STATE["running"] = False
        _STARTUP_BACKGROUND_STATE["finished_at"] = timestamp
    if error is not None:
        _STARTUP_BACKGROUND_STATE["last_error"] = error


def _quant_prometheus_enabled() -> bool:
    raw = str(os.environ.get("QUANT_PROMETHEUS_ENABLED", "true")).strip().lower()
    return raw not in {"0", "false", "no", "off"}


async def _run_quant_metrics_loop() -> None:
    if _GRAFANA_METRICS is None:
        logger.info("Quant Prometheus bridge omitido: GrafanaDashboard/prometheus_client no disponible")
        return

    interval_raw = str(os.environ.get("QUANT_PROMETHEUS_SYNC_INTERVAL_SEC", "5")).strip()
    try:
        interval_sec = max(2.0, float(interval_raw))
    except Exception:
        interval_sec = 5.0

    try:
        _GRAFANA_METRICS.enable_embedded_metrics()
        logger.info("Quant Prometheus bridge activo en /metrics (interval=%ss)", interval_sec)
    except Exception:
        logger.exception("No se pudo habilitar Quant Prometheus bridge")
        return

    while True:
        try:
            await asyncio.to_thread(
                _GRAFANA_METRICS.sync_from_canonical,
                account_scope="paper",
                internal_portfolio=_portfolio,
            )
        except asyncio.CancelledError:
            raise
        except Exception:
            logger.debug("Quant Prometheus bridge sync skipped", exc_info=True)
        await asyncio.sleep(interval_sec)


async def _run_options_runtime_loop() -> None:
    """Loop residente paper-only del Options Engine (briefing→entry→autoclose→journal)."""
    if not options_runtime_loop_enabled():
        logger.info("Options runtime loop omitido: QUANT_OPTIONS_RUNTIME_LOOP_ENABLED=false")
        return
    loop = build_default_options_runtime_loop()
    try:
        await loop.run_forever()
    except asyncio.CancelledError:
        loop.stop()
        raise
    except Exception:
        logger.exception("Options runtime loop stopped by error")
        raise


async def _run_options_pipeline_scheduler_loop() -> None:
    """Scheduler residente para ejecutar el pipeline multi-activo en paper."""
    if not options_pipeline_runtime_enabled():
        logger.info("Options pipeline scheduler omitido: QUANT_OPTIONS_PIPELINE_RUNTIME_ENABLED=false")
        return
    scheduler = build_default_options_pipeline_scheduler()
    try:
        await scheduler.run_forever()
    except asyncio.CancelledError:
        scheduler.stop()
        raise
    except Exception:
        logger.exception("Options pipeline scheduler stopped by error")
        raise


@app.on_event("startup")
async def preload_tradier_sessions() -> None:
    _JOURNAL.init_db()
    _ensure_core_runtime_bootstrap()
    global _startup_services_task, _metrics_sync_task, _options_runtime_task, _options_pipeline_runtime_task
    if _quant_prometheus_enabled() and (_metrics_sync_task is None or _metrics_sync_task.done()):
        _metrics_sync_task = asyncio.create_task(
            _run_quant_metrics_loop(),
            name="quant-prometheus-sync",
        )
    if _options_runtime_task is None or _options_runtime_task.done():
        _options_runtime_task = asyncio.create_task(
            _run_options_runtime_loop(),
            name="options-paper-runtime-loop",
        )
    if _options_pipeline_runtime_task is None or _options_pipeline_runtime_task.done():
        _options_pipeline_runtime_task = asyncio.create_task(
            _run_options_pipeline_scheduler_loop(),
            name="options-pipeline-runtime-loop",
        )
    if _startup_services_task and not _startup_services_task.done():
        return
    _startup_services_task = asyncio.create_task(
        _start_background_services(),
        name="quant-background-startup",
    )
    return
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
    # Fase 4 — Learning Orchestrator (loop de aprendizaje cerrado)
    try:
        asyncio.create_task(
            run_learning_loop(reconcile_interval_sec=300),
            name="learning_orchestrator",
        )
        logger.info("LearningOrchestrator iniciado (reconcile cada 5 min)")
    except Exception:
        logger.exception("LearningOrchestrator: fallo en startup (no crítico)")


async def _start_background_services() -> None:
    global _auto_cycle_task
    _mark_startup_background("starting")
    try:
        if _lightweight_startup_enabled():
            logger.warning(
                "QUANT lightweight startup enabled: skipping learning loop and auto-cycle autostart; preserving lightweight data feeders and communication stack when enabled"
            )
            _mark_startup_background("lightweight_preload")
            if settings.startup_preload_sessions_in_background:
                for scope, token in (("paper", settings.tradier_paper_token), ("live", settings.tradier_live_token)):
                    if not token:
                        continue
                    try:
                        await asyncio.to_thread(_ACCOUNT_MANAGER.resolve, account_scope=scope)  # type: ignore[arg-type]
                        logger.info("Tradier %s session preloaded", scope)
                    except Exception:
                        logger.exception("Unable to preload Tradier %s session", scope)
            if settings.startup_visual_connect_enabled:
                try:
                    cam_snap = await asyncio.to_thread(
                        apply_startup_camera_autoconfigure,
                        vision_service=_VISION,
                        settings=settings,
                    )
                    logger.info("Startup camera autoconfigure (lightweight): %s", cam_snap)
                except Exception:
                    logger.exception("Startup camera autoconfigure failed (lightweight, non-fatal)")
                try:
                    snap = await asyncio.to_thread(
                        apply_startup_visual_connections,
                        vision_service=_VISION,
                        operation_center=_OPERATION_CENTER,
                        settings=settings,
                    )
                    logger.info("Startup visual connect (lightweight): %s", snap)
                except Exception:
                    logger.exception("Startup visual connect failed (lightweight)")
            else:
                logger.info("Startup visual connect omitido en lightweight por QUANT_STARTUP_VISUAL_CONNECT_ENABLED=false")
            if settings.startup_alert_dispatcher_enabled:
                try:
                    _mark_startup_background("lightweight_starting_alert_dispatcher")
                    await get_alert_dispatcher().start()
                    logger.info("AlertDispatcher iniciado (lightweight)")
                except Exception:
                    logger.exception("AlertDispatcher: fallo en startup lightweight (no critico)")
            else:
                logger.info("AlertDispatcher omitido en lightweight por QUANT_STARTUP_ALERT_DISPATCHER_ENABLED=false")
            if settings.startup_notifications_enabled:
                try:
                    from notifications.briefing_service import configure_operational_briefing
                    from notifications.scheduler import attach_exit_intelligence_bridge, start_notification_scheduler

                    configure_operational_briefing(
                        operation_center=_OPERATION_CENTER,
                        scanner=_SCANNER,
                        canonical_service=_CANONICAL_SNAPSHOT,
                        vision_service=_VISION,
                        settings=settings,
                    )
                    attach_exit_intelligence_bridge()
                    if settings.notify_enabled:
                        start_notification_scheduler()
                        logger.info("Operational briefing: scheduler activo en lightweight")
                    else:
                        logger.info("Operational briefing: configurado en lightweight; scheduler off (QUANT_NOTIFY_ENABLED=false)")
                except Exception:
                    logger.exception("Operational notifications: fallo en startup lightweight (no critico)")
            else:
                logger.info("Operational notifications omitidas en lightweight por QUANT_STARTUP_NOTIFICATIONS_ENABLED=false")
            if settings.startup_journal_sync_enabled:
                if settings.startup_journal_sync_delay_sec > 0:
                    _mark_startup_background("lightweight_waiting_journal_sync")
                    await asyncio.sleep(settings.startup_journal_sync_delay_sec)
                _mark_startup_background("lightweight_starting_journal_sync")
                await _JOURNAL_SYNC.start()
            else:
                logger.info("Journal sync omitido en lightweight por QUANT_STARTUP_JOURNAL_SYNC_ENABLED=false")
            if settings.startup_scanner_enabled and settings.scanner_auto_start and settings.scanner_enabled:
                if settings.startup_scanner_delay_sec > 0:
                    _mark_startup_background("lightweight_waiting_scanner")
                    await asyncio.sleep(settings.startup_scanner_delay_sec)
                _mark_startup_background("lightweight_starting_scanner")
                await _SCANNER.start()
            elif not settings.startup_scanner_enabled:
                logger.info("Scanner omitido en lightweight por QUANT_STARTUP_SCANNER_ENABLED=false")
            if settings.startup_snapshot_prewarm_enabled:
                _prewarm_quant_snapshot_caches()
            else:
                logger.info("Snapshot prewarm omitido en lightweight por QUANT_STARTUP_SNAPSHOT_PREWARM_ENABLED=false")
            _mark_startup_background("completed_lightweight")
            return
        _mark_startup_background("preloading_sessions")
        if settings.startup_preload_sessions_in_background:
            for scope, token in (("paper", settings.tradier_paper_token), ("live", settings.tradier_live_token)):
                if not token:
                    continue
                try:
                    await asyncio.to_thread(_ACCOUNT_MANAGER.resolve, account_scope=scope)  # type: ignore[arg-type]
                    logger.info("Tradier %s session preloaded", scope)
                except Exception:
                    logger.exception("Unable to preload Tradier %s session", scope)

        if settings.startup_alert_dispatcher_enabled:
            if settings.startup_alert_dispatcher_delay_sec > 0:
                _mark_startup_background("waiting_alert_dispatcher")
                await asyncio.sleep(settings.startup_alert_dispatcher_delay_sec)
            _mark_startup_background("starting_alert_dispatcher")
            try:
                await get_alert_dispatcher().start()
                logger.info("AlertDispatcher iniciado")
            except Exception:
                logger.exception("AlertDispatcher: fallo en startup (no critico)")
        else:
            logger.info("AlertDispatcher omitido por QUANT_STARTUP_ALERT_DISPATCHER_ENABLED=false")

        if settings.startup_notifications_enabled:
            try:
                from notifications.briefing_service import configure_operational_briefing
                from notifications.scheduler import attach_exit_intelligence_bridge, start_notification_scheduler

                configure_operational_briefing(
                    operation_center=_OPERATION_CENTER,
                    scanner=_SCANNER,
                    canonical_service=_CANONICAL_SNAPSHOT,
                    vision_service=_VISION,
                    settings=settings,
                )
                attach_exit_intelligence_bridge()
                if settings.notify_enabled:
                    start_notification_scheduler()
                    logger.info("Operational briefing: scheduler activo (QUANT_NOTIFY_ENABLED=true)")
                else:
                    logger.info("Operational briefing: configurado; scheduler off (QUANT_NOTIFY_ENABLED=false)")
            except Exception:
                logger.exception("Operational notifications: fallo en startup (no crítico)")
        else:
            logger.info("Operational notifications omitidas por QUANT_STARTUP_NOTIFICATIONS_ENABLED=false")

        if settings.startup_journal_sync_enabled:
            if settings.startup_journal_sync_delay_sec > 0:
                _mark_startup_background("waiting_journal_sync")
                await asyncio.sleep(settings.startup_journal_sync_delay_sec)
            _mark_startup_background("starting_journal_sync")
            await _JOURNAL_SYNC.start()
        else:
            logger.info("Journal sync omitido por QUANT_STARTUP_JOURNAL_SYNC_ENABLED=false")

        if settings.startup_scanner_enabled and settings.scanner_auto_start and settings.scanner_enabled:
            if settings.startup_scanner_delay_sec > 0:
                _mark_startup_background("waiting_scanner")
                await asyncio.sleep(settings.startup_scanner_delay_sec)
            _mark_startup_background("starting_scanner")
            await _SCANNER.start()
        elif not settings.startup_scanner_enabled:
            logger.info("Scanner omitido por QUANT_STARTUP_SCANNER_ENABLED=false")

        if settings.startup_learning_enabled:
            if settings.startup_learning_delay_sec > 0:
                _mark_startup_background("waiting_learning_loop")
                await asyncio.sleep(settings.startup_learning_delay_sec)
            _mark_startup_background("starting_learning_loop")
            asyncio.create_task(
                run_learning_loop(reconcile_interval_sec=300),
                name="learning_orchestrator",
            )
            logger.info("LearningOrchestrator iniciado (reconcile cada 5 min)")
        else:
            logger.info("LearningOrchestrator omitido por QUANT_STARTUP_LEARNING_ENABLED=false")
        if _ensure_auto_cycle_running():
            logger.info(
                "Auto-cycle iniciado en startup: mode=%s interval=%ss max_per_cycle=%s",
                _OPERATION_CENTER.get_config().get("auton_mode"),
                _AUTO_CYCLE_STATE.get("loop_interval_sec"),
                _AUTO_CYCLE_STATE.get("max_per_cycle"),
            )
        if settings.startup_snapshot_prewarm_enabled:
            _prewarm_quant_snapshot_caches()
        else:
            logger.info("Snapshot prewarm omitido por QUANT_STARTUP_SNAPSHOT_PREWARM_ENABLED=false")
        if settings.startup_visual_connect_enabled:
            try:
                cam_snap = await asyncio.to_thread(
                    apply_startup_camera_autoconfigure,
                    vision_service=_VISION,
                    settings=settings,
                )
                logger.info("Startup camera autoconfigure: %s", cam_snap)
            except Exception:
                logger.exception("Startup camera autoconfigure failed (non-fatal)")
            try:
                snap = await asyncio.to_thread(
                    apply_startup_visual_connections,
                    vision_service=_VISION,
                    operation_center=_OPERATION_CENTER,
                    settings=settings,
                )
                logger.info("Startup visual connect: %s", snap)
            except Exception:
                logger.exception("Startup visual connect failed")
        else:
            logger.info("Startup visual connect omitido por QUANT_STARTUP_VISUAL_CONNECT_ENABLED=false")
        _mark_startup_background("completed")
    except asyncio.CancelledError:
        _mark_startup_background("cancelled")
        raise
    except Exception as exc:
        logger.exception("Background startup failed")
        _mark_startup_background("failed", error=str(exc))


@app.on_event("shutdown")
async def stop_background_services() -> None:
    global _auto_cycle_task, _startup_services_task, _metrics_sync_task, _options_runtime_task, _options_pipeline_runtime_task
    if _options_pipeline_runtime_task and not _options_pipeline_runtime_task.done():
        _options_pipeline_runtime_task.cancel()
        try:
            await _options_pipeline_runtime_task
        except asyncio.CancelledError:
            pass
        _options_pipeline_runtime_task = None
    if _options_runtime_task and not _options_runtime_task.done():
        _options_runtime_task.cancel()
        try:
            await _options_runtime_task
        except asyncio.CancelledError:
            pass
        _options_runtime_task = None
    if _metrics_sync_task and not _metrics_sync_task.done():
        _metrics_sync_task.cancel()
        try:
            await _metrics_sync_task
        except asyncio.CancelledError:
            pass
        _metrics_sync_task = None
    if _startup_services_task and not _startup_services_task.done():
        _startup_services_task.cancel()
        try:
            await _startup_services_task
        except asyncio.CancelledError:
            pass
        _startup_services_task = None
    await _JOURNAL_SYNC.stop()
    await _SCANNER.stop()
    stop_learning_loop()
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
# Symbols that Tradier sandbox cannot trade (asset class restricted).
# These are excluded from both exit_pass and entry_pass in the auto-cycle.
_SANDBOX_RESTRICTED_SYMBOLS: frozenset[str] = frozenset({"OS", "AA"})

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
    "selector_session_mode_override": None,
    "current_stage": "idle",
    "stage_started_at": None,
    "stage_history": [],
}
_auto_cycle_task: asyncio.Task | None = None


def _safe_float_or_none(value: object) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _ensure_core_runtime_bootstrap() -> None:
    global _portfolio, _strategies
    if _portfolio is None:
        from execution.portfolio import Portfolio

        _portfolio = Portfolio(
            initial_capital=10_000.0,
            max_position_pct=settings.max_position_pct,
            max_drawdown_pct=settings.max_drawdown_pct,
        )
        logger.info("Quant API bootstrap: internal portfolio initialized")
    if "ma_cross" not in _strategies:
        from strategies.ma_cross import MACrossStrategy

        _strategies["ma_cross"] = MACrossStrategy(
            name="ma_cross",
            symbols=settings.assets,
            timeframe=settings.primary_timeframe,
            fast_period=10,
            slow_period=50,
        )
        logger.info("Quant API bootstrap: default strategy registry initialized")


def _lightweight_startup_enabled() -> bool:
    return bool(settings.lightweight_startup)


def _lightweight_live_update_payload(
    *,
    account_scope: str | None,
    account_id: str | None,
) -> dict[str, object]:
    scope = str(account_scope or "paper").strip().lower() or "paper"
    return {
        "type": "quant.live_update",
        "generated_at": datetime.utcnow().isoformat(),
        "lightweight_mode": True,
        "status": {
            "ok": True,
            "mode": "lightweight",
            "account_scope": scope,
            "account_id": account_id,
        },
        "canonical_snapshot": {
            "account_scope": scope,
            "account_id": account_id,
            "source_label": "lightweight",
            "balances": {},
            "totals": {},
            "positions": [],
            "reconciliation": {"state": "deferred"},
            "monitor_summary": {},
            "simulators": {},
        },
        "monitor_summary": {},
        "operation_status": {
            "mode": "lightweight",
            "account_scope": scope,
            "auton_mode_active": False,
            "failsafe": {
                "active": False,
                "reason": "lightweight_startup",
            },
        },
        "scanner_report": {
            "available": False,
            "status": "deferred",
            "reason": "lightweight_startup",
            "activity": [],
        },
    }


def _lightweight_operation_update_payload() -> dict[str, object]:
    return {
        "type": "quant.operation_update",
        "generated_at": datetime.utcnow().isoformat(),
        "lightweight_mode": True,
        "operation_status": {
            "mode": "lightweight",
            "auton_mode_active": False,
            "failsafe": {
                "active": False,
                "reason": "lightweight_startup",
            },
        },
    }


def _live_update_interval_sec() -> float:
    # Protege el loop HTTP cuando el dashboard abre varias pestañas/conexiones.
    return max(float(settings.tradier_live_update_interval_sec or 0), 5.0)


def _lightweight_status_payload(
    *,
    account_scope: str | None,
    account_id: str | None,
) -> dict[str, object]:
    scope = str(account_scope or "paper").strip().lower() or "paper"
    return {
        "account_scope": scope,
        "account_id": account_id,
        "uptime_sec": round(time.time() - _START_TIME, 1),
        "active_strategies": _active_strategy_ids(),
        "account_session": {
            "account_scope": scope,
            "account_id": account_id,
            "mode": "lightweight",
        },
        "source": "lightweight",
        "source_label": "Lightweight",
        "balances": {},
        "totals": {},
        "pdt_status": {},
        "reconciliation": {"state": "deferred"},
        "lightweight_mode": True,
    }


def _lightweight_canonical_snapshot_payload(
    *,
    account_scope: str | None,
    account_id: str | None,
) -> dict[str, object]:
    scope = str(account_scope or "paper").strip().lower() or "paper"
    return {
        "generated_at": datetime.utcnow().isoformat(),
        "account_scope": scope,
        "account_id": account_id,
        "source": "lightweight",
        "source_label": "Lightweight",
        "balances": {},
        "totals": {},
        "positions": [],
        "gross_exposure": 0.0,
        "reconciliation": {"state": "deferred"},
        "monitor_summary": {},
        "simulators": {},
        "lightweight_mode": True,
    }


def _lightweight_monitor_summary_payload(
    *,
    account_scope: str | None,
    account_id: str | None,
) -> dict[str, object]:
    scope = str(account_scope or "paper").strip().lower() or "paper"
    return {
        "generated_at": datetime.utcnow().isoformat(),
        "account_scope": scope,
        "account_id": account_id,
        "source": "lightweight",
        "source_label": "Lightweight",
        "account_session": {
            "account_scope": scope,
            "account_id": account_id,
            "mode": "lightweight",
        },
        "balances": {},
        "totals": {},
        "alerts": [],
        "strategies": [],
        "reconciliation": {"state": "deferred"},
        "simulators": {},
        "lightweight_mode": True,
    }


def _auto_cycle_inactive_reasons(config: dict[str, Any] | None = None) -> list[str]:
    reasons: list[str] = []
    if _lightweight_startup_enabled():
        reasons.append("lightweight_startup_enabled")
    if not bool(settings.autocycle_auto_start):
        reasons.append("autocycle_auto_start_disabled")
    if not bool(settings.scanner_enabled):
        reasons.append("legacy_scanner_setting_disabled")
    elif not bool(settings.scanner_auto_start):
        reasons.append("scanner_auto_start_disabled")
    op_config = config or _OPERATION_CENTER.get_config()
    if not _auton_mode_requires_loop(op_config):
        reasons.append("auton_mode_off")
    return reasons


def _rebased_recent_equity_curve(
    chart_trades: list[dict[str, object]],
    *,
    canonical_balances: dict[str, object],
    canonical_totals: dict[str, object],
) -> list[dict[str, float | int]]:
    """Rebase the latest closed-trade window to the current account equity."""

    import dateutil.parser as _dp

    parsed_points: list[tuple[int, float]] = []
    last_unix: int | None = None
    for trade in chart_trades:
        ts = trade.get("exit_time")
        try:
            parsed_ts = _dp.parse(ts).timestamp() if ts else None
            unix = int(round(parsed_ts)) if parsed_ts is not None and math.isfinite(parsed_ts) else None
        except Exception:
            unix = None
        pnl = _safe_float_or_none(trade.get("pnl"))
        if unix is None or pnl is None:
            continue
        if last_unix is not None and unix <= last_unix:
            unix = last_unix + 1
        parsed_points.append((unix, pnl))
        last_unix = unix

    if not parsed_points:
        return []

    current_total_equity = _safe_float_or_none(canonical_balances.get("total_equity"))
    current_open_pnl = _safe_float_or_none(canonical_totals.get("open_pnl")) or 0.0
    endpoint_equity = (
        current_total_equity - current_open_pnl
        if current_total_equity is not None
        else None
    )

    cumulative_window_pnl = sum(pnl for _, pnl in parsed_points)
    if endpoint_equity is None:
        legacy_tail = _safe_float_or_none((chart_trades[-1] if chart_trades else {}).get("equity"))
        base_equity = (legacy_tail - cumulative_window_pnl) if legacy_tail is not None else 10_000.0
    else:
        base_equity = endpoint_equity - cumulative_window_pnl

    running_equity = base_equity
    equity_curve: list[dict[str, float | int]] = []
    for unix, pnl in parsed_points:
        running_equity += pnl
        equity_curve.append({"time": unix, "value": round(running_equity, 2)})
    return equity_curve


def _drawdown_curve_from_equity(
    equity_curve: list[dict[str, float | int]],
) -> list[dict[str, float | int]]:
    if not equity_curve:
        return []
    peak = float(equity_curve[0]["value"])
    drawdown_curve: list[dict[str, float | int]] = []
    for point in equity_curve:
        value = float(point["value"])
        if value > peak:
            peak = value
        dd_pct = ((value - peak) / peak * 100.0) if peak > 0 else 0.0
        drawdown_curve.append({"time": int(point["time"]), "value": round(max(dd_pct, -100.0), 4)})
    return drawdown_curve


def _auto_cycle_mark_stage(stage: str, **details: Any) -> None:
    timestamp = datetime.utcnow().isoformat()
    previous_stage = str(_AUTO_CYCLE_STATE.get("current_stage") or "idle")
    history = list(_AUTO_CYCLE_STATE.get("stage_history") or [])
    history.append(
        {
            "timestamp": timestamp,
            "stage": stage,
            "previous_stage": previous_stage,
            "details": details or {},
        }
    )
    _AUTO_CYCLE_STATE["current_stage"] = stage
    _AUTO_CYCLE_STATE["stage_started_at"] = timestamp
    _AUTO_CYCLE_STATE["stage_history"] = history[-20:]


def _auton_mode_requires_loop(config: dict[str, Any]) -> bool:
    return str(config.get("auton_mode") or "off").strip().lower() != "off"


def _auton_mode_execution_policy(auton_mode: str, *, requested_max_per_cycle: int) -> dict[str, Any]:
    mode = str(auton_mode or "off").strip().lower()
    action = "submit" if mode not in {"off", "paper_supervised"} else "preview"
    aggressive = mode == "paper_aggressive"
    # Aggressive paper explores wider opportunity set while staying paper-only.
    if aggressive:
        max_per_cycle = max(int(requested_max_per_cycle or 1), min(int(requested_max_per_cycle or 1) * 2, 10))
    else:
        max_per_cycle = max(1, int(requested_max_per_cycle or 1))
    return {
        "mode": mode,
        "action": action,
        "entry_action": action,
        "aggressive": aggressive,
        "effective_max_per_cycle": max_per_cycle,
    }


def _opportunity_trace_ids(candidate: dict[str, Any], *, action: str, auton_mode: str) -> tuple[str, str]:
    symbol = str(candidate.get("symbol") or "").strip().upper()
    timeframe = str(candidate.get("timeframe") or "intraday")
    method = str(candidate.get("method") or candidate.get("strategy_type") or "unknown")
    regime = str(candidate.get("market_regime") or candidate.get("regime") or "unknown")
    score = float(candidate.get("selection_score") or 0.0)
    seed = f"{symbol}|{timeframe}|{method}|{regime}|{score:.6f}|{action}|{auton_mode}"
    digest = hashlib.sha1(seed.encode("utf-8")).hexdigest()
    return (f"opp_{digest[:16]}", f"tr_{digest[16:32]}")


def _ensure_auto_cycle_running() -> bool:
    global _auto_cycle_task
    if _AUTO_CYCLE_STATE.get("running") and _auto_cycle_task and not _auto_cycle_task.done():
        return False
    if _lightweight_startup_enabled() or not bool(settings.autocycle_auto_start):
        logger.info(
            "Auto-cycle autostart skipped: lightweight_startup=%s autocycle_auto_start=%s",
            _lightweight_startup_enabled(),
            bool(settings.autocycle_auto_start),
        )
        return False
    op_config = _OPERATION_CENTER.get_config()
    if not _auton_mode_requires_loop(op_config):
        return False
    interval_sec = int(_AUTO_CYCLE_STATE.get("loop_interval_sec") or 120)
    max_per_cycle = int(_AUTO_CYCLE_STATE.get("max_per_cycle") or 1)
    _AUTO_CYCLE_STATE["selector_session_mode_override"] = None
    _auto_cycle_task = asyncio.create_task(
        _auto_cycle_loop(interval_sec, max_per_cycle),
        name="quant-auto-cycle-startup",
    )
    return True


def _auth(x_api_key: str | None) -> None:
    if x_api_key != settings.api_key:
        raise HTTPException(status_code=401, detail="API key inválida")


def _legacy_scanner_gone_response(*, ms: float) -> JSONResponse:
    payload = StdResponse(
        ok=False,
        data={
            "ok": False,
            "deprecated": True,
            "message": "Legacy scanner disabled; use /api/radar/opportunities",
            "replacement_endpoint": "/api/radar/opportunities",
        },
        error="legacy_scanner_disabled",
        ms=round(ms, 2),
    )
    return JSONResponse(status_code=410, content=payload.model_dump())


def _now_ms() -> float:
    return time.perf_counter() * 1000


def _std_resp(ok: bool, data, ms: float, error: str | None = None) -> StdResponse:
    return StdResponse(ok=ok, data=data, ms=round(ms, 2), error=error)


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


def _snapshot_cache_key(kind: str, *, account_scope: str | None, account_id: str | None) -> str:
    scope = str(account_scope or "paper").strip().lower() or "paper"
    return f"{kind}:{scope}:{str(account_id or '').strip()}"


def _snapshot_cache_get(cache_key: str) -> tuple[dict[str, object] | None, float | None, str | None]:
    with _SNAPSHOT_CACHE_LOCK:
        payload = deepcopy(_SNAPSHOT_CACHE.get(cache_key)) if cache_key in _SNAPSHOT_CACHE else None
        ts = _SNAPSHOT_CACHE_AT.get(cache_key)
        error = _SNAPSHOT_CACHE_ERRORS.get(cache_key)
    age = (time.monotonic() - ts) if ts else None
    return payload, age, error


def _snapshot_cache_set(cache_key: str, payload: dict[str, object]) -> None:
    with _SNAPSHOT_CACHE_LOCK:
        _SNAPSHOT_CACHE[cache_key] = deepcopy(payload)
        _SNAPSHOT_CACHE_AT[cache_key] = time.monotonic()
        _SNAPSHOT_CACHE_ERRORS.pop(cache_key, None)


def _snapshot_cache_set_error(cache_key: str, error: str) -> None:
    with _SNAPSHOT_CACHE_LOCK:
        _SNAPSHOT_CACHE_ERRORS[cache_key] = error


def _snapshot_cache_mark(payload: dict[str, object], *, stale: bool, age_sec: float | None, error: str | None) -> dict[str, object]:
    marked = deepcopy(payload)
    marked["cache"] = {
        "stale": bool(stale),
        "age_sec": round(float(age_sec), 3) if age_sec is not None else None,
        "last_error": error,
    }
    return marked


def _websocket_client_key(websocket: WebSocket) -> str:
    client = websocket.client
    host = getattr(client, "host", None) or "unknown"
    scope = str(websocket.query_params.get("account_scope") or "paper").strip().lower() or "paper"
    return f"{host}:{scope}"


async def _register_ws_connection(websocket: WebSocket) -> tuple[bool, str]:
    """Registra el WS si hay cupo; si no, rechaza la nueva conexión (sin cerrar las demás).

    Desalojar conexiones existentes provocaba bucles reconectar↔evicción y saturaba el event loop,
    dejando HTTP (/health) sin responder.
    """
    client_key = _websocket_client_key(websocket)
    with _WS_CLIENT_LOCK:
        bucket_all = set(_WS_CLIENT_CONNECTIONS.get(client_key, set()))
        active = {ws for ws in bucket_all if getattr(ws, "application_state", None) != WebSocketState.DISCONNECTED}
        if len(active) >= _WS_CLIENT_LIMIT:
            return False, client_key
        active.add(websocket)
        _WS_CLIENT_CONNECTIONS[client_key] = active
        return True, client_key


def _unregister_ws_connection(websocket: WebSocket, client_key: str | None) -> None:
    if not client_key:
        return
    with _WS_CLIENT_LOCK:
        bucket = _WS_CLIENT_CONNECTIONS.get(client_key)
        if not bucket:
            return
        bucket.discard(websocket)
        if bucket:
            _WS_CLIENT_CONNECTIONS[client_key] = bucket
        else:
            _WS_CLIENT_CONNECTIONS.pop(client_key, None)


def _snapshot_cache_start(cache_key: str, builder) -> threading.Thread | None:
    with _SNAPSHOT_CACHE_LOCK:
        current = _SNAPSHOT_CACHE_WORKERS.get(cache_key)
        if current and current.is_alive():
            return current

        def _worker() -> None:
            try:
                payload = builder()
                _snapshot_cache_set(cache_key, payload)
            except Exception as exc:
                logger.warning("Quant snapshot refresh failed for %s: %s", cache_key, exc)
                _snapshot_cache_set_error(cache_key, str(exc))
            finally:
                with _SNAPSHOT_CACHE_LOCK:
                    current_thread = _SNAPSHOT_CACHE_WORKERS.get(cache_key)
                    if current_thread is threading.current_thread():
                        _SNAPSHOT_CACHE_WORKERS.pop(cache_key, None)

        worker = threading.Thread(target=_worker, name=f"quant-cache-{cache_key.replace(':', '-')}", daemon=True)
        _SNAPSHOT_CACHE_WORKERS[cache_key] = worker
        worker.start()
        return worker


def _snapshot_cache_worker_alive(cache_key: str) -> bool:
    with _SNAPSHOT_CACHE_LOCK:
        worker = _SNAPSHOT_CACHE_WORKERS.get(cache_key)
        return bool(worker and worker.is_alive())


async def _resolve_cached_payload(
    *,
    cache_key: str,
    builder,
    warmup_factory,
    timeout_sec: float = 1.25,
    ttl_sec: float = _SNAPSHOT_CACHE_TTL_SEC,
    max_stale_sec: float = _SNAPSHOT_CACHE_MAX_STALE_SEC,
) -> dict[str, object]:
    cached, age_sec, error = _snapshot_cache_get(cache_key)
    if cached is not None and age_sec is not None and age_sec <= ttl_sec:
        return _snapshot_cache_mark(cached, stale=False, age_sec=age_sec, error=error)

    worker = _snapshot_cache_start(cache_key, builder)
    if cached is not None and age_sec is not None and age_sec <= max_stale_sec:
        return _snapshot_cache_mark(cached, stale=True, age_sec=age_sec, error=error)

    deadline = time.perf_counter() + max(float(timeout_sec or 0.0), 0.0)
    while time.perf_counter() < deadline:
        cached, age_sec, error = _snapshot_cache_get(cache_key)
        if cached is not None:
            return _snapshot_cache_mark(cached, stale=False, age_sec=age_sec, error=error)
        if not _snapshot_cache_worker_alive(cache_key):
            break
        await asyncio.sleep(0.05)

    cached, age_sec, error = _snapshot_cache_get(cache_key)
    if cached is not None:
        return _snapshot_cache_mark(cached, stale=True, age_sec=age_sec, error=error)

    warmup_error = error or "refresh_pending"
    if worker and worker.is_alive():
        logger.warning("Quant endpoint warmup fallback for %s: %s", cache_key, warmup_error)
    return _snapshot_cache_mark(warmup_factory(), stale=True, age_sec=None, error=warmup_error)


def _prewarm_quant_snapshot_caches() -> None:
    default_scope = "paper"
    for kind, builder in (
        ("status", lambda: _build_status_payload(account_scope=default_scope, account_id=None).model_dump()),
        ("canonical", lambda: _canonical_snapshot_payload(account_scope=default_scope, account_id=None)),
        ("positions", lambda: _tradier_positions_payload(account_scope=default_scope, account_id=None)),
    ):
        _snapshot_cache_start(_snapshot_cache_key(kind, account_scope=default_scope, account_id=None), builder)


def _scope_account_type(account_scope: str | None) -> str | None:
    scope = str(account_scope or "").strip().lower()
    return scope if scope in {"paper", "live"} else None


def _warmup_positions_payload(*, account_scope: str | None, account_id: str | None) -> dict[str, object]:
    scope = str(account_scope or "paper").strip().lower() or "paper"
    return {
        "source": "warmup",
        "source_label": "Tradier warmup",
        "account_scope": scope,
        "account_id": account_id,
        "positions": [],
        "open_positions": 0,
        "total_pnl": 0.0,
        "market_value": 0.0,
        "gross_exposure": 0.0,
        "reconciliation": {"state": "warming_up"},
        "simulators": {},
        "position_management": {"summary": {}, "watchlist": [], "alerts": []},
        "exit_governance": {"summary": {}, "recommendations": [], "alerts": []},
    }


def _warmup_dashboard_overview_payload(*, account_scope: str | None, account_id: str | None) -> dict[str, object]:
    scope = str(account_scope or "paper").strip().lower() or "paper"
    return {
        "equity": None,
        "realized_pnl": 0.0,
        "unrealized_pnl": 0.0,
        "sharpe_ratio": None,
        "win_rate_pct": None,
        "profit_factor": None,
        "expectancy": None,
        "total_trades": 0,
        "open_positions": 0,
        "max_drawdown_pct": None,
        "calmar_ratio": None,
        "equity_curve": [],
        "drawdown_curve": [],
        "trade_pnls": [],
        "monte_carlo": None,
        "recent_trades": [],
        "heatmap": [],
        "daily_pnl_pct": None,
        "source": "warmup",
        "source_label": "Tradier warmup",
        "account_scope": scope,
        "account_id": account_id,
        "balances": {},
        "totals": {},
        "reconciliation": {"state": "warming_up"},
        "simulators": {},
        "historical_source": {"label": f"Journal {scope}", "kind": "journal"},
        "position_management": {"summary": {}, "watchlist": [], "alerts": []},
        "exit_governance": {"summary": {}, "recommendations": [], "alerts": []},
    }


def _warmup_scanner_report_payload() -> dict[str, object]:
    return {
        "available": False,
        "status": "warming_up",
        "reason": "scanner_report_refresh_pending",
        "activity": [],
    }


def _annotate_position_rows(
    payload: dict[str, object],
    *,
    account_scope: str | None,
) -> dict[str, object]:
    account_type = _scope_account_type(account_scope)
    position_management = _JOURNAL_PRO.position_management_snapshot(account_type=account_type, limit=20)
    exit_governance = _JOURNAL_PRO.exit_governance_snapshot(account_type=account_type, limit=20)
    watchlist_rows = {
        str((item.get("symbol") or "")).strip().upper(): item
        for item in (position_management.get("watchlist") or [])
        if isinstance(item, dict) and str(item.get("symbol") or "").strip()
    }
    recommendation_rows = {
        str((item.get("symbol") or "")).strip().upper(): item
        for item in (exit_governance.get("recommendations") or [])
        if isinstance(item, dict) and str(item.get("symbol") or "").strip()
    }

    positions: list[dict[str, object]] = []
    for raw_position in payload.get("positions") or []:
        if not isinstance(raw_position, dict):
            continue
        position = deepcopy(raw_position)
        key = str(position.get("underlying") or position.get("symbol") or "").strip().upper()
        watch = watchlist_rows.get(key, {})
        recommendation = recommendation_rows.get(key, {})
        position["management_status"] = watch.get("status") or "normal"
        position["alert_reasons"] = list(watch.get("alert_reasons") or [])
        position["holding_hours"] = watch.get("holding_hours")
        position["open_r_multiple"] = watch.get("open_r_multiple")
        position["thesis_drift_pct"] = watch.get("thesis_drift_pct")
        position["risk_budget"] = watch.get("risk_budget")
        position["symbol_heat_pct"] = watch.get("symbol_heat_pct")
        position["exit_recommendation"] = recommendation.get("recommendation") or "hold"
        position["exit_reason"] = recommendation.get("exit_reason") or "monitor_only"
        position["exit_urgency"] = recommendation.get("urgency") or "low"
        positions.append(position)

    enriched = deepcopy(payload)
    enriched["positions"] = positions
    enriched["position_management"] = {
        "summary": position_management.get("summary") or {},
        "watchlist": position_management.get("watchlist") or [],
        "alerts": position_management.get("alerts") or [],
    }
    enriched["exit_governance"] = {
        "summary": exit_governance.get("summary") or {},
        "recommendations": exit_governance.get("recommendations") or [],
        "alerts": exit_governance.get("alerts") or [],
    }
    return enriched


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


def _hhmm_to_minutes(value: str, *, fallback: str) -> int:
    raw = str(value or fallback).strip() or fallback
    try:
        hour_str, minute_str = raw.split(":", 1)
        hour = max(0, min(int(hour_str), 23))
        minute = max(0, min(int(minute_str), 59))
        return (hour * 60) + minute
    except (TypeError, ValueError):
        fallback_hour, fallback_minute = fallback.split(":", 1)
        return (int(fallback_hour) * 60) + int(fallback_minute)


def _entry_pass_runtime_gate(
    *,
    action: str,
    account_scope: str | None,
    account_id: str | None,
    now_et: datetime | None = None,
) -> dict[str, object]:
    normalized_action = str(action or "").strip().lower()
    open_positions = _broker_open_positions_count(account_scope=account_scope, account_id=account_id)
    max_open_positions = max(1, int(getattr(settings, "market_open_max_positions", 3) or 3))
    payload: dict[str, object] = {
        "action": normalized_action,
        "skip_entry_pass": False,
        "reason": None,
        "reason_detail": None,
        "open_positions": open_positions,
        "max_open_positions": max_open_positions,
        "market_open": True,
    }
    if normalized_action != "submit":
        return payload

    current_et = now_et or datetime.now(_NEW_YORK_TZ)
    if current_et.tzinfo is None:
        current_et = current_et.replace(tzinfo=_NEW_YORK_TZ)
    else:
        current_et = current_et.astimezone(_NEW_YORK_TZ)
    open_minutes = _hhmm_to_minutes(settings.market_open_schedule_open_et, fallback="09:30")
    close_minutes = _hhmm_to_minutes(settings.market_open_schedule_close_et, fallback="16:00")
    current_minutes = (current_et.hour * 60) + current_et.minute

    if current_et.weekday() > 4:
        payload.update({
            "skip_entry_pass": True,
            "reason": "market_closed",
            "reason_detail": "market_closed_weekend",
            "market_open": False,
        })
        return payload
    if current_minutes < open_minutes or current_minutes >= close_minutes:
        payload.update({
            "skip_entry_pass": True,
            "reason": "market_closed",
            "reason_detail": f"market_closed_schedule:{settings.market_open_schedule_open_et}-{settings.market_open_schedule_close_et}",
            "market_open": False,
        })
        return payload
    if open_positions >= max_open_positions:
        payload.update({
            "skip_entry_pass": True,
            "reason": "max_open_positions",
            "reason_detail": f"open_positions={open_positions} max_open_positions={max_open_positions}",
        })
        return payload
    return payload


def _market_open_operational_config_snapshot() -> dict[str, object]:
    """Valores efectivos tras resolver env > market_open_config.json > defaults (ver settings)."""
    return {
        "config_path": str(settings.market_open_config_path),
        "config_loaded": bool(getattr(settings, "market_open_config_loaded", False)),
        "schedule_open_et": str(getattr(settings, "market_open_schedule_open_et", "09:30") or "09:30"),
        "schedule_close_et": str(getattr(settings, "market_open_schedule_close_et", "16:00") or "16:00"),
        "scan_interval_min": int(getattr(settings, "market_open_scan_interval_min", 5) or 5),
        "max_positions": int(getattr(settings, "market_open_max_positions", 3) or 3),
        "kelly_fraction": float(getattr(settings, "kelly_fraction", 0.25) or 0.25),
        "kelly_max_position_pct": float(getattr(settings, "kelly_max_position_pct", 0.20) or 0.20),
        "equity_kelly_max_risk_per_trade_pct": float(
            getattr(settings, "equity_kelly_max_risk_per_trade_pct", 0.01) or 0.01
        ),
        "daily_loss_limit_pct": float(getattr(settings, "market_open_daily_loss_limit_pct", 0.02) or 0.02),
    }


def _attach_operational_self_audit_summary(payload: dict[str, object]) -> None:
    """Añade resumen de self-audit operativo al payload de readiness (paper-first; no bloquea submit)."""
    if not settings.operational_self_audit_enabled():
        return
    try:
        from learning.trading_self_audit_protocol import read_trading_self_audit_protocol
        from operations.operational_self_audit import OperationalSelfAuditContext, run_operational_self_audit

        protocol = read_trading_self_audit_protocol(_REPO_ROOT / "reports" / "trading_self_audit_protocol.json")
        oc = _OPERATION_CENTER.get_config()
        if not isinstance(oc, dict):
            oc = {}
        scope_raw = str(oc.get("account_scope") or settings.tradier_default_scope or "paper").strip().lower()
        scope_lit = "live" if scope_raw == "live" else "paper"
        market_snap = payload.get("market_open_operational")
        if not isinstance(market_snap, dict):
            market_snap = {}
        ctx = OperationalSelfAuditContext(
            settings=settings,
            operation_config=oc,
            market_open_snapshot=market_snap,
            protocol=protocol,
            positions_summary=None,
            scope=scope_lit,
        )
        result = run_operational_self_audit(ctx)
        payload["self_audit_summary"] = {
            "overall_severity": result.overall_severity,
            "passed": result.passed,
            "checks_count": len(result.checks),
            "ts_utc": result.ts_utc,
            "scope": result.scope,
        }
    except Exception as exc:
        logger.warning("operational_self_audit: adjunto a readiness falló: %s", exc)


def _tradier_positions_payload(
    *,
    account_scope: str | None,
    account_id: str | None,
) -> dict[str, object]:
    payload = _CANONICAL_SNAPSHOT.build_positions_payload(
        account_scope=account_scope,
        account_id=account_id,
        internal_portfolio=_portfolio,
    )
    return _annotate_position_rows(payload, account_scope=account_scope)


def _equity_strategy_type_for_direction(direction: str | None) -> str:
    return "equity_short" if str(direction or "").strip().lower() in {"short", "bear", "down", "bajista"} else "equity_long"


def _open_symbols_from_positions_payload(payload: dict[str, object]) -> set[str]:
    symbols: set[str] = set()
    for position in payload.get("positions") or []:
        if not isinstance(position, dict):
            continue
        for key in ("underlying", "symbol"):
            value = str(position.get(key) or "").strip().upper()
            if value:
                symbols.add(value)
    return symbols


def _reconciliation_is_healthy(payload: dict[str, object]) -> bool:
    reconciliation = payload.get("reconciliation") or {}
    if not isinstance(reconciliation, dict):
        return False
    return str(reconciliation.get("state") or "").lower() == "healthy"


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


def _warmup_status_payload(*, account_scope: str | None, account_id: str | None) -> dict[str, object]:
    payload = _lightweight_status_payload(account_scope=account_scope, account_id=account_id)
    payload["source"] = "warmup"
    payload["source_label"] = "Quant warmup"
    payload["reconciliation"] = {"state": "warming_up"}
    return payload


async def _cached_status_payload(*, account_scope: str | None, account_id: str | None) -> dict[str, object]:
    cache_key = _snapshot_cache_key("status", account_scope=account_scope, account_id=account_id)
    return await _resolve_cached_payload(
        cache_key=cache_key,
        builder=lambda: _build_status_payload(account_scope=account_scope, account_id=account_id).model_dump(),
        warmup_factory=lambda: _warmup_status_payload(account_scope=account_scope, account_id=account_id),
    )


async def _cached_canonical_snapshot_payload(*, account_scope: str | None, account_id: str | None) -> dict[str, object]:
    cache_key = _snapshot_cache_key("canonical", account_scope=account_scope, account_id=account_id)
    return await _resolve_cached_payload(
        cache_key=cache_key,
        builder=lambda: _canonical_snapshot_payload(account_scope=account_scope, account_id=account_id),
        warmup_factory=lambda: _lightweight_canonical_snapshot_payload(account_scope=account_scope, account_id=account_id),
    )


async def _cached_positions_payload(*, account_scope: str | None, account_id: str | None) -> dict[str, object]:
    cache_key = _snapshot_cache_key("positions", account_scope=account_scope, account_id=account_id)
    return await _resolve_cached_payload(
        cache_key=cache_key,
        builder=lambda: _tradier_positions_payload(account_scope=account_scope, account_id=account_id),
        warmup_factory=lambda: _warmup_positions_payload(account_scope=account_scope, account_id=account_id),
    )


async def _cached_scanner_report_payload() -> dict[str, object]:
    cache_key = _snapshot_cache_key("scanner", account_scope="global", account_id=None)
    return await _resolve_cached_payload(
        cache_key=cache_key,
        builder=lambda: _compact_scanner_report(_SCANNER.report(24)),
        warmup_factory=_warmup_scanner_report_payload,
        timeout_sec=1.0,
        ttl_sec=10.0,
        max_stale_sec=300.0,
    )


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


def _compact_live_canonical_snapshot(snapshot: dict[str, object]) -> dict[str, object]:
    compact = deepcopy(snapshot)
    positions = list(compact.get("positions") or [])
    if positions:
        compact["positions_count"] = len(positions)
        compact["positions_truncated"] = True
        compact["positions"] = []
    compact["monitor_summary"] = _compact_monitor_summary(snapshot.get("monitor_summary") or {})
    exit_governance = compact.get("exit_governance") or {}
    if isinstance(exit_governance, dict):
        compact["exit_governance"] = {
            "summary": exit_governance.get("summary") or {},
            "alerts": list(exit_governance.get("alerts") or [])[:8],
        }
    position_management = compact.get("position_management") or {}
    if isinstance(position_management, dict):
        compact["position_management"] = {
            "summary": position_management.get("summary") or {},
            "alerts": list(position_management.get("alerts") or [])[:8],
        }
    return compact


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
                "ml_score": item.get("ml_score"),
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
        if _lightweight_startup_enabled():
            payload = await _cached_status_payload(account_scope=account_scope, account_id=account_id)
            payload["lightweight_mode"] = True
            session = dict(payload.get("account_session") or {})
            session.setdefault("mode", "lightweight")
            payload["account_session"] = session
            return StdResponse(ok=True, data=payload, ms=round((time.perf_counter() - t0) * 1000, 2))
        payload = await _cached_status_payload(account_scope=account_scope, account_id=account_id)
        return StdResponse(ok=True, data=payload, ms=round((time.perf_counter() - t0) * 1000, 2))
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
        if _lightweight_startup_enabled():
            payload = await _cached_canonical_snapshot_payload(account_scope=account_scope, account_id=account_id)
            payload["lightweight_mode"] = True
            return StdResponse(ok=True, data=payload, ms=round((time.perf_counter() - t0) * 1000, 2))
        payload = await _cached_canonical_snapshot_payload(account_scope=account_scope, account_id=account_id)
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
    t0 = time.perf_counter()
    scope = str(account_scope or "").strip().lower()
    if scope in {"paper", "live"}:
        try:
            payload = await _cached_positions_payload(account_scope=scope, account_id=account_id)
            return StdResponse(ok=True, data=payload, ms=round((time.perf_counter() - t0) * 1000, 2))
        except Exception as exc:
            logger.exception("Error building Tradier positions payload")
            return StdResponse(ok=False, error=str(exc), ms=round((time.perf_counter() - t0) * 1000, 2))
    if not _portfolio:
        return StdResponse(ok=False, error="Portfolio no inicializado", ms=round((time.perf_counter() - t0) * 1000, 2))
    return StdResponse(ok=True, data=_portfolio.summary(), ms=round((time.perf_counter() - t0) * 1000, 2))


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
        if _lightweight_startup_enabled():
            snapshot = await _cached_canonical_snapshot_payload(account_scope=account_scope, account_id=account_id)
            data = dict(snapshot.get("monitor_summary") or {})
            data["generated_at"] = snapshot.get("generated_at")
            data["account_scope"] = snapshot.get("account_scope") or account_scope or "paper"
            data["account_id"] = snapshot.get("account_id") or account_id
            data["source"] = snapshot.get("source")
            data["source_label"] = snapshot.get("source_label")
            data["reconciliation"] = snapshot.get("reconciliation")
            data["simulators"] = snapshot.get("simulators")
            data["lightweight_mode"] = True
            return StdResponse(ok=True, data=data, ms=round((time.perf_counter() - t0) * 1000, 2))
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


@app.get("/journal/chart-data", response_model=StdResponse, tags=["Journal"])
async def journal_chart_data(
    x_api_key: str | None = Header(None),
    account_scope: str = "paper",
    limit: int = 500,
):
    """Datos optimizados para charts avanzados: equity curve, R-multiples, calendar."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        payload = await asyncio.to_thread(_JOURNAL.chart_data, account_scope=account_scope, limit=limit)
        return StdResponse(ok=True, data=payload, ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as exc:
        logger.exception("Error building journal chart-data")
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
        payload = OperationStatusPayload.model_validate(await asyncio.to_thread(_OPERATION_CENTER.status))
        return StdResponse(ok=True, data=payload.model_dump(), ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as exc:
        logger.exception("Error building operation status")
        return StdResponse(ok=False, error=str(exc), ms=round((time.perf_counter() - t0) * 1000, 2))


@app.get("/operation/status/lite", response_model=StdResponse, tags=["Operation"])
async def operation_status_lite(x_api_key: str | None = Header(None)):
    """Lightweight operational status for health checks and tight polling loops."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        payload = _OPERATION_CENTER.status_lite()
        return StdResponse(ok=True, data=payload, ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as exc:
        logger.exception("Error building lightweight operation status")
        return StdResponse(ok=False, error=str(exc), ms=round((time.perf_counter() - t0) * 1000, 2))


def _operation_config_snapshot_payload(config: dict[str, Any]) -> dict[str, Any]:
    vision = config.get("vision") if isinstance(config.get("vision"), dict) else {}
    executor = config.get("executor") if isinstance(config.get("executor"), dict) else {}
    return {
        "generated_at": datetime.utcnow().isoformat(),
        "config": {
            "account_scope": config.get("account_scope"),
            "paper_only": bool(config.get("paper_only", True)),
            "auton_mode": config.get("auton_mode"),
            "executor_mode": config.get("executor_mode") or executor.get("mode"),
            "vision_mode": config.get("vision_mode") or vision.get("provider"),
            "require_operator_present": bool(config.get("require_operator_present", False)),
            "min_auton_win_rate_pct": float(config.get("min_auton_win_rate_pct") or 0.0),
            "max_level4_bpr_pct": float(config.get("max_level4_bpr_pct") or 0.0),
            "auto_pause_on_operational_errors": bool(config.get("auto_pause_on_operational_errors", True)),
            "operational_error_limit": int(config.get("operational_error_limit") or 0),
            "notes": config.get("notes") or "",
        },
        "vision": vision,
        "executor": executor,
    }


@app.post("/operation/config", response_model=StdResponse, tags=["Operation"])
async def operation_config(
    body: OperationConfigPayload,
    x_api_key: str | None = Header(None),
):
    """Update paper-first operational control-plane config."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        _OPERATION_CENTER.update_config(body.model_dump(exclude_unset=True))
        payload = _operation_config_snapshot_payload(await asyncio.to_thread(_OPERATION_CENTER.get_config))
        return StdResponse(ok=True, data=payload, ms=round((time.perf_counter() - t0) * 1000, 2))
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
    """Loop background: toma candidatos del scanner y los evalúa en OperationCenter.

    Ciclo completo por iteración:
    1. Exit pass: evalúa posiciones abiertas contra exit_governance y cierra las urgentes.
    2. Journal sync: sincroniza libro local con broker para actualizar closed trades.
    3. Entry pass: evalúa candidatos del scanner y somete nuevas entradas.
    """
    global _AUTO_CYCLE_STATE
    _AUTO_CYCLE_STATE.update({
        "running": True, "cycle_count": 0, "error": None,
        "started_at": datetime.utcnow().isoformat(), "stopped_at": None,
        "loop_interval_sec": interval_sec, "max_per_cycle": max_per_cycle,
        "current_stage": "starting",
        "stage_started_at": datetime.utcnow().isoformat(),
        "stage_history": [],
    })
    _auto_cycle_mark_stage("starting", interval_sec=interval_sec, max_per_cycle=max_per_cycle)
    logger.info("[auto-cycle] Iniciado. interval=%ds max_per_cycle=%d", interval_sec, max_per_cycle)
    try:
        while _AUTO_CYCLE_STATE["running"]:
            _auto_cycle_mark_stage("sleeping", interval_sec=interval_sec)
            await asyncio.sleep(interval_sec)
            if not _AUTO_CYCLE_STATE["running"]:
                break
            try:
                _auto_cycle_mark_stage("config_refresh")
                op_config = await asyncio.to_thread(_OPERATION_CENTER.get_config)
                policy = _auton_mode_execution_policy(
                    str(op_config.get("auton_mode") or "paper_supervised"),
                    requested_max_per_cycle=max_per_cycle,
                )
                auton_mode = str(policy["mode"])
                action = str(policy["action"])
                entry_action = str(policy["entry_action"])
                effective_max_per_cycle = int(policy["effective_max_per_cycle"])
                _AUTO_CYCLE_STATE["effective_max_per_cycle"] = effective_max_per_cycle

                # ── 1. EXIT PASS: cerrar posiciones que cumplen criterio de salida ─────
                exits_sent = 0
                if action == "submit":
                    _auto_cycle_mark_stage("exit_pass", action=action)
                    try:
                        exit_snap = await asyncio.to_thread(
                            _JOURNAL_PRO.exit_governance_snapshot,
                            account_type="paper", limit=20,
                        )
                        urgent_exits = [
                            c for c in (exit_snap.get("recommendations") or [])
                            if c.get("recommendation") in {"exit_now", "de_risk"}
                            and c.get("urgency") in {"high", "medium"}
                        ]
                        for pos in urgent_exits:
                            sym = str(pos.get("symbol") or "").strip().upper()
                            if not sym:
                                continue
                            # Symbols restricted in Tradier sandbox — skip exit and entry
                            if sym in _SANDBOX_RESTRICTED_SYMBOLS:
                                logger.debug("[auto-cycle] skip sandbox-restricted symbol=%s", sym)
                                continue
                            # strategy_type determina la dirección original de la posición
                            st = str(pos.get("strategy_type") or "equity_long").lower()
                            # long → cierre con sell; short → cierre con buy_to_cover
                            if st not in {"equity_long", "equity_short", "untracked"}:
                                logger.warning(
                                    "[auto-cycle] exit candidate requires manual close build sym=%s strategy=%s reason=%s urgency=%s",
                                    sym,
                                    st,
                                    pos.get("exit_reason"),
                                    pos.get("urgency"),
                                )
                                try:
                                    get_event_store().append("exit.manual_review_required", {
                                        "symbol": sym,
                                        "strategy_type": st,
                                        "reason": pos.get("exit_reason"),
                                        "urgency": pos.get("urgency"),
                                        "journal_key": pos.get("journal_key"),
                                        "recommendation": pos.get("recommendation"),
                                    }, source="auto_cycle")
                                except Exception:
                                    pass
                                continue
                            close_side = "sell" if st != "equity_short" else "buy_to_cover"
                            close_order = OrderRequest(
                                symbol=sym,
                                side=close_side,  # type: ignore[arg-type]
                                size=1,
                                order_type="market",
                                asset_class="equity",  # type: ignore[arg-type]
                                account_scope="paper",
                                preview=False,  # cierre real, no preview
                                position_effect="close",  # evita que pase por gate de apertura
                                strategy_type=st,  # type: ignore[arg-type]
                                tag="exitgov",
                            )
                            try:
                                close_result = await asyncio.to_thread(
                                    _OPERATION_CENTER.evaluate_candidate,
                                    order=close_order, action="submit", capture_context=False,
                                )
                                exits_sent += 1
                                logger.info(
                                    "[auto-cycle] EXIT %s sym=%s reason=%s urgency=%s blocked=%s",
                                    close_side.upper(), sym,
                                    pos.get("exit_reason"), pos.get("urgency"),
                                    close_result.get("blocked"),
                                )
                                try:
                                    get_event_store().append("exit.submitted", {
                                        "symbol": sym, "side": close_side,
                                        "reason": pos.get("exit_reason"),
                                        "urgency": pos.get("urgency"),
                                        "blocked": close_result.get("blocked"),
                                    }, source="auto_cycle")
                                except Exception:
                                    pass
                            except Exception as _ex_err:
                                logger.warning("[auto-cycle] exit order error sym=%s: %s", sym, _ex_err)
                    except Exception as _eg_err:
                        logger.warning("[auto-cycle] exit_governance_snapshot error: %s", _eg_err)

                # ── 2. JOURNAL SYNC: sincronizar libro con broker ─────────────────────
                if action == "submit":
                    try:
                        _auto_cycle_mark_stage("journal_sync", action=action)
                        await asyncio.to_thread(_JOURNAL.sync_scope, "paper")
                        logger.debug("[auto-cycle] journal sync OK")
                    except Exception as _sync_err:
                        logger.debug("[auto-cycle] journal sync warning: %s", _sync_err)
                else:
                    _auto_cycle_mark_stage("journal_sync_skipped", action=action)

                # ── 3. ENTRY PASS: evaluar nuevas entradas ────────────────────────────
                entry_gate = await asyncio.to_thread(
                    _entry_pass_runtime_gate,
                    action=entry_action,
                    account_scope="paper",
                    account_id=None,
                )
                current_open_positions = int(entry_gate.get("open_positions") or 0)
                max_open_positions = int(entry_gate.get("max_open_positions") or 3)
                if bool(entry_gate.get("skip_entry_pass")):
                    reason = str(entry_gate.get("reason") or "entry_pass_skipped")
                    reason_detail = str(entry_gate.get("reason_detail") or reason)
                    _AUTO_CYCLE_STATE.update({
                        "cycle_count": _AUTO_CYCLE_STATE["cycle_count"] + 1,
                        "last_cycle_at": datetime.utcnow().isoformat(),
                        "last_action": action,
                        "entry_action_last": entry_action,
                        "last_result": {
                            "action": entry_action,
                            "blocked": True,
                            "reasons": [reason_detail],
                            "exits_sent": exits_sent,
                            "open_positions": current_open_positions,
                            "max_open_positions": max_open_positions,
                        },
                        "exits_sent_last_cycle": exits_sent,
                    })
                    if reason == "market_closed":
                        _auto_cycle_mark_stage("entry_skipped_market_closed", action=entry_action, detail=reason_detail)
                        logger.info("[auto-cycle] market closed, skipping entry pass (%s)", reason_detail)
                    else:
                        _auto_cycle_mark_stage(
                            "entry_skipped_max_open_positions",
                            action=entry_action,
                            open_positions=current_open_positions,
                            max_open_positions=max_open_positions,
                        )
                        logger.info(
                            "[auto-cycle] max_open_positions guard triggered, skipping entry pass (%s/%s)",
                            current_open_positions,
                            max_open_positions,
                        )
                    continue

                _auto_cycle_mark_stage("scanner_report", action=entry_action, configured_action=action)
                report = await asyncio.to_thread(_SCANNER.report, 24)
                candidates = list(report.get("candidates") or [])
                if not candidates:
                    _AUTO_CYCLE_STATE["last_cycle_at"] = datetime.utcnow().isoformat()
                    _AUTO_CYCLE_STATE["cycle_count"] += 1
                    _auto_cycle_mark_stage("cycle_complete", action=entry_action, candidates=0, exits_sent=exits_sent)
                    logger.debug("[auto-cycle] Ciclo %d — sin candidatos (exits_sent=%d)",
                                 _AUTO_CYCLE_STATE["cycle_count"], exits_sent)
                    continue

                sorted_cands = sorted(
                    [c for c in candidates if str(c.get("symbol") or "").strip().upper() not in _SANDBOX_RESTRICTED_SYMBOLS],
                    key=lambda c: (
                        float(c.get("selection_score") or 0),
                        float(c.get("ml_score")) if c.get("ml_score") is not None else -1.0,
                    ),
                    reverse=True,
                )

                positions_payload = _tradier_positions_payload(account_scope="paper", account_id=None)
                open_symbols = _open_symbols_from_positions_payload(positions_payload)
                rebuild_guard_active = _OPERATION_CENTER.paper_rebuild_guard_active(
                    account_scope="paper",
                    reconciliation=positions_payload.get("reconciliation") if isinstance(positions_payload, dict) else None,
                )

                # Reconciliation gate: in paper mode, only block if Tradier didn't respond
                # (the paper_local simulator always reports 0 positions, causing false failures)
                _recon_source = str((positions_payload.get("source") or "")).lower()
                _recon_has_data = bool(positions_payload.get("positions"))
                _recon_healthy = _reconciliation_is_healthy(positions_payload)
                if entry_action == "submit" and not _recon_healthy:
                    if _recon_source == "tradier" and _recon_has_data:
                        # Tradier responded with positions — paper_local mismatch is expected, proceed
                        logger.info("[auto-cycle] reconciliation degraded (paper_local mismatch) but Tradier OK — proceeding")
                    else:
                        _AUTO_CYCLE_STATE.update({
                            "cycle_count": _AUTO_CYCLE_STATE["cycle_count"] + 1,
                            "last_cycle_at": datetime.utcnow().isoformat(),
                            "last_action": action,
                            "last_result": {
                                "action": action,
                                "blocked": True,
                                "reasons": ["Reconciliation gate: broker no responde o estado desconocido."],
                                "exits_sent": exits_sent,
                            },
                        })
                        _auto_cycle_mark_stage("blocked_reconciliation", action=action, exits_sent=exits_sent)
                        logger.warning("[auto-cycle] entry blocked: reconciliation unhealthy, source=%s (exits_sent=%d)", _recon_source, exits_sent)
                        continue

                last_result = None
                executed_count = 0
                selector_session_mode = str(
                    _AUTO_CYCLE_STATE.get("selector_session_mode_override")
                    or settings.selector_session_mode
                    or "balanced"
                ).lower()
                require_optionable = bool(settings.selector_options_require_available)
                for cand in sorted_cands:
                    if entry_action == "submit" and current_open_positions + executed_count >= max_open_positions:
                        last_result = {
                            "action": entry_action,
                            "blocked": True,
                            "reasons": [
                                f"Max open positions guard blocked new entry ({current_open_positions + executed_count}/{max_open_positions})."
                            ],
                            "exits_sent": exits_sent,
                            "open_positions": current_open_positions,
                            "max_open_positions": max_open_positions,
                        }
                        logger.info(
                            "[auto-cycle] max_open_positions=%d reached (open=%d attempted=%d) - stopping entry pass",
                            max_open_positions,
                            current_open_positions,
                            executed_count,
                        )
                        break
                    symbol = str(cand.get("symbol") or "").strip()
                    if not symbol:
                        continue
                    opportunity_id, trace_id = _opportunity_trace_ids(
                        dict(cand),
                        action=entry_action,
                        auton_mode=auton_mode,
                    )
                    _auto_cycle_mark_stage(
                        "candidate_selected",
                        symbol=symbol,
                        action=entry_action,
                        selector_session_mode=selector_session_mode,
                    )
                    if selector_session_mode == "options_only" and require_optionable and not bool(cand.get("has_options")):
                        visual_evidence = (
                            collect_visual_evidence(
                                symbol=symbol,
                                timeframe=str(cand.get("timeframe") or "5m"),
                                opportunity_id=opportunity_id,
                                trace_id=trace_id,
                                chart_source=str(getattr(settings, "chart_provider_default", "tradingview") or "tradingview"),
                                candidate=dict(cand),
                                chart_plan=dict(cand.get("chart_plan") or {}),
                                camera_plan=dict(cand.get("camera_plan") or {}),
                            )
                            if auton_mode == "paper_aggressive"
                            else {}
                        )
                        baseline = build_baseline_decision(
                            candidate=dict(cand),
                            order_seed={"symbol": symbol},
                            action=entry_action,
                            auton_mode=auton_mode,
                            opportunity_id=opportunity_id,
                            trace_id=trace_id,
                            visual_context=visual_evidence,
                        )
                        advisory = build_advisory_shadow(baseline=baseline, candidate=dict(cand))
                        final_decision = resolve_final_paper_decision(
                            baseline=baseline,
                            advisory=advisory,
                            evaluate_result=None,
                            skip_reason="options_only_requires_options",
                        )
                        try:
                            record_shadow_triplet(
                                event_store=get_event_store(),
                                baseline=baseline,
                                advisory=advisory,
                                final=final_decision,
                            )
                        except Exception:
                            pass
                        try:
                            record_paper_aggressive_decision(
                                decision_path="final",
                                mode=auton_mode,
                                pre_trade_score=float(cand.get("selection_score") or 0.0),
                                executed=False,
                                session=str(cand.get("timeframe") or "intraday"),
                                regime=str(cand.get("market_regime") or cand.get("regime") or "unknown"),
                                strategy=str(cand.get("strategy_type") or "unknown"),
                                model_path=str(advisory.get("requested_model") or "none"),
                                policy_variant=str(final_decision.get("policy_variant") or "baseline_v1"),
                                cohort_id=str(final_decision.get("cohort_id") or "cohort_unknown"),
                                error_taxonomy_v2=final_decision.get("error_taxonomy_v2"),
                                realism=final_decision.get("paper_realism"),
                                visual_evidence=final_decision.get("visual_evidence"),
                            )
                        except Exception:
                            pass
                        last_result = {
                            "symbol": symbol,
                            "action": entry_action,
                            "blocked": True,
                            "reasons": [f"Options-only session skipped {symbol.upper()} because has_options is false."],
                            "selection_score": cand.get("selection_score"),
                            "win_rate_pct": None,
                            "opportunity_id": opportunity_id,
                            "trace_id": trace_id,
                            "decision_id": baseline.get("decision_id"),
                        }
                        logger.info("[auto-cycle] symbol=%s action=%s blocked=options_only_requires_options", symbol, entry_action)
                        continue
                    if symbol.upper() in open_symbols and not rebuild_guard_active:
                        visual_evidence = (
                            collect_visual_evidence(
                                symbol=symbol,
                                timeframe=str(cand.get("timeframe") or "5m"),
                                opportunity_id=opportunity_id,
                                trace_id=trace_id,
                                chart_source=str(getattr(settings, "chart_provider_default", "tradingview") or "tradingview"),
                                candidate=dict(cand),
                                chart_plan=dict(cand.get("chart_plan") or {}),
                                camera_plan=dict(cand.get("camera_plan") or {}),
                            )
                            if auton_mode == "paper_aggressive"
                            else {}
                        )
                        baseline = build_baseline_decision(
                            candidate=dict(cand),
                            order_seed={"symbol": symbol},
                            action=entry_action,
                            auton_mode=auton_mode,
                            opportunity_id=opportunity_id,
                            trace_id=trace_id,
                            visual_context=visual_evidence,
                        )
                        advisory = build_advisory_shadow(baseline=baseline, candidate=dict(cand))
                        final_decision = resolve_final_paper_decision(
                            baseline=baseline,
                            advisory=advisory,
                            evaluate_result=None,
                            skip_reason="open_symbol_guard",
                        )
                        try:
                            record_shadow_triplet(
                                event_store=get_event_store(),
                                baseline=baseline,
                                advisory=advisory,
                                final=final_decision,
                            )
                        except Exception:
                            pass
                        try:
                            record_paper_aggressive_decision(
                                decision_path="final",
                                mode=auton_mode,
                                pre_trade_score=float(cand.get("selection_score") or 0.0),
                                executed=False,
                                session=str(cand.get("timeframe") or "intraday"),
                                regime=str(cand.get("market_regime") or cand.get("regime") or "unknown"),
                                strategy=str(cand.get("strategy_type") or "unknown"),
                                model_path=str(advisory.get("requested_model") or "none"),
                                policy_variant=str(final_decision.get("policy_variant") or "baseline_v1"),
                                cohort_id=str(final_decision.get("cohort_id") or "cohort_unknown"),
                                error_taxonomy_v2=final_decision.get("error_taxonomy_v2"),
                                realism=final_decision.get("paper_realism"),
                                visual_evidence=final_decision.get("visual_evidence"),
                            )
                        except Exception:
                            pass
                        last_result = {
                            "symbol": symbol,
                            "action": entry_action,
                            "blocked": True,
                            "reasons": [f"Open-symbol guard blocked re-entry for {symbol.upper()}."],
                            "selection_score": cand.get("selection_score"),
                            "win_rate_pct": None,
                            "opportunity_id": opportunity_id,
                            "trace_id": trace_id,
                            "decision_id": baseline.get("decision_id"),
                        }
                        logger.info("[auto-cycle] symbol=%s action=%s blocked=open_symbol_guard", symbol, entry_action)
                        continue
                    if symbol.upper() in open_symbols and rebuild_guard_active:
                        logger.warning(
                            "[auto-cycle] open-symbol guard bypassed post journal rebuild for %s while reconciliation stabilizes",
                            symbol,
                        )
                    direction = str(cand.get("direction") or "long").lower()
                    fallback_side = "buy" if direction in {"long", "bull", "up"} else "sell_short"
                    _auto_cycle_mark_stage("selector_proposal", symbol=symbol, action=entry_action)
                    proposal = _SELECTOR.proposal(
                        candidate=dict(cand),
                        account_scope="paper",
                        options_session_mode=selector_session_mode,
                        chart_provider=str(getattr(settings, "chart_provider_default", "tradingview") or "tradingview"),
                    )
                    order_seed = dict(proposal.get("order_seed") or {})
                    if not order_seed:
                        ac_raw = str(cand.get("asset_class") or "equity").lower()
                        asset_class = ac_raw if ac_raw in {"equity", "option", "multileg", "combo"} else "equity"
                        order_seed = {
                            "symbol": symbol,
                            "side": fallback_side,
                            "size": 1,
                            "order_type": "market",
                            "asset_class": asset_class,
                            "account_scope": "paper",
                            "preview": (entry_action == "preview"),
                            "strategy_type": _equity_strategy_type_for_direction(direction) if asset_class == "equity" else None,
                            "entry_reference_price": float(cand.get("price") or 0.0) or None,
                            "entry_expected_move_pct": float(cand.get("predicted_move_pct") or 0.0) or None,
                            "entry_confidence_reference_pct": float(cand.get("local_win_rate_pct") or 0.0) or None,
                        }
                    asset_class = str(order_seed.get("asset_class") or cand.get("asset_class") or "equity").lower()
                    if asset_class == "equity" and not order_seed.get("strategy_type"):
                        order_seed["strategy_type"] = _equity_strategy_type_for_direction(direction)
                    side = str(order_seed.get("side") or fallback_side).lower()
                    order_seed["preview"] = (entry_action == "preview")
                    order = OrderRequest(**order_seed)
                    _auto_cycle_mark_stage(
                        "operation_center_evaluate",
                        symbol=symbol,
                        action=entry_action,
                        side=side,
                        asset_class=asset_class,
                    )
                    result = await asyncio.to_thread(
                        _OPERATION_CENTER.evaluate_candidate,
                        order=order,
                        action=entry_action,
                        capture_context=bool(order.chart_plan or order.camera_plan),  # type: ignore[call-arg]
                    )
                    visual_evidence = (
                        collect_visual_evidence(
                            symbol=symbol,
                            timeframe=str(cand.get("timeframe") or "5m"),
                            opportunity_id=opportunity_id,
                            trace_id=trace_id,
                            chart_source=str(getattr(settings, "chart_provider_default", "tradingview") or "tradingview"),
                            candidate=dict(cand),
                            chart_plan=dict(order_seed.get("chart_plan") or cand.get("chart_plan") or {}),
                            camera_plan=dict(order_seed.get("camera_plan") or cand.get("camera_plan") or {}),
                        )
                        if auton_mode == "paper_aggressive"
                        else {}
                    )
                    baseline = build_baseline_decision(
                        candidate=dict(cand),
                        order_seed=dict(order_seed),
                        action=entry_action,
                        auton_mode=auton_mode,
                        opportunity_id=opportunity_id,
                        trace_id=trace_id,
                        visual_context=visual_evidence,
                    )
                    advisory = build_advisory_shadow(baseline=baseline, candidate=dict(cand))
                    final_decision = resolve_final_paper_decision(
                        baseline=baseline,
                        advisory=advisory,
                        evaluate_result=result,
                    )
                    try:
                        record_shadow_triplet(
                            event_store=get_event_store(),
                            baseline=baseline,
                            advisory=advisory,
                            final=final_decision,
                        )
                    except Exception as _shadow_err:
                        logger.debug("[auto-cycle] shadow triplet skipped: %s", _shadow_err)
                    if auton_mode == "paper_aggressive":
                        try:
                            get_event_store().append(
                                "visual.evidence",
                                {
                                    "symbol": symbol,
                                    "timeframe": str(cand.get("timeframe") or "5m"),
                                    "opportunity_id": opportunity_id,
                                    "trace_id": trace_id,
                                    "decision_id": baseline.get("decision_id"),
                                    "provider_used": visual_evidence.get("provider_used"),
                                    "used_fallback": visual_evidence.get("used_fallback"),
                                    "fallback_reason": visual_evidence.get("fallback_reason"),
                                    "screenshot_path": visual_evidence.get("screenshot_path"),
                                    "meta_path": visual_evidence.get("meta_path"),
                                    "chart_bias": visual_evidence.get("chart_bias"),
                                    "confluence_score": visual_evidence.get("confluence_score"),
                                    "confluence_bucket": visual_evidence.get("confluence_bucket"),
                                    "fusion_decision_reason": visual_evidence.get("fusion_decision_reason"),
                                    "recommended_action": visual_evidence.get("recommended_action"),
                                    "visual_confidence": visual_evidence.get("visual_confidence"),
                                    "visual_decision_context": visual_evidence.get("visual_decision_context"),
                                    "seasonality_context": visual_evidence.get("seasonality_context"),
                                    "multi_timeframe_view": visual_evidence.get("multi_timeframe_view"),
                                    "operational_context": visual_evidence.get("operational_context"),
                                },
                                source="auto_cycle",
                            )
                        except Exception:
                            pass
                    try:
                        record_paper_aggressive_decision(
                            decision_path="final",
                            mode=auton_mode,
                            pre_trade_score=float(cand.get("selection_score") or 0.0),
                            executed=bool(final_decision.get("executed_in_paper")),
                            session=str(cand.get("timeframe") or "intraday"),
                            regime=str(cand.get("market_regime") or cand.get("regime") or "unknown"),
                            strategy=str(order_seed.get("strategy_type") or cand.get("strategy_type") or "unknown"),
                            model_path=str(advisory.get("requested_model") or "none"),
                            policy_variant=str(final_decision.get("policy_variant") or "baseline_v1"),
                            cohort_id=str(final_decision.get("cohort_id") or "cohort_unknown"),
                            error_taxonomy_v2=final_decision.get("error_taxonomy_v2"),
                            realism=final_decision.get("paper_realism"),
                            visual_evidence=final_decision.get("visual_evidence"),
                        )
                    except Exception as _metrics_err:
                        logger.debug("[auto-cycle] aggressive decision metrics skipped: %s", _metrics_err)
                    last_result = {
                        "symbol": symbol, "action": entry_action,
                        "blocked": bool(result.get("blocked")),
                        "reasons": (result.get("reasons") or []),
                        "selection_score": cand.get("selection_score"),
                        "win_rate_pct": (result.get("probability") or {}).get("win_rate_pct"),
                        "exits_sent": exits_sent,
                        "decision_id": baseline.get("decision_id"),
                        "opportunity_id": baseline.get("opportunity_id"),
                        "trace_id": baseline.get("trace_id"),
                        "policy_variant": final_decision.get("policy_variant"),
                        "cohort_id": final_decision.get("cohort_id"),
                        "baseline_decision": final_decision.get("baseline_decision"),
                        "advisory_decision": final_decision.get("advisory_decision"),
                        "final_decision": final_decision.get("final_decision"),
                    }
                    # IC Signal Tracker: registrar señal para medir poder predictivo del scanner
                    _ic_signal_id: str | None = None
                    if not last_result["blocked"]:
                        try:
                            _predicted_move = float(cand.get("predicted_move_pct") or 0.0)
                            _entry_price = float(cand.get("price") or 0.0)
                            if _entry_price > 0 and _predicted_move != 0.0:
                                _ic_signal_id = get_ic_tracker().record_signal(
                                    symbol=symbol,
                                    method=str(cand.get("method") or cand.get("strategy_type") or "unknown"),
                                    predicted_move_pct=_predicted_move * (1.0 if side == "buy" else -1.0),
                                    entry_price=_entry_price,
                                    timeframe=str(cand.get("timeframe") or "1d"),
                                    selection_score=float(cand.get("selection_score") or 0.0),
                                )
                                if _ic_signal_id:
                                    last_result["ic_signal_id"] = _ic_signal_id
                        except Exception as _ic_err:
                            logger.debug("[auto-cycle] ic_tracker.record_signal skipped: %s", _ic_err)
                    # Knowledge advisory — contexto académico no bloqueante
                    if not last_result["blocked"] and entry_action == "submit":
                        try:
                            _kb_ctx = get_knowledge_base().advisory_context(
                                method=str(cand.get("method") or cand.get("strategy_type") or ""),
                                timeframe=str(cand.get("timeframe") or "1d"),
                            )
                            last_result["knowledge_advisory"] = {
                                "academic_support_score": _kb_ctx.get("academic_support_score"),
                                "warnings": _kb_ctx.get("warnings", []),
                                "recommendations": _kb_ctx.get("recommendations", []),
                                "sources_used": _kb_ctx.get("sources_used", 0),
                            }
                            if _kb_ctx.get("warnings"):
                                logger.debug(
                                    "[auto-cycle] knowledge warnings for %s: %s",
                                    symbol, _kb_ctx["warnings"][:1],
                                )
                        except Exception as _kb_err:
                            logger.debug("[auto-cycle] knowledge_advisory skipped: %s", _kb_err)
                    logger.info("[auto-cycle] ENTRY %s symbol=%s action=%s blocked=%s score=%.1f",
                                side.upper(), symbol, entry_action, last_result["blocked"],
                                float(cand.get("selection_score") or 0))
                    # Event Store: registrar señal/orden para replay y auditoría
                    try:
                        _es = get_event_store()
                        _es_topic = "order.submitted" if not last_result["blocked"] else "signal.blocked"
                        _es.append(_es_topic, {
                            "symbol": symbol, "side": side, "action": entry_action,
                            "blocked": last_result["blocked"],
                            "selection_score": float(cand.get("selection_score") or 0),
                            "reasons": last_result.get("reasons", []),
                            "opportunity_id": baseline.get("opportunity_id"),
                            "trace_id": baseline.get("trace_id"),
                            "decision_id": baseline.get("decision_id"),
                            "cohort_id": final_decision.get("cohort_id"),
                            "policy_variant": final_decision.get("policy_variant"),
                        }, source="auto_cycle")
                    except Exception:
                        pass  # event store is non-critical
                    # FIX 2026-04-13: count executor_exception as consumed slot
                    # When executor raises (e.g. Tradier timeout), the order may have
                    # been sent. Counting as executed prevents runaway batch entries.
                    _executor_attempted = any(
                        str(r).startswith("Fallo operativo del ejecutor") or "executor_exception" in str(r)
                        for r in (last_result.get("reasons") or [])
                    )
                    if not last_result["blocked"] or _executor_attempted:
                        executed_count += 1
                        if executed_count >= effective_max_per_cycle:
                            logger.info(
                                "[auto-cycle] max_per_cycle=%d reached (attempted=%s) — stopping entry pass",
                                effective_max_per_cycle, _executor_attempted,
                            )
                            break

                _AUTO_CYCLE_STATE.update({
                    "cycle_count": _AUTO_CYCLE_STATE["cycle_count"] + 1,
                    "last_cycle_at": datetime.utcnow().isoformat(),
                    "last_candidate_symbol": sorted_cands[0].get("symbol") if sorted_cands else None,
                    "last_action": action,
                    "entry_action_last": entry_action,
                    "last_result": last_result,
                    "exits_sent_last_cycle": exits_sent,
                })
                # ── 4. RISK CHECK: factor correlation → NEXUS alert si >60% ────
                try:
                    _risk = await asyncio.to_thread(
                        get_analytics_engine().factor_correlation, "paper"
                    )
                    if _risk.get("risk_level") in ("CRITICAL", "WARNING"):
                        _risk_msg = (
                            f"⚠ ATLAS Risk Alert [{_risk['risk_level']}] — "
                            f"Factor concentration detected: "
                            + ", ".join(
                                f"{a['type']}={a.get('symbol') or a.get('strategy')} ({a['pct']}%)"
                                for a in _risk.get("alerts", [])
                            )
                        )
                        logger.warning("[auto-cycle] %s", _risk_msg)
                        get_event_store().append("risk.concentration", _risk, source="auto_cycle")
                        if _risk["risk_level"] == "CRITICAL":
                            await get_alert_dispatcher().system_error(
                                component="quant_core.risk_engine",
                                error=_risk_msg,
                                critical=True,
                            )
                except Exception as _risk_err:
                    logger.debug("[auto-cycle] risk check skipped: %s", _risk_err)

                _auto_cycle_mark_stage(
                    "cycle_complete",
                    action=entry_action,
                    configured_action=action,
                    candidates=len(sorted_cands),
                    exits_sent=exits_sent,
                    blocked=bool((last_result or {}).get("blocked")),
                )
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                _AUTO_CYCLE_STATE["error"] = str(exc)
                _AUTO_CYCLE_STATE["last_cycle_at"] = datetime.utcnow().isoformat()
                _auto_cycle_mark_stage("cycle_error", error=str(exc))
                logger.exception("[auto-cycle] Error en ciclo: %s", exc)
    except asyncio.CancelledError:
        pass
    finally:
        _AUTO_CYCLE_STATE.update({"running": False, "stopped_at": datetime.utcnow().isoformat()})
        _auto_cycle_mark_stage("stopped", cycles=_AUTO_CYCLE_STATE["cycle_count"])
        logger.info("[auto-cycle] Detenido. ciclos=%d", _AUTO_CYCLE_STATE["cycle_count"])


@app.post("/operation/loop/start", response_model=StdResponse, tags=["Operation"])
async def operation_loop_start(
    body: LoopStartRequest,
    x_api_key: str | None = Header(None),
):
    """Inicia el ciclo autónomo: toma candidatos del scanner y los envía a OperationCenter.
    En modo paper_supervised ejecuta preview (evalúa sin enviar orden).
    En modo paper_autonomous ejecuta submit (envía a paper trading).
    En modo paper_aggressive ejecuta submit con mayor cobertura por ciclo (paper-only).
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
    _AUTO_CYCLE_STATE["selector_session_mode_override"] = body.selector_session_mode
    _auto_cycle_task = asyncio.create_task(
        _auto_cycle_loop(body.interval_sec, body.max_per_cycle)
    )
    return StdResponse(
        ok=True,
        data={
            "started": True,
            "interval_sec": body.interval_sec,
            "max_per_cycle": body.max_per_cycle,
            "selector_session_mode": body.selector_session_mode or str(settings.selector_session_mode or "balanced").lower(),
        },
        ms=round((time.perf_counter() - t0) * 1000, 2),
    )


@app.post("/operation/loop/stop", response_model=StdResponse, tags=["Operation"])
async def operation_loop_stop(x_api_key: str | None = Header(None)):
    """Detiene el ciclo autónomo de evaluación."""
    _auth(x_api_key)
    global _auto_cycle_task
    t0 = time.perf_counter()
    _AUTO_CYCLE_STATE["running"] = False
    _AUTO_CYCLE_STATE["selector_session_mode_override"] = None
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
    op_config = _OPERATION_CENTER.get_config()
    state["configured_auton_mode"] = op_config.get("auton_mode")
    if not state["running"] and not state["task_alive"]:
        state["inactive_reasons"] = _auto_cycle_inactive_reasons(op_config)
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


@app.get("/camera/health", response_model=StdResponse, tags=["Operation"])
async def camera_health_endpoint(x_api_key: str | None = Header(None)):
    """Salud textual del sensor de visión/cámara (sin streaming de vídeo)."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        payload = await asyncio.to_thread(build_quant_camera_health_payload, _VISION)
        return StdResponse(ok=True, data=payload, ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as exc:
        logger.exception("camera/health error")
        return StdResponse(
            ok=False,
            error=str(exc),
            ms=round((time.perf_counter() - t0) * 1000, 2),
        )


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


def _operation_readiness_payload_fast() -> dict[str, object]:
    """Readiness liviano: sin diagnose() ni VisualPipeline; visión vía status_for_gate() (solo proveedor activo)."""
    payload = build_readiness_fast_payload(
        operation_center=_OPERATION_CENTER,
        vision_service=_VISION,
        st=settings,
    )
    payload["market_open_operational"] = _market_open_operational_config_snapshot()
    _attach_operational_self_audit_summary(payload)
    return payload


def _operation_readiness_payload_diagnostic() -> dict[str, object]:
    """Diagnóstico profundo: misma semántica ready + diagnose + pipeline OCR."""
    chart = _OPERATION_CENTER.chart_execution.status()
    vision_diag = _VISION.diagnose()
    vp_status: dict[str, object] = {}
    try:
        from atlas_code_quant.vision.visual_pipeline import VisualPipeline

        vp_status = VisualPipeline.get_instance().status()
    except Exception as exc:
        vp_status = {"error": str(exc)}
    auto_open = bool(getattr(settings, "chart_auto_open_enabled", False))
    warmup_on = bool(getattr(settings, "startup_chart_warmup_enabled", False))
    chart_plan_needed = auto_open or warmup_on
    probe_detail = ""
    if chart_plan_needed:
        symbols = list(getattr(settings, "startup_chart_warmup_symbols", []) or [])
        probe_sym = symbols[0] if symbols else "SPY"
        tf = str(getattr(settings, "startup_chart_warmup_timeframe", "1h") or "1h")
        pv = str(getattr(settings, "chart_provider_default", "tradingview") or "tradingview")
        chart_plan_buildable, probe_detail = chart_plan_probe_ok(probe_sym, tf, pv)
    else:
        chart_plan_buildable = True
    warm_ok = startup_warmup_gate_satisfied(
        chart,
        startup_chart_warmup_enabled=warmup_on,
        chart_auto_open_enabled=auto_open,
    )
    vp_semantic_ok = not (isinstance(vp_status, dict) and "error" in vp_status)
    ready, reasons = evaluate_operational_readiness(
        chart_execution=chart,
        vision_provider_ready=bool(vision_diag.get("provider_ready")),
        chart_auto_open_enabled=auto_open,
        chart_plan_buildable=chart_plan_buildable,
        startup_chart_warmup_enabled=warmup_on,
        startup_warmup_satisfied=warm_ok,
        visual_pipeline_ok=vp_semantic_ok,
    )
    payload: dict[str, object] = {
        "ready": ready,
        "reasons_not_ready": reasons,
        "readiness_mode": "diagnostic",
        "chart_execution": chart,
        "vision_status": None,
        "vision_diagnose": vision_diag,
        "chart_plan_probe": {
            "required": chart_plan_needed,
            "ok": chart_plan_buildable,
            "detail": probe_detail or None,
        },
        "startup_warmup_gate": {
            "required": warmup_on and auto_open,
            "satisfied": warm_ok,
        },
        "visual_pipeline": vp_status,
        "quant_flags": quant_readiness_flags(settings),
        "market_open_operational": _market_open_operational_config_snapshot(),
    }
    _attach_operational_self_audit_summary(payload)
    return payload


@app.get("/operation/readiness", response_model=StdResponse, tags=["Operation"])
async def operation_readiness_endpoint(x_api_key: str | None = Header(None)):
    """Gate operativo liviano: `data.ready` + `StdResponse.ok` reflejan listo/no listo (no solo HTTP 200)."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        payload = await asyncio.to_thread(_operation_readiness_payload_fast)
        ready = bool((payload or {}).get("ready"))
        return StdResponse(
            ok=ready,
            data=payload,
            ms=round((time.perf_counter() - t0) * 1000, 2),
        )
    except Exception as exc:
        logger.exception("Error en operation readiness")
        return StdResponse(ok=False, error=str(exc), ms=round((time.perf_counter() - t0) * 1000, 2))


@app.get("/operation/readiness/diagnostic", response_model=StdResponse, tags=["Operation"])
async def operation_readiness_diagnostic_endpoint(x_api_key: str | None = Header(None)):
    """Diagnóstico enriquecido (latencia mayor); mismo contrato semántico `ready` / `ok`."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        payload = await asyncio.to_thread(_operation_readiness_payload_diagnostic)
        ready = bool((payload or {}).get("ready"))
        return StdResponse(
            ok=ready,
            data=payload,
            ms=round((time.perf_counter() - t0) * 1000, 2),
        )
    except Exception as exc:
        logger.exception("Error en operation readiness diagnostic")
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


@app.get("/api/v2/quant/camera/health", response_model=StdResponse, tags=["V2"])
async def camera_health_v2(x_api_key: str | None = Header(None)):
    return await camera_health_endpoint(x_api_key=x_api_key)


@app.get("/api/v2/quant/operation/readiness", response_model=StdResponse, tags=["V2"])
async def operation_readiness_v2(x_api_key: str | None = Header(None)):
    return await operation_readiness_endpoint(x_api_key=x_api_key)


@app.get("/api/v2/quant/operation/readiness/diagnostic", response_model=StdResponse, tags=["V2"])
async def operation_readiness_diagnostic_v2(x_api_key: str | None = Header(None)):
    return await operation_readiness_diagnostic_endpoint(x_api_key=x_api_key)


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
    if not _legacy_scanner_enabled():
        return _legacy_scanner_gone_response(ms=(time.perf_counter() - t0) * 1000)
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
    if not _legacy_scanner_enabled():
        return _legacy_scanner_gone_response(ms=(time.perf_counter() - t0) * 1000)
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
    if not _legacy_scanner_enabled():
        return _legacy_scanner_gone_response(ms=(time.perf_counter() - t0) * 1000)
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
    if not _legacy_scanner_enabled():
        return _legacy_scanner_gone_response(ms=(time.perf_counter() - t0) * 1000)
    try:
        payload = ScannerReportPayload.model_validate(await _SCANNER.control(body.action))
        return StdResponse(ok=True, data=payload.model_dump(), ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as exc:
        logger.exception("Error controlling scanner")
        return StdResponse(ok=False, error=str(exc), ms=round((time.perf_counter() - t0) * 1000, 2))


def _enrich_universe_search_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Normaliza filas de búsqueda en modo compat (legacy)."""
    out: list[dict[str, Any]] = []
    for entry in rows:
        if not isinstance(entry, dict):
            continue
        sym = str(entry.get("symbol") or "").strip().upper()
        if not sym:
            continue
        out.append(
            {
                "symbol": sym,
                "name": str(entry.get("security_name") or entry.get("name") or ""),
                "exchange": str(entry.get("exchange") or ""),
                "is_etf": bool(entry.get("is_etf") or str(entry.get("asset_class") or "") == "equity_etf"),
                "asset_class": str(entry.get("asset_class") or "unknown"),
                "is_optionable": bool(entry.get("is_optionable", True)),
            }
        )
    return out


@app.get("/scanner/universe/search", response_model=StdResponse, tags=["Scanner"])
async def scanner_universe_search(
    q: str = "",
    limit: int = 25,
    x_api_key: str | None = Header(None),
):
    """Búsqueda sobre el universo US del catálogo del scanner (NASDAQ Trader + other listed)."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    if not _legacy_scanner_enabled():
        return _legacy_scanner_gone_response(ms=(time.perf_counter() - t0) * 1000)
    try:
        lim = max(1, min(int(limit), 100))
        intake = _RADAR_CLIENT.fetch_opportunities(limit=max(lim, _FLAGS.radar_intake_limit), min_score=0.0)
        matches = []
        needle = str(q or "").strip().upper()
        for opp in intake.batch.items:
            if needle and needle not in opp.symbol:
                continue
            matches.append(
                {
                    "symbol": opp.symbol,
                    "name": opp.asset_class,
                    "exchange": "",
                    "is_etf": opp.asset_class == "equity_etf",
                    "asset_class": opp.asset_class,
                    "is_optionable": True,
                }
            )
            if len(matches) >= lim:
                break
        raw = {"query": q, "truncated": len(matches) >= lim, "matches": matches}
        enriched_matches = _enrich_universe_search_rows(list(raw.get("matches") or []))
        payload = {
            **{k: v for k, v in raw.items() if k != "matches"},
            "matches": enriched_matches,
        }
        return StdResponse(ok=True, data=payload, ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as exc:
        logger.exception("Error en búsqueda de universo")
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
                options_session_mode=body.options_session_mode,
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


@app.get("/api/v2/quant/operation/status/lite", response_model=StdResponse, tags=["V2"])
async def operation_status_lite_v2(x_api_key: str | None = Header(None)):
    return await operation_status_lite(x_api_key=x_api_key)


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


@app.get("/api/v2/quant/scanner/universe/search", response_model=StdResponse, tags=["V2"])
async def scanner_universe_search_v2(
    q: str = "",
    limit: int = 25,
    x_api_key: str | None = Header(None),
):
    return await scanner_universe_search(q=q, limit=limit, x_api_key=x_api_key)


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
    allowed, client_key = await _register_ws_connection(websocket)
    if not allowed:
        await websocket.close(code=1013, reason="Too many dashboard connections")
        return
    account_scope = websocket.query_params.get("account_scope")
    account_id = websocket.query_params.get("account_id")
    await websocket.accept()
    try:
        while True:
            try:
                if _lightweight_startup_enabled():
                    await websocket.send_json(
                        _lightweight_live_update_payload(
                            account_scope=account_scope,
                            account_id=account_id,
                        )
                    )
                    await asyncio.sleep(_live_update_interval_sec())
                    continue
                canonical_snapshot_payload = await _cached_canonical_snapshot_payload(
                    account_scope=account_scope,
                    account_id=account_id,
                )
                await websocket.send_json(
                    {
                        "type": "quant.live_update",
                        "generated_at": datetime.utcnow().isoformat(),
                        "canonical_snapshot": _compact_live_canonical_snapshot(canonical_snapshot_payload),
                    }
                )
            except WebSocketDisconnect:
                return
            except Exception as exc:
                logger.exception("Error producing live update payload")
                if getattr(websocket, "application_state", None) == WebSocketState.DISCONNECTED:
                    return
                await websocket.send_json(
                    {
                        "type": "quant.live_error",
                        "generated_at": datetime.utcnow().isoformat(),
                        "error": str(exc),
                    }
                )
            await asyncio.sleep(_live_update_interval_sec())
    except WebSocketDisconnect:
        return
    finally:
        _unregister_ws_connection(websocket, client_key)


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
    allowed, client_key = await _register_ws_connection(websocket)
    if not allowed:
        await websocket.close(code=1013, reason="Too many dashboard connections")
        return
    await websocket.accept()
    try:
        while True:
            if _lightweight_startup_enabled():
                await websocket.send_json(_lightweight_operation_update_payload())
                await asyncio.sleep(_live_update_interval_sec())
                continue
            payload = _OPERATION_CENTER.status_lite()
            await websocket.send_json(
                {
                    "type": "quant.operation_update",
                    "generated_at": datetime.utcnow().isoformat(),
                    "operation_status": payload,
                }
            )
            await asyncio.sleep(_live_update_interval_sec())
    except WebSocketDisconnect:
        return
    except Exception:
        logger.exception("Operation websocket loop failed")
        try:
            await websocket.close(code=1011, reason="Operation stream failure")
        except RuntimeError:
            pass
    finally:
        _unregister_ws_connection(websocket, client_key)


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
    import math
    import statistics as _stats

    def _build_dashboard_payload() -> dict[str, object]:
        scope = account_scope or "paper"
        canonical_cache_key = _snapshot_cache_key("canonical", account_scope=scope, account_id=account_id)
        cached_canonical, _, _ = _snapshot_cache_get(canonical_cache_key)
        if isinstance(cached_canonical, dict):
            canonical_snapshot = cached_canonical
        else:
            try:
                canonical_snapshot = _canonical_snapshot_payload(account_scope=scope, account_id=account_id)
            except Exception:
                _snapshot_cache_start(
                    canonical_cache_key,
                    lambda: _canonical_snapshot_payload(account_scope=scope, account_id=account_id),
                )
                canonical_snapshot = {}
        canonical_balances = canonical_snapshot.get("balances") or {}
        canonical_totals = canonical_snapshot.get("totals") or {}

        chart_data = _JOURNAL.chart_data(account_scope=scope, limit=500)
        chart_trades = list(chart_data.get("trades") or [])
        equity_curve = _rebased_recent_equity_curve(
            chart_trades,
            canonical_balances=canonical_balances,
            canonical_totals=canonical_totals,
        )
        drawdown_curve = _drawdown_curve_from_equity(equity_curve)

        trade_pnls = [
            pnl_value
            for pnl_value in (
                float(trade.get("pnl", 0) or 0)
                for trade in chart_trades
            )
            if math.isfinite(pnl_value)
        ]
        signal_threshold = 1e-9
        informative_trade_pnls = [pnl for pnl in trade_pnls if abs(pnl) > signal_threshold]
        has_statistical_signal = len(informative_trade_pnls) >= 2
        if not has_statistical_signal:
            # Evita curvas/plots engañosos cuando el journal solo aporta cierres flat (pnl=0).
            equity_curve = []
            drawdown_curve = []
        wins = [pnl for pnl in informative_trade_pnls if pnl > 0]
        losses = [pnl for pnl in informative_trade_pnls if pnl < 0]
        realized_pnl = float(sum(informative_trade_pnls))
        profit_factor = round(sum(wins) / abs(sum(losses)), 4) if losses else None
        expectancy = round(realized_pnl / len(informative_trade_pnls), 4) if informative_trade_pnls else None
        win_rate_pct = round(len(wins) / len(informative_trade_pnls) * 100.0, 2) if informative_trade_pnls else None
        sharpe_ratio = None
        if len(informative_trade_pnls) >= 2:
            sigma = _stats.stdev(informative_trade_pnls)
            if sigma > 1e-9:
                sharpe_ratio = round((_stats.mean(informative_trade_pnls) / sigma) * math.sqrt(len(informative_trade_pnls)), 4)
        max_dd = min((pt["value"] for pt in drawdown_curve), default=0)
        calmar = round(realized_pnl / abs(max_dd), 3) if max_dd < -0.001 else None

        heatmap_buckets: dict[tuple[int, int], dict[str, int]] = {}
        for trade in chart_trades:
            pnl_value = float(trade.get("pnl", 0) or 0)
            if abs(pnl_value) <= signal_threshold:
                continue
            ts = trade.get("exit_time")
            if not ts:
                continue
            try:
                import dateutil.parser as _dp
                dt = _dp.parse(ts)
            except Exception:
                continue
            key = (int(dt.weekday()), int(dt.hour))
            bucket = heatmap_buckets.setdefault(key, {"trades": 0, "wins": 0})
            bucket["trades"] += 1
            if pnl_value > 0:
                bucket["wins"] += 1
        heatmap = [
            {
                "weekday": weekday,
                "hour": hour,
                "trades": bucket["trades"],
                "wins": bucket["wins"],
                "success_rate_pct": round(bucket["wins"] / bucket["trades"] * 100.0, 2) if bucket["trades"] else 0.0,
            }
            for (weekday, hour), bucket in sorted(heatmap_buckets.items())
        ]

        mc_data = None
        if len(informative_trade_pnls) >= 5:
            import random as _rand
            _rand.seed(42)
            n_paths, n_steps = 200, min(50, len(informative_trade_pnls))
            paths = []
            for _ in range(n_paths):
                sample = _rand.choices(informative_trade_pnls, k=n_steps)
                cumulative = 0.0
                path = []
                for pnl in sample:
                    cumulative += pnl
                    path.append(round(cumulative, 2))
                paths.append(path)
            transposed = list(zip(*paths))
            labels = list(range(1, n_steps + 1))
            median = [round(_stats.median(step), 2) for step in transposed]
            p05 = [round(sorted(step)[int(len(step) * 0.05)], 2) for step in transposed]
            p95 = [round(sorted(step)[int(len(step) * 0.95)], 2) for step in transposed]
            mc_var_95 = p05[-1] if p05 else None
            terminal_values = [path[-1] for path in paths]
            mc_cvar_95 = (
                round(
                    sum(value for value in terminal_values if value <= (mc_var_95 or 0))
                    / max(1, sum(1 for value in terminal_values if value <= (mc_var_95 or 0))),
                    2,
                )
                if mc_var_95 is not None
                else None
            )
            mc_prob_profit = round(sum(1 for path in paths if path[-1] > 0) / n_paths * 100, 1)
            mc_data = {
                "labels": labels,
                "median": median,
                "p05": p05,
                "p95": p95,
                "var_95": mc_var_95,
                "cvar_95": mc_cvar_95,
                "prob_profit": mc_prob_profit,
            }

        payload = {
            "equity": round(float(canonical_balances.get("total_equity") or 0.0), 2) if canonical_balances.get("total_equity") is not None else None,
            "realized_pnl": round(realized_pnl, 2),
            "unrealized_pnl": round(float(canonical_totals.get("open_pnl") or 0), 2),
            "sharpe_ratio": sharpe_ratio,
            "win_rate_pct": win_rate_pct,
            "profit_factor": profit_factor,
            "expectancy": expectancy,
            "total_trades": len(informative_trade_pnls),
            "raw_total_trades": len(chart_trades),
            "open_positions": int(canonical_totals.get("positions") or 0),
            "max_drawdown_pct": round(max_dd, 3) if drawdown_curve else None,
            "calmar_ratio": calmar,
            "equity_curve": equity_curve,
            "drawdown_curve": drawdown_curve,
            "trade_pnls": informative_trade_pnls,
            "monte_carlo": mc_data,
            "recent_trades": [{"pnl": pnl} for pnl in informative_trade_pnls[-30:]],
            "heatmap": heatmap,
            "daily_pnl_pct": None,
            "stats_signal_ok": has_statistical_signal,
            "source": canonical_snapshot.get("source"),
            "source_label": canonical_snapshot.get("source_label"),
            "account_scope": canonical_snapshot.get("account_scope"),
            "account_id": canonical_snapshot.get("account_id"),
            "balances": canonical_balances,
            "totals": canonical_totals,
            "reconciliation": canonical_snapshot.get("reconciliation"),
            "simulators": canonical_snapshot.get("simulators"),
            "historical_source": {
                "label": f"Journal {scope} reciente",
                "kind": "journal_recent",
                "curve_basis": "rebased_recent_closed_trades",
            },
            "position_management": canonical_snapshot.get("position_management") or {"summary": {}, "watchlist": [], "alerts": []},
            "exit_governance": canonical_snapshot.get("exit_governance") or {"summary": {}, "recommendations": [], "alerts": []},
        }
        return payload

    try:
        payload = await asyncio.to_thread(_build_dashboard_payload)
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


# ── Briefing operativo (notificaciones inteligentes) ──────────────────────────

@app.get("/notifications/status", response_model=StdResponse, tags=["Notifications"])
async def notifications_status_endpoint(x_api_key: str | None = Header(None)):
    """Estado del módulo de briefing: flags, canales, snapshots recientes."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        from notifications.briefing_service import get_operational_briefing_service

        data = get_operational_briefing_service().status()
        return StdResponse(ok=True, data=data, ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as e:
        return StdResponse(ok=False, error=str(e), ms=round((time.perf_counter() - t0) * 1000, 2))


@app.post("/notifications/premarket/run", response_model=StdResponse, tags=["Notifications"])
async def notifications_premarket_run(x_api_key: str | None = Header(None)):
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        from notifications.briefing_service import get_operational_briefing_service

        out = await get_operational_briefing_service().run_premarket()
        return StdResponse(ok=True, data=out, ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as e:
        logger.exception("notifications premarket")
        return StdResponse(ok=False, error=str(e), ms=round((time.perf_counter() - t0) * 1000, 2))


@app.post("/notifications/eod/run", response_model=StdResponse, tags=["Notifications"])
async def notifications_eod_run(x_api_key: str | None = Header(None)):
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        from notifications.briefing_service import get_operational_briefing_service

        out = await get_operational_briefing_service().run_eod()
        return StdResponse(ok=True, data=out, ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as e:
        logger.exception("notifications eod")
        return StdResponse(ok=False, error=str(e), ms=round((time.perf_counter() - t0) * 1000, 2))


@app.post("/notifications/test", response_model=StdResponse, tags=["Notifications"])
async def notifications_test(
    message: str = "Test briefing operativo",
    x_api_key: str | None = Header(None),
):
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        from notifications.briefing_service import get_operational_briefing_service

        out = await get_operational_briefing_service().run_test(message)
        return StdResponse(ok=True, data=out, ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as e:
        return StdResponse(ok=False, error=str(e), ms=round((time.perf_counter() - t0) * 1000, 2))


@app.get("/notifications/recent", response_model=StdResponse, tags=["Notifications"])
async def notifications_recent(limit: int = 30, x_api_key: str | None = Header(None)):
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        from notifications.briefing_service import get_operational_briefing_service

        svc = get_operational_briefing_service()
        data = {
            "snapshots": svc._store.list_recent_snapshots(limit=min(limit, 50)),
            "events": svc._store.tail_events(limit=min(limit, 200)),
        }
        return StdResponse(ok=True, data=data, ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as e:
        return StdResponse(ok=False, error=str(e), ms=round((time.perf_counter() - t0) * 1000, 2))


@app.get("/api/v2/quant/notifications/status", response_model=StdResponse, tags=["V2"])
async def notifications_status_v2(x_api_key: str | None = Header(None)):
    return await notifications_status_endpoint(x_api_key=x_api_key)


@app.post("/api/v2/quant/notifications/premarket/run", response_model=StdResponse, tags=["V2"])
async def notifications_premarket_v2(x_api_key: str | None = Header(None)):
    return await notifications_premarket_run(x_api_key=x_api_key)


@app.post("/api/v2/quant/notifications/eod/run", response_model=StdResponse, tags=["V2"])
async def notifications_eod_v2(x_api_key: str | None = Header(None)):
    return await notifications_eod_run(x_api_key=x_api_key)


@app.post("/api/v2/quant/notifications/test", response_model=StdResponse, tags=["V2"])
async def notifications_test_v2(
    message: str = "Test briefing operativo",
    x_api_key: str | None = Header(None),
):
    return await notifications_test(message=message, x_api_key=x_api_key)


@app.get("/api/v2/quant/notifications/recent", response_model=StdResponse, tags=["V2"])
async def notifications_recent_v2(limit: int = 30, x_api_key: str | None = Header(None)):
    return await notifications_recent(limit=limit, x_api_key=x_api_key)


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


# ═══════════════════════════════════════════════════════════════════════════════
# KNOWLEDGE BASE — biblioteca académica consultable para toma de decisiones
# ═══════════════════════════════════════════════════════════════════════════════

@app.get("/knowledge/query", response_model=StdResponse, tags=["Knowledge"])
async def knowledge_query(
    method: str | None = None,
    timeframe: str | None = None,
    topic: str | None = None,
    max_sources: int = 5,
    x_api_key: str | None = Header(None),
):
    """Consulta la biblioteca académica de ATLAS-Quant.

    Retorna fuentes relevantes, hallazgos clave, warnings y recomendaciones
    operativas basadas en evidencia académica.

    Args:
        method: Método del scanner (trend_ema_stack, breakout_donchian, etc.)
        timeframe: Temporalidad (1d, swing, intraday, short, medium, long)
        topic: Tema (momentum, mean_reversion, microstructure, options, etc.)
        max_sources: Máximo de fuentes a incluir (default 5)
    """
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        kb = get_knowledge_base()
        ctx = kb.advisory_context(
            method=method,
            timeframe=timeframe,
            topic=topic,
            max_sources=max_sources,
        )
        return StdResponse(ok=True, data=ctx, ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as exc:
        return StdResponse(ok=False, error=str(exc), ms=round((time.perf_counter() - t0) * 1000, 2))


@app.get("/knowledge/sources", response_model=StdResponse, tags=["Knowledge"])
async def knowledge_sources(
    category: str | None = None,
    confidence: str | None = None,
    x_api_key: str | None = Header(None),
):
    """Lista las fuentes del catálogo académico.

    Args:
        category: Filtrar por categoría (CAT1-CAT5)
        confidence: Filtrar por nivel mínimo de confianza (high, medium, low)
    """
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        kb = get_knowledge_base()
        sources = kb.sources_by_confidence(min_level=confidence or "low")
        if category:
            sources = [s for s in sources if str(s.get("category", "")) == category.upper()]
        compact = [
            {
                "id": s["id"],
                "title": s["title"],
                "authors": s.get("authors", []),
                "year": s.get("year"),
                "category": s.get("category"),
                "topic": s.get("topic"),
                "confidence_level": s.get("confidence_level"),
                "timeframes": s.get("timeframes", []),
                "scanner_methods": s.get("scanner_methods", []),
                "tags": s.get("tags", []),
            }
            for s in sources
        ]
        return StdResponse(ok=True, data={"sources": compact, "total": len(compact)},
                           ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as exc:
        return StdResponse(ok=False, error=str(exc), ms=round((time.perf_counter() - t0) * 1000, 2))


@app.get("/knowledge/summary", response_model=StdResponse, tags=["Knowledge"])
async def knowledge_summary(x_api_key: str | None = Header(None)):
    """Resumen estadístico de la biblioteca académica."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        kb = get_knowledge_base()
        return StdResponse(ok=True, data=kb.summary(),
                           ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as exc:
        return StdResponse(ok=False, error=str(exc), ms=round((time.perf_counter() - t0) * 1000, 2))


@app.get("/knowledge/source/{source_id}", response_model=StdResponse, tags=["Knowledge"])
async def knowledge_source_detail(source_id: str, x_api_key: str | None = Header(None)):
    """Ficha completa de una referencia por ID."""
    _auth(x_api_key)
    t0 = time.perf_counter()
    try:
        kb = get_knowledge_base()
        src = kb.source_by_id(source_id)
        if src is None:
            return StdResponse(ok=False, error=f"Source '{source_id}' not found",
                               ms=round((time.perf_counter() - t0) * 1000, 2))
        return StdResponse(ok=True, data=src, ms=round((time.perf_counter() - t0) * 1000, 2))
    except Exception as exc:
        return StdResponse(ok=False, error=str(exc), ms=round((time.perf_counter() - t0) * 1000, 2))


# ── V2 aliases Knowledge ──────────────────────────────────────────────────────
@app.get("/api/v2/quant/knowledge/query",   response_model=StdResponse, tags=["V2"])
async def knowledge_query_v2(method: str | None = None, timeframe: str | None = None,
                               topic: str | None = None, max_sources: int = 5,
                               x_api_key: str | None = Header(None)):
    return await knowledge_query(method=method, timeframe=timeframe, topic=topic,
                                  max_sources=max_sources, x_api_key=x_api_key)

@app.get("/api/v2/quant/knowledge/sources", response_model=StdResponse, tags=["V2"])
async def knowledge_sources_v2(category: str | None = None, confidence: str | None = None,
                                 x_api_key: str | None = Header(None)):
    return await knowledge_sources(category=category, confidence=confidence, x_api_key=x_api_key)

@app.get("/api/v2/quant/knowledge/summary", response_model=StdResponse, tags=["V2"])
async def knowledge_summary_v2(x_api_key: str | None = Header(None)):
    return await knowledge_summary(x_api_key=x_api_key)


@app.get("/knowledge/search", response_model=StdResponse, tags=["Knowledge"])
async def knowledge_search(
    q: str,
    max_results: int = 10,
    x_api_key: str | None = Header(None),
):
    """Búsqueda full-text sobre título, hallazgos, utility, tags y subtopics."""
    t0 = _now_ms()
    try:
        kb = get_knowledge_base()
        results = kb.search(q, max_results=max_results)
        return _std_resp(True, {
            "query": q,
            "max_results": max_results,
            "results_found": len(results),
            "results": results,
        }, _now_ms() - t0)
    except Exception as e:
        logger.exception("[knowledge/search] error")
        return _std_resp(False, None, _now_ms() - t0, str(e))


@app.post("/knowledge/ingest", response_model=StdResponse, tags=["Knowledge"])
async def knowledge_ingest(
    generate_fichas: bool = True,
    download_papers: bool = False,
    source_ids: list[str] | None = None,
    x_api_key: str | None = Header(None),
):
    """Genera fichas JSON y opcionalmente descarga papers open-access."""
    t0 = _now_ms()
    try:
        from knowledge.ingest_pipeline import (
            generate_fichas as _gen_fichas,
            ingest_open_access as _ingest_oa,
            ingest_status as _status,
        )
        result: dict = {}
        if generate_fichas:
            result["fichas"] = _gen_fichas()
        if download_papers:
            result["open_access"] = _ingest_oa(source_ids=source_ids)
        result["status"] = _status()
        return _std_resp(True, result, _now_ms() - t0)
    except Exception as e:
        logger.exception("[knowledge/ingest] error")
        return _std_resp(False, None, _now_ms() - t0, str(e))


@app.get("/knowledge/ingest/status", response_model=StdResponse, tags=["Knowledge"])
async def knowledge_ingest_status(x_api_key: str | None = Header(None)):
    """Estado del pipeline de ingest: fichas generadas, papers descargados."""
    t0 = _now_ms()
    try:
        from knowledge.ingest_pipeline import ingest_status as _status
        return _std_resp(True, _status(), _now_ms() - t0)
    except Exception as e:
        logger.exception("[knowledge/ingest/status] error")
        return _std_resp(False, None, _now_ms() - t0, str(e))


@app.get("/api/v2/quant/knowledge/search", response_model=StdResponse, tags=["V2"])
async def knowledge_search_v2(q: str, max_results: int = 10,
                               x_api_key: str | None = Header(None)):
    return await knowledge_search(q=q, max_results=max_results, x_api_key=x_api_key)


@app.post("/api/v2/quant/knowledge/ingest", response_model=StdResponse, tags=["V2"])
async def knowledge_ingest_v2(
    generate_fichas: bool = True,
    download_papers: bool = False,
    source_ids: list[str] | None = None,
    x_api_key: str | None = Header(None),
):
    return await knowledge_ingest(
        generate_fichas=generate_fichas,
        download_papers=download_papers,
        source_ids=source_ids,
        x_api_key=x_api_key,
    )


@app.get("/api/v2/quant/knowledge/ingest/status", response_model=StdResponse, tags=["V2"])
async def knowledge_ingest_status_v2(x_api_key: str | None = Header(None)):
    return await knowledge_ingest_status(x_api_key=x_api_key)


# ── Learning Orchestrator — endpoints ─────────────────────────────────────────

@app.get("/learning/orchestrator/status", response_model=StdResponse, tags=["Learning"])
async def learning_orchestrator_status(x_api_key: str | None = Header(None)):
    """Estado del loop de aprendizaje: reconciliaciones, IC updates, políticas."""
    t0 = _now_ms()
    return _std_resp(True, get_orchestrator_status(), _now_ms() - t0)


@app.post("/learning/orchestrator/reconcile", response_model=StdResponse, tags=["Learning"])
async def learning_orchestrator_reconcile(x_api_key: str | None = Header(None)):
    """Fuerza una reconciliación inmediata de posiciones cerradas."""
    t0 = _now_ms()
    try:
        result = await reconcile_closed_positions()
        if result.get("ic_updated", 0) > 0:
            ic_result = await asyncio.to_thread(apply_ic_to_policy)
            result["ic_policy"] = ic_result
        return _std_resp(True, result, _now_ms() - t0)
    except Exception as e:
        logger.exception("[learning/reconcile] error")
        return _std_resp(False, None, _now_ms() - t0, str(e))


@app.post("/learning/orchestrator/daily-analysis", response_model=StdResponse, tags=["Learning"])
async def learning_daily_analysis(x_api_key: str | None = Header(None)):
    """Fuerza el análisis diario completo: métricas, patrones, readiness."""
    t0 = _now_ms()
    try:
        result = await _run_daily_analysis()
        return _std_resp(True, result, _now_ms() - t0)
    except Exception as e:
        logger.exception("[learning/daily-analysis] error")
        return _std_resp(False, None, _now_ms() - t0, str(e))


@app.get("/learning/ic/summary", response_model=StdResponse, tags=["Learning"])
async def learning_ic_summary(method: str | None = None,
                               x_api_key: str | None = Header(None)):
    """IC actual por método. Base científica del sizing y filtros del scanner."""
    _auth(x_api_key)
    t0 = _now_ms()
    try:
        tracker = get_ic_tracker()
        if method:
            data = tracker.compute_ic(method=method)
        else:
            data = tracker.summary()
        return _std_resp(True, data, _now_ms() - t0)
    except Exception as e:
        logger.exception("[learning/ic/summary] error")
        return _std_resp(False, None, _now_ms() - t0, str(e))


@app.get("/api/v2/quant/learning/orchestrator/status", response_model=StdResponse, tags=["V2"])
async def learning_orch_status_v2(x_api_key: str | None = Header(None)):
    return await learning_orchestrator_status(x_api_key=x_api_key)


@app.post("/api/v2/quant/learning/orchestrator/reconcile", response_model=StdResponse, tags=["V2"])
async def learning_orch_reconcile_v2(x_api_key: str | None = Header(None)):
    return await learning_orchestrator_reconcile(x_api_key=x_api_key)


@app.post("/api/v2/quant/learning/orchestrator/daily-analysis", response_model=StdResponse, tags=["V2"])
async def learning_daily_analysis_v2(x_api_key: str | None = Header(None)):
    return await learning_daily_analysis(x_api_key=x_api_key)


@app.get("/api/v2/quant/learning/ic/summary", response_model=StdResponse, tags=["V2"])
async def learning_ic_summary_v2(method: str | None = None,
                                  x_api_key: str | None = Header(None)):
    return await learning_ic_summary(method=method, x_api_key=x_api_key)


# ══════════════════════════════════════════════════════════════════════════════
# DuckDB Analytics, Event Store & Strategy Evolution
# ══════════════════════════════════════════════════════════════════════════════

@app.get("/journal/analytics", response_model=StdResponse, tags=["Journal"])
async def journal_analytics(account_scope: str = "paper",
                            x_api_key: str | None = Header(None)):
    """Full analytics bundle via DuckDB: equity curve, daily PnL, R-distribution,
    strategy performance, and factor correlation risk alerts."""
    _auth(x_api_key)
    t0 = _now_ms()
    try:
        data = await asyncio.to_thread(get_analytics_engine().full_analytics, account_scope)
        return _std_resp(True, data, _now_ms() - t0)
    except Exception as e:
        logger.exception("[journal/analytics] error")
        return _std_resp(False, None, _now_ms() - t0, str(e))


@app.get("/journal/analytics/risk", response_model=StdResponse, tags=["Journal"])
async def journal_analytics_risk(account_scope: str = "paper",
                                  x_api_key: str | None = Header(None)):
    """Factor correlation risk check — detects symbol/strategy concentration >60%."""
    _auth(x_api_key)
    t0 = _now_ms()
    try:
        data = await asyncio.to_thread(get_analytics_engine().factor_correlation, account_scope)
        return _std_resp(True, data, _now_ms() - t0)
    except Exception as e:
        logger.exception("[journal/analytics/risk] error")
        return _std_resp(False, None, _now_ms() - t0, str(e))


@app.get("/events/stats", response_model=StdResponse, tags=["Learning"])
async def event_store_stats(x_api_key: str | None = Header(None)):
    """Event store statistics: buffer size, topic counts, disk usage."""
    _auth(x_api_key)
    t0 = _now_ms()
    try:
        data = get_event_store().stats()
        return _std_resp(True, data, _now_ms() - t0)
    except Exception as e:
        return _std_resp(False, None, _now_ms() - t0, str(e))


@app.get("/events/query", response_model=StdResponse, tags=["Learning"])
async def event_store_query(topic: str | None = None, source: str | None = None,
                            since: str | None = None, limit: int = 100,
                            x_api_key: str | None = Header(None)):
    """Query events from the in-memory event store."""
    _auth(x_api_key)
    t0 = _now_ms()
    try:
        data = get_event_store().query(topic=topic, source=source, since=since, limit=limit)
        return _std_resp(True, {"count": len(data), "events": data}, _now_ms() - t0)
    except Exception as e:
        return _std_resp(False, None, _now_ms() - t0, str(e))


@app.post("/evolution/run", response_model=StdResponse, tags=["Learning"])
async def evolution_run(symbols: str = "SPY,QQQ,AAPL,MSFT,NVDA",
                        generations: int = 20,
                        population: int = 30,
                        period: str = "3mo",
                        x_api_key: str | None = Header(None)):
    """Launch a strategy evolution run using Genetic Programming + vectorized backtesting.

    Downloads OHLCV data via yfinance and runs genetic evolution for each symbol.
    Args:
        symbols: Comma-separated list of symbols.
        generations: Number of evolutionary generations (max 50).
        population: Population size per generation (max 100).
        period: yfinance period string (e.g. "3mo", "6mo", "1y").
    """
    _auth(x_api_key)
    t0 = _now_ms()
    try:
        import yfinance as yf
        sym_list = [s.strip().upper() for s in symbols.split(",") if s.strip()]
        gens = min(generations, 50)
        pop = min(population, 100)
        all_results: list[dict] = []
        evolver = StrategyEvolver()

        for sym in sym_list:
            logger.info("[evolution/run] Downloading %s period=%s", sym, period)
            ticker = yf.Ticker(sym)
            df = await asyncio.to_thread(ticker.history, period=period, interval="1d")
            if df is None or len(df) < 100:
                all_results.append({"symbol": sym, "error": f"Insufficient data ({len(df) if df is not None else 0} bars, need 100+)"})
                continue
            # Normalize column names to lowercase
            df.columns = [c.lower() for c in df.columns]
            result = await asyncio.to_thread(
                evolver.evolve, ohlcv=df, symbol=sym,
                generations=gens, population_size=pop,
            )
            all_results.append({
                "symbol": sym,
                "bars": len(df),
                "generations": gens,
                "total_backtests": result.total_backtests,
                "duration_sec": result.duration_sec,
                "passing_count": len(result.passing_genomes),
                "best_genomes": result.best_genomes[:5],
                "generation_stats": result.generation_stats[-3:],  # last 3 gen stats
            })

        get_event_store().append("evolution.completed", {
            "symbols": sym_list,
            "generations": gens,
            "period": period,
            "results_count": len(all_results),
        }, source="strategy_evolver")
        return _std_resp(True, {"symbols": sym_list, "period": period, "results": all_results}, _now_ms() - t0)
    except Exception as e:
        logger.exception("[evolution/run] error")
        return _std_resp(False, None, _now_ms() - t0, str(e))


@app.get("/evolution/results", response_model=StdResponse, tags=["Learning"])
async def evolution_results(x_api_key: str | None = Header(None)):
    """List saved evolution results from data/learning/evolution/."""
    _auth(x_api_key)
    t0 = _now_ms()
    try:
        from pathlib import Path
        evo_dir = Path(settings.data_dir).parent / "learning" / "evolution"
        if not evo_dir.exists():
            return _std_resp(True, {"results": []}, _now_ms() - t0)
        import json
        results = []
        for f in sorted(evo_dir.glob("*.json"), key=lambda p: p.stat().st_mtime, reverse=True)[:20]:
            try:
                data = json.loads(f.read_text(encoding="utf-8"))
                gen_stats = data.get("generation_stats", [])
                best = data.get("best_genomes", [{}])[0] if data.get("best_genomes") else {}
                results.append({"file": f.name, "summary": {
                    "symbol": data.get("symbol", ""),
                    "generations": data.get("generations", 0),
                    "population_size": data.get("population_size", 0),
                    "total_backtests": data.get("total_backtests", 0),
                    "passing_count": data.get("passing_count", 0),
                    "duration_sec": data.get("duration_sec", 0),
                    "best_fitness": best.get("fitness", 0),
                    "best_sharpe": best.get("sharpe", 0),
                    "best_ic": best.get("ic", 0),
                    "best_profit_factor": best.get("profit_factor", 0),
                    "best_win_rate": best.get("win_rate", 0),
                    "last_gen_stats": gen_stats[-1] if gen_stats else {},
                }})
            except Exception:
                pass
        return _std_resp(True, {"results": results}, _now_ms() - t0)
    except Exception as e:
        return _std_resp(False, None, _now_ms() - t0, str(e))

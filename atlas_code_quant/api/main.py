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

import time
import logging
from datetime import datetime

from fastapi import FastAPI, HTTPException, Header
from fastapi.middleware.cors import CORSMiddleware

from api.schemas import (
    EvalSignalRequest, ActivateStrategyRequest, OrderRequest,
    SignalResponse, PortfolioResponse, HealthResponse, StdResponse,
    StatusEnum, SignalEnum,
)
from config.settings import settings

logger = logging.getLogger("quant.api")
_START_TIME = time.time()

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

# ── Registry global (se popula al iniciar el motor) ──────────────────────────
_strategies: dict = {}
_portfolio = None


def _auth(x_api_key: str | None) -> None:
    if x_api_key != settings.api_key:
        raise HTTPException(status_code=401, detail="API key inválida")


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
        return StdResponse(
            ok=True,
            data=signal.to_dict(),
            ms=round((time.perf_counter() - t0) * 1000, 2),
        )
    except Exception as e:
        logger.exception("Error evaluando señal")
        return StdResponse(ok=False, error=str(e))


@app.post("/order", response_model=StdResponse, tags=["Trading"])
async def place_order(body: OrderRequest, x_api_key: str | None = Header(None)):
    """Coloca una orden de trading (paper o live según configuración)."""
    _auth(x_api_key)
    if not settings.paper_trading:
        raise HTTPException(status_code=403, detail="Live trading no habilitado")
    if not _portfolio:
        return StdResponse(ok=False, error="Portfolio no inicializado")
    if body.side == "buy":
        pos = _portfolio.open_position(
            body.symbol, "long", body.size, body.price or 0,
            body.stop_loss, body.take_profit,
        )
        return StdResponse(ok=pos is not None, data=pos.to_dict() if pos else None)
    pnl = _portfolio.close_position(body.symbol, body.price or 0)
    return StdResponse(ok=True, data={"pnl": pnl})


@app.get("/positions", response_model=StdResponse, tags=["Trading"])
async def get_positions(x_api_key: str | None = Header(None)):
    """Retorna posiciones abiertas y resumen del portfolio."""
    _auth(x_api_key)
    if not _portfolio:
        return StdResponse(ok=False, error="Portfolio no inicializado")
    return StdResponse(ok=True, data=_portfolio.summary())


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

"""
main.py — Punto de entrada del módulo Radar Kalshi.

Modos de uso
------------

1. **Stand-alone** (proceso aislado, sin Atlas Core):

   .. code-block:: bash

      python -m modules.atlas_radar_kalshi.main --serve

   Levanta uvicorn en ``0.0.0.0:8792`` con la API + UI del módulo.
   El dashboard central de atlas-core (``:8791``) puede embeber esta
   URL o usar el cross-link ``/api/radar/atlas-link``.

2. **Integrado en atlas-core (recomendado)**:

   En ``atlas_adapter/atlas_http_api.py`` (o donde se monte la
   ``FastAPI app``)::

       from modules.atlas_radar_kalshi.main import register
       register(app)

   con eso el radar queda colgado en ``:8791/ui/radar`` y el bus de
   WebSocket en ``:8791/api/radar/stream``.

3. **Solo loop de trading** (sin servidor HTTP): ``python -m
   modules.atlas_radar_kalshi.main --loop``.
"""
from __future__ import annotations

import argparse
import asyncio
import logging
import os
from pathlib import Path
from typing import Optional

from .brain import RadarBrain
from .config import get_settings, reload_settings
from .dashboard import RadarState, build_router
from .executor import KalshiExecutor
from .risk import KellyRiskManager
from .scanner import KalshiScanner, MarketEvent
from .utils.logger import get_logger


# Runtime lock para evitar doble orquestador cuando hay procesos duplicados
# del adapter en Windows (uno con puerto y otro "zombie" sin bind).
_RUNTIME_LOCK_PATH: Optional[Path] = None
_RUNTIME_LOCK_OWNED: bool = False


def _acquire_runtime_lock(lock_path: Path) -> bool:
    global _RUNTIME_LOCK_PATH, _RUNTIME_LOCK_OWNED
    if _RUNTIME_LOCK_OWNED:
        return True

    lock_path.parent.mkdir(parents=True, exist_ok=True)

    # Reclamación de lock stale (archivo con PID inexistente).
    if lock_path.exists():
        try:
            existing_pid = int(lock_path.read_text(encoding="utf-8").strip() or "0")
        except Exception:
            existing_pid = 0
        stale = True
        if existing_pid > 0:
            try:
                os.kill(existing_pid, 0)
                stale = False
            except Exception:
                stale = True
        if stale:
            try:
                lock_path.unlink(missing_ok=True)
            except Exception:
                pass

    try:
        fd = os.open(str(lock_path), os.O_CREAT | os.O_EXCL | os.O_WRONLY)
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write(str(os.getpid()))
    except FileExistsError:
        return False
    except Exception:
        return False

    _RUNTIME_LOCK_PATH = lock_path
    _RUNTIME_LOCK_OWNED = True
    return True


def _release_runtime_lock() -> None:
    global _RUNTIME_LOCK_PATH, _RUNTIME_LOCK_OWNED
    if not _RUNTIME_LOCK_OWNED:
        return

    lock_path = _RUNTIME_LOCK_PATH
    _RUNTIME_LOCK_PATH = None
    _RUNTIME_LOCK_OWNED = False
    if lock_path is None:
        return
    try:
        if lock_path.exists():
            pid_txt = lock_path.read_text(encoding="utf-8").strip()
            if pid_txt == str(os.getpid()):
                lock_path.unlink(missing_ok=True)
    except Exception:
        try:
            lock_path.unlink(missing_ok=True)
        except Exception:
            pass


# ===========================================================================
# Pipeline (loop principal)
# ===========================================================================
async def trading_loop(state: Optional[RadarState] = None) -> None:
    """
    Coreografía:
      scanner -> brain -> risk -> executor
    Publicando eventos al :class:`RadarState` (consumido por el dashboard).
    """
    settings = get_settings()
    log = get_logger("main", settings.log_dir, settings.log_level)
    state = state or RadarState(settings)
    state.session_started = asyncio.get_event_loop().time()

    scanner = KalshiScanner(settings)
    brain = RadarBrain(settings)
    risk = KellyRiskManager(settings)
    executor = KalshiExecutor(settings)

    # Saldo inicial (no bloqueante: si falla, seguimos en modo paper)
    try:
        state.balance_cents = await executor.balance_cents()
        log.info("Balance inicial: %.2f USD", state.balance_cents / 100)
    except Exception as exc:
        log.warning("Sin balance (modo paper): %s", exc)

    async for ev in scanner.stream():
        try:
            await _handle_event(ev, state, scanner, brain, risk, executor, log)
        except Exception as exc:
            log.exception("Error procesando evento %s: %s", ev.kind, exc)
            await state.broadcast({"type": "error", "error": str(exc)})


async def _handle_event(
    ev: MarketEvent, state: RadarState,
    scanner: KalshiScanner, brain: RadarBrain,
    risk: KellyRiskManager, executor: KalshiExecutor, log,
) -> None:
    if ev.kind == "new_market":
        brain.update_market_meta(ev.market_ticker, ev.payload)
        state.update_market(ev.market_ticker, {
            "title": ev.payload.get("title"),
            "close_time": ev.payload.get("close_time"),
        })
        return

    if ev.kind != "orderbook":
        # ticker/trade alimentan el histórico vía scanner._record_history
        return

    # Reconstruimos snapshot tipado
    from .scanner import OrderBookSnapshot  # local import evita ciclos
    book = OrderBookSnapshot(**ev.payload)
    history = scanner.history(ev.market_ticker)

    decision = await brain.evaluate(ev.market_ticker, book, history)
    state.push_decision(decision)
    state.update_market(ev.market_ticker, {
        "p_market": decision.p_market,
        "p_model": decision.p_model,
        "edge": decision.edge,
        "side": decision.side,
        "confidence": decision.confidence,
        "mc_winrate": decision.mc_winrate,
    })
    await state.broadcast({
        "type": "decision",
        "ticker": decision.market_ticker,
        "edge": decision.edge,
        "side": decision.side,
        "p_model": decision.p_model,
        "p_market": decision.p_market,
    })

    if not decision.actionable(get_settings().edge_threshold):
        return

    # Sizing — usamos el mejor bid del lado elegido como precio limit.
    if decision.side == "YES" and book.yes_bids:
        price = int(book.yes_bids[0][0])
    elif decision.side == "NO" and book.no_bids:
        price = int(book.no_bids[0][0])
    else:
        price = 50  # fallback razonable
    sizing = risk.size(decision, balance_cents=state.balance_cents,
                       price_cents=price)
    if sizing.contracts == 0:
        return

    # Ejecución
    result = await executor.submit(sizing)
    state.push_order(sizing, result)
    state.balance_cents = max(0, state.balance_cents - sizing.notional_cents)
    await state.broadcast({
        "type": "order",
        "ticker": sizing.market_ticker,
        "side": sizing.side,
        "contracts": sizing.contracts,
        "price": sizing.price_cents,
        "ok": result.ok,
        "status": result.status,
    })


# ===========================================================================
# Integración con atlas-core (FastAPI app principal)
# ===========================================================================
def register(app, state: Optional[RadarState] = None,
             use_v2: Optional[bool] = None) -> RadarState:
    """
    Monta el radar en una ``FastAPI`` ya existente (:8791).

    Por defecto usa el ``Orchestrator`` v2 (calibración + ensemble + gating +
    risk engine + executor v2 + exits + journal). Si ``ATLAS_RADAR_LEGACY=1``
    o ``use_v2=False``, usa el ``trading_loop`` legado de PR #14.
    """
    import os as _os
    if use_v2 is None:
        use_v2 = _os.getenv("ATLAS_RADAR_LEGACY", "0") != "1"

    reload_settings()
    state = state or RadarState()
    app.include_router(build_router(state))

    if use_v2:
        from .orchestrator import Orchestrator
        orch = Orchestrator(state=state)
        lock_path = state.settings.log_dir / "atlas_radar_kalshi.runtime.lock"
        runtime_enabled = _acquire_runtime_lock(lock_path)
        try:
            state.runtime_enabled = runtime_enabled  # type: ignore[attr-defined]
        except Exception:
            pass
        if not runtime_enabled:
            logging.getLogger("radar.kalshi.main").warning(
                "Radar runtime lock ocupado; este proceso NO iniciará orquestador."
            )
        # exponer el orquestador en el state para los endpoints /api/radar/*
        try:
            state.orchestrator = orch  # type: ignore[attr-defined]
        except Exception:
            pass

        @app.on_event("startup")
        async def _start_radar() -> None:  # pragma: no cover - runtime hook
            if not getattr(state, "runtime_enabled", True):
                return
            task = getattr(state, "orchestrator_task", None)
            if task is None or task.done():
                state.orchestrator_task = asyncio.create_task(
                    orch.start(), name="atlas-radar-kalshi-orch"
                )

        @app.on_event("shutdown")
        async def _stop_radar() -> None:  # pragma: no cover - runtime hook
            orch.stop()
            task = getattr(state, "orchestrator_task", None)
            if task is not None and not task.done():
                task.cancel()
            if getattr(state, "runtime_enabled", True):
                _release_runtime_lock()
    else:
        @app.on_event("startup")
        async def _start_radar_legacy() -> None:  # pragma: no cover
            asyncio.create_task(trading_loop(state),
                                name="atlas-radar-kalshi-loop")

    return state


# ===========================================================================
# CLI stand-alone
# ===========================================================================
def _build_standalone_app():  # pragma: no cover
    from fastapi import FastAPI
    app = FastAPI(title="Atlas Radar Kalshi", version="0.1.0")
    register(app)
    return app


def main() -> None:  # pragma: no cover
    parser = argparse.ArgumentParser(prog="atlas-radar-kalshi")
    parser.add_argument("--serve", action="store_true",
                        help="Levanta uvicorn en :8792 con UI + API.")
    parser.add_argument("--loop", action="store_true",
                        help="Solo loop de trading, sin servidor HTTP.")
    parser.add_argument("--legacy", action="store_true",
                        help="Usar trading_loop legacy en lugar de Orchestrator v2.")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8792)
    args = parser.parse_args()

    if args.loop and not args.serve:
        if args.legacy:
            asyncio.run(trading_loop())
        else:
            from .orchestrator import Orchestrator
            asyncio.run(Orchestrator().start())
        return

    import uvicorn
    uvicorn.run(_build_standalone_app(), host=args.host, port=args.port)


if __name__ == "__main__":  # pragma: no cover
    main()

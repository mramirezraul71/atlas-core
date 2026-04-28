"""
orchestrator.py — Loop autónomo end-to-end (scan→score→select→risk→exec→exit).

Combina:

- ``KalshiScanner`` (capa 1)
- ``RadarBrain`` (LLM + Markov + Monte Carlo, capa 2 antigua)
- ``SignalEnsemble`` + ``Calibrator`` (ensemble + calibración)
- ``Gating`` (filtros de calidad)
- ``RiskEngine`` (Kelly + caps + breakers + kill)
- ``KalshiExecutorV2`` (idempotency + reintentos + reconcile)
- ``ExitManager`` (TP/SL/time-stop/forced-exit)
- ``Journal`` (auditoría)

Watchdog: si no hay actividad reciente del feed (ver ``_watchdog``), marca
``health.degraded``. Eso **no** bloquea nuevas entradas en el pipeline
actual; afecta salidas forzadas (``ExitManager`` con ``data_degraded``)
y el aviso en UI. La ausencia de órdenes suele venir del gating/riesgo.
"""
from __future__ import annotations

import asyncio
import os
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Optional

from .brain import RadarBrain
from .calibration import Calibrator, fit_from_disk
from .config import RadarSettings, get_settings
from .dashboard import RadarState
from .exit_manager import ExitConfig, ExitManager, Position
from .executor_v2 import (ExecConfig, FillReport, KalshiExecutorV2,
                          OrderRequestV2)
from .gating import GateConfig, Gating
from .risk_engine import RiskEngine, RiskLimits, RiskState
from .scanner import KalshiScanner, MarketEvent, OrderBookSnapshot
from .signals import EnsembleWeights, SignalEnsemble, SignalReadout
from .state.journal import Journal
from .utils.logger import get_logger
from .profiles import RADAR_PROFILE_PRESETS


# ===========================================================================
@dataclass
class Health:
    last_event_ts: float = 0.0
    last_decision_ts: float = 0.0
    last_order_ts: float = 0.0
    degraded: bool = False
    # Diagnóstico watchdog (se rellenan en _watchdog)
    watchdog_stale_sec: float = 60.0
    seconds_since_activity: float = 0.0

    def tick(self, kind: str) -> None:
        now = time.time()
        if kind == "event":
            self.last_event_ts = now
        elif kind == "decision":
            self.last_decision_ts = now
        elif kind == "order":
            self.last_order_ts = now


# ===========================================================================
class Orchestrator:
    """Wiring + loop principal del Radar."""

    def __init__(
        self,
        settings: Optional[RadarSettings] = None,
        state: Optional[RadarState] = None,
        risk_limits: Optional[RiskLimits] = None,
        gate_cfg: Optional[GateConfig] = None,
        exit_cfg: Optional[ExitConfig] = None,
        exec_cfg: Optional[ExecConfig] = None,
    ) -> None:
        self.settings = settings or get_settings()
        self.state = state or RadarState(self.settings)
        self.log = get_logger("orchestrator", self.settings.log_dir,
                              self.settings.log_level)
        profile = (os.getenv("RADAR_PROFILE", "paper_safe") or "").strip().lower()
        if profile not in RADAR_PROFILE_PRESETS:
            self.log.warning(
                "RADAR_PROFILE desconocido '%s'; usando paper_safe", profile
            )
            profile = "paper_safe"
        self.profile = profile
        p_gate = RADAR_PROFILE_PRESETS[self.profile]["gate"]
        p_risk = RADAR_PROFILE_PRESETS[self.profile]["risk"]
        self.scanner = KalshiScanner(self.settings)
        self.brain = RadarBrain(self.settings)
        self.ensemble = SignalEnsemble(EnsembleWeights(
            micro=float(os.getenv("RADAR_W_MICRO", 0.30)),
            markov=float(os.getenv("RADAR_W_MARKOV", 0.20)),
            llm=float(os.getenv("RADAR_W_LLM", 0.40)),
            momentum=float(os.getenv("RADAR_W_MOM", 0.10)),
        ))
        self.calibrator: Calibrator = fit_from_disk(
            self.settings.log_dir / "radar_calibration.jsonl",
            method=os.getenv("RADAR_CAL_METHOD", "platt"),
        )
        self.gating = Gating(gate_cfg or GateConfig(
            edge_net_min=float(os.getenv("RADAR_EDGE_NET_MIN", p_gate["edge_net_min"])),
            confidence_min=float(os.getenv("RADAR_CONFIDENCE_MIN", p_gate["confidence_min"])),
            spread_max_ticks=int(os.getenv("RADAR_SPREAD_MAX", p_gate["spread_max_ticks"])),
            min_depth_yes=int(os.getenv("RADAR_MIN_DEPTH_YES", p_gate["min_depth_yes"])),
            min_depth_no=int(os.getenv("RADAR_MIN_DEPTH_NO", p_gate["min_depth_no"])),
            max_quote_age_ms=int(os.getenv("RADAR_MAX_QUOTE_AGE_MS", p_gate["max_quote_age_ms"])),
            max_latency_ms=int(os.getenv("RADAR_MAX_LATENCY_MS", p_gate["max_latency_ms"])),
            cooldown_seconds=int(os.getenv("RADAR_COOLDOWN_S", p_gate["cooldown_seconds"])),
        ))
        self.risk = RiskEngine(risk_limits or RiskLimits(
            kelly_fraction=float(os.getenv("RADAR_KELLY_FRACTION", p_risk["kelly_fraction"])),
            max_position_pct=float(os.getenv("RADAR_MAX_POSITION_PCT", p_risk["max_position_pct"])),
            max_market_exposure_pct=float(os.getenv("RADAR_MAX_MARKET_EXP", p_risk["max_market_exposure_pct"])),
            max_total_exposure_pct=float(os.getenv("RADAR_MAX_TOTAL_EXP", p_risk["max_total_exposure_pct"])),
            daily_dd_limit_pct=float(os.getenv("RADAR_DAILY_DD", p_risk["daily_dd_limit_pct"])),
            weekly_dd_limit_pct=float(os.getenv("RADAR_WEEKLY_DD", p_risk["weekly_dd_limit_pct"])),
            max_consecutive_losses=int(os.getenv("RADAR_MAX_CL", p_risk["max_consecutive_losses"])),
            max_open_positions=int(os.getenv("RADAR_MAX_OPEN", p_risk["max_open_positions"])),
            max_orders_per_minute=int(os.getenv("RADAR_MAX_OPM", p_risk["max_orders_per_minute"])),
        ))
        self.exit_mgr = ExitManager(exit_cfg or ExitConfig(
            tp_capture_pct=float(os.getenv("RADAR_TP_PCT", 0.6)),
            sl_ticks=int(os.getenv("RADAR_SL_TICKS", 4)),
            sl_edge_revert=float(os.getenv("RADAR_SL_EDGE", -0.02)),
            time_stop_seconds=int(os.getenv("RADAR_TIME_STOP_S", 1800)),
        ))
        self.executor = KalshiExecutorV2(self.settings, exec_cfg or ExecConfig(
            enable_live=os.getenv("ATLAS_RADAR_LIVE", "0") == "1",
            prefer_maker=os.getenv("RADAR_PREFER_MAKER", "1") == "1",
            max_chase_ticks=int(os.getenv("RADAR_MAX_CHASE", 1)),
            max_retries=int(os.getenv("RADAR_MAX_RETRIES", 3)),
        ))
        self.journal = Journal(self.settings.log_dir)
        self.health = Health()
        self._stop = asyncio.Event()
        self._decision_cooldown_s = max(
            0.0, float(os.getenv("RADAR_DECISION_COOLDOWN_S", "5.0"))
        )
        self._next_decision_by_ticker: dict[str, float] = {}
        self._event_queue: asyncio.Queue[MarketEvent] = asyncio.Queue(
            maxsize=max(100, int(self.settings.event_queue_maxsize))
        )
        self._event_workers = max(1, int(self.settings.decision_workers))
        self._worker_tasks: list[asyncio.Task] = []
        self._pending_tickers: set[str] = set()
        self._dropped_events: int = 0
        self._journal_all_decisions = (
            os.getenv("RADAR_JOURNAL_ALL_DECISIONS", "0") == "1"
        )
        self._last_degraded_log_ts: float = 0.0
        self._last_recover_ts: float = 0.0

    # ------------------------------------------------------------------
    async def start(self) -> None:
        """Punto de entrada: boot + watchdog + scan loop."""
        self.log.info("Orchestrator boot — env=%s live=%s",
                      self.settings.kalshi_environment,
                      self.executor.cfg.enable_live)
        try:
            self.risk.update_balance(await self.executor.balance_cents())
        except Exception as exc:
            self.log.warning("Balance lookup failed: %s", exc)
            self.risk.update_balance(100_000)  # paper fallback
        self.state.balance_cents = self.risk.state.balance_cents

        self._worker_tasks = [
            asyncio.create_task(
                self._event_worker(idx), name=f"radar-event-worker-{idx}"
            )
            for idx in range(self._event_workers)
        ]
        watchdog = asyncio.create_task(self._watchdog(), name="radar-watchdog")
        try:
            async for ev in self.scanner.stream():
                self.health.tick("event")
                self._enqueue_event(ev)
                if self._stop.is_set():
                    break
        finally:
            watchdog.cancel()
            for t in self._worker_tasks:
                t.cancel()
            self._worker_tasks.clear()

    def stop(self) -> None:
        self._stop.set()

    # ------------------------------------------------------------------
    async def _watchdog(self) -> None:
        """Heartbeat + degradado si scanner queda silencioso > 60s.

        Además sincroniza ``state.exec_metrics``/``risk_status``/``health`` y
        atiende el flag ``state.kill_requested`` (botones del dashboard).
        """
        while not self._stop.is_set():
            await asyncio.sleep(5)
            stale_sec = max(
                10.0,
                float(os.getenv("RADAR_WATCHDOG_STALE_SEC", "120")),
            )
            self.health.watchdog_stale_sec = stale_sec
            # Actividad: el feed principal marca eventos; las decisiones
            # refuerzan (si el cola tarda, sigue contando "vida" del bucle).
            act_ts = max(
                self.health.last_event_ts,
                self.health.last_decision_ts,
            )
            if act_ts <= 0.0:
                act_ts = 1.0
            age = time.time() - act_ts
            self.health.seconds_since_activity = max(0.0, age)
            self.health.degraded = age > stale_sec
            if self.health.degraded:
                now = time.time()
                if now - self._last_degraded_log_ts >= 90.0:
                    self._last_degraded_log_ts = now
                    self.log.warning(
                        "Watchdog: degraded=True (sin actividad >%.0fs; "
                        "ahora=%.0fs; last_event=%.1f last_decision=%.1f). "
                        "Ajusta RADAR_WATCHDOG_STALE_SEC o revisa scanner/feed.",
                        stale_sec, age, self.health.last_event_ts,
                        self.health.last_decision_ts,
                    )
                recover_after = max(
                    stale_sec,
                    float(os.getenv("RADAR_WATCHDOG_RECOVER_SEC", "180")),
                )
                recover_cd = max(
                    30.0,
                    float(os.getenv("RADAR_WATCHDOG_RECOVER_COOLDOWN_SEC", "90")),
                )
                if age >= recover_after and (now - self._last_recover_ts) >= recover_cd:
                    self._last_recover_ts = now
                    await self._attempt_feed_recovery(age_s=age)
            # publicar al state para que /api/radar/{metrics,risk,health} sirvan datos
            try:
                self.state.exec_metrics = self.executor.metrics.to_dict()  # type: ignore[attr-defined]
                self.state.risk_status = self.risk.status()  # type: ignore[attr-defined]
                self.state.health = self.health  # type: ignore[attr-defined]
                self.state.queue_depth = int(self._event_queue.qsize())  # type: ignore[attr-defined]
                self.state.queue_dropped = int(self._dropped_events)  # type: ignore[attr-defined]
            except Exception:
                pass
            # botones kill / resume del dashboard
            kill_req = bool(getattr(self.state, "kill_requested", False))
            if kill_req and not self.risk.state.kill_switch:
                self.risk.kill("dashboard")
            elif not kill_req and self.risk.state.kill_switch and \
                    os.getenv("ATLAS_RADAR_KILL", "0") != "1":
                self.risk.reset_kill()
            await self.state.broadcast({"type": "ping",
                                        "degraded": self.health.degraded})

    async def _attempt_feed_recovery(self, age_s: float) -> None:
        """Autocuración del feed sin reiniciar todo PUSH."""
        self.log.warning(
            "Watchdog recovery: feed inactivo %.0fs. Ejecutando discover+pulse+reconnect.",
            age_s,
        )
        try:
            self.scanner.refresh_feed_mode()
        except Exception:
            pass
        try:
            new_tickers = await self.scanner.discover_markets()
            if new_tickers:
                self.log.info("Recovery discover: nuevos tickers=%d", len(new_tickers))
        except Exception as exc:
            self.log.warning("Recovery discover falló: %s", exc)
        try:
            emitted = await self.scanner.pulse_public_orderbooks(
                limit=max(5, min(25, int(self.settings.ws_max_tickers)))
            )
            if emitted > 0:
                self.log.info("Recovery pulse: orderbooks emitidos=%d", emitted)
        except Exception as exc:
            self.log.warning("Recovery pulse falló: %s", exc)
        try:
            await self.scanner.reconnect()
        except Exception as exc:
            self.log.debug("Recovery reconnect (best-effort): %s", exc)

    def _enqueue_event(self, ev: MarketEvent) -> None:
        """Encola sin bloquear ingestión WS; deduplica por ticker."""
        ticker_scoped = ev.kind in {"orderbook", "ticker"}
        if ticker_scoped:
            ticker = ev.market_ticker
            if ticker in self._pending_tickers:
                return
            self._pending_tickers.add(ticker)
        try:
            self._event_queue.put_nowait(ev)
        except asyncio.QueueFull:
            self._dropped_events += 1
            if ticker_scoped:
                self._pending_tickers.discard(ev.market_ticker)
            if self._dropped_events == 1 or self._dropped_events % 200 == 0:
                self.log.warning(
                    "Event queue full: dropped=%d max=%d",
                    self._dropped_events,
                    self._event_queue.maxsize,
                )

    async def _event_worker(self, idx: int) -> None:
        """Worker de eventos con backpressure controlado."""
        while not self._stop.is_set():
            ev = await self._event_queue.get()
            try:
                await self._on_event(ev)
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                self.log.exception("Worker[%d] fallo procesando %s: %s",
                                   idx, ev.kind, exc)
            finally:
                if ev.kind in {"orderbook", "ticker"}:
                    self._pending_tickers.discard(ev.market_ticker)
                self._event_queue.task_done()

    # ------------------------------------------------------------------
    async def _on_event(self, ev: MarketEvent) -> None:
        if ev.kind == "new_market":
            self.brain.update_market_meta(ev.market_ticker, ev.payload)
            self.state.update_market(ev.market_ticker, {
                "title": ev.payload.get("title"),
                "close_time": ev.payload.get("close_time"),
            })
            return
        if ev.kind == "orderbook":
            book = OrderBookSnapshot(**ev.payload)
        elif ev.kind == "ticker":
            book = self._book_from_ticker(ev)
            if book is None:
                return
        else:
            return

        history = self.scanner.history(ev.market_ticker)
        self.state.update_market(ev.market_ticker, {
            "p_market": book.yes_mid or 0.5,
            "last_book_ts": ev.ts.isoformat(),
        })

        now = time.time()
        next_allowed = self._next_decision_by_ticker.get(ev.market_ticker, 0.0)
        if now < next_allowed:
            return
        self._next_decision_by_ticker[ev.market_ticker] = (
            now + self._decision_cooldown_s
        )

        # 1) brain (LLM + MC) — devuelve también p_market
        decision = await self.brain.evaluate(ev.market_ticker, book, history)
        # 2) ensemble + calibración
        readout = self.ensemble.evaluate(book, history,
                                         p_llm=decision.p_model,
                                         llm_confidence=decision.confidence)
        p_calibrated = self.calibrator.predict(readout.p_ensemble)
        readout = readout.model_copy(update={"p_ensemble": p_calibrated})
        self.health.tick("decision")
        self.state.push_decision(decision)

        # 3) gestionar salidas primero (tiene prioridad)
        await self._manage_exits(ev.market_ticker, book, readout)

        # 4) gating sobre nueva entrada
        gate = self.gating.evaluate(
            ev.market_ticker, readout, p_market=book.yes_mid or 0.5,
            quote_age_ms=self._quote_age_ms(book, ev),
            latency_ms=0,
        )
        if self._journal_all_decisions or gate.accepted:
            self.journal.write("decisions", {
                "ticker": ev.market_ticker,
                "decision": decision.model_dump(mode="json"),
                "readout": readout.model_dump(mode="json"),
                "gate": gate.model_dump(mode="json"),
            })

        # propaga al dashboard
        self.state.update_market(ev.market_ticker, {
            "p_market": book.yes_mid or 0.5,
            "p_model": readout.p_ensemble,
            "edge": gate.edge_gross,
            "edge_net": gate.edge_net,
            "side": gate.side if gate.accepted else None,
            "confidence": readout.confidence,
            "spread": readout.spread_ticks,
            "depth_yes": readout.depth_yes,
            "depth_no": readout.depth_no,
            "score": gate.score,
            "gate_reason": gate.reason,
        })
        await self.state.broadcast({
            "type": "decision",
            "ticker": ev.market_ticker,
            "edge": gate.edge_gross,
            "edge_net": gate.edge_net,
            "side": gate.side, "p_model": readout.p_ensemble,
            "p_market": book.yes_mid or 0.5,
            "score": gate.score, "gate": gate.reason,
        })
        if not gate.accepted:
            return

        # 5) sizing
        sizing = self.risk.size(gate, readout, ev.market_ticker)
        if sizing.contracts <= 0:
            if sizing.safe_mode:
                self.journal.write("risk", {"event": "safe_mode",
                                            "rationale": sizing.rationale})
            return

        # 6) ejecución
        coid = self.executor.make_client_order_id(
            ev.market_ticker, sizing.side, sizing.price_cents, sizing.contracts
        )
        order = OrderRequestV2(
            market_ticker=ev.market_ticker, side=sizing.side,
            contracts=sizing.contracts, price_cents=sizing.price_cents,
            client_order_id=coid, reason="entry",
        )
        report = await self.executor.submit(order)
        self.health.tick("order")
        self.gating.stamp(ev.market_ticker)
        self.risk.on_order(ev.market_ticker, sizing.notional_cents)
        self.state.push_order(self._sizing_to_position(sizing), self._fill_to_result(report))
        self.journal.write("orders", {"order": order.model_dump(),
                                      "report": report.model_dump(mode="json"),
                                      "sizing": sizing.model_dump()})

        if report.ok and report.filled_contracts > 0:
            tp, sl = ExitManager.build_targets(
                sizing.side, int(report.avg_fill_price or sizing.price_cents),
                p_fair=readout.p_ensemble,
                tp_capture_pct=self.exit_mgr.cfg.tp_capture_pct,
                sl_ticks=self.exit_mgr.cfg.sl_ticks,
            )
            self.exit_mgr.open(Position(
                ticker=ev.market_ticker, side=sizing.side,
                entry_price=int(report.avg_fill_price or sizing.price_cents),
                size=report.filled_contracts,
                entry_ts=time.time(),
                edge_at_entry=gate.edge_net,
                target_price=tp, stop_price=sl,
                order_id=report.order_id,
            ))

        await self.state.broadcast({
            "type": "order",
            "ticker": ev.market_ticker, "side": sizing.side,
            "contracts": report.filled_contracts,
            "price": int(report.avg_fill_price or sizing.price_cents),
            "ok": report.ok, "status": report.status,
            "latency_ms": report.latency_ms,
        })

    # ------------------------------------------------------------------
    async def _manage_exits(self, ticker: str, book: OrderBookSnapshot,
                            readout: SignalReadout) -> None:
        cur_price = int(round((book.yes_mid or 0.5) * 100))
        cur_edge = readout.p_ensemble - (book.yes_mid or 0.5)
        forced = (self.risk.state.kill_switch or
                  os.getenv("ATLAS_RADAR_KILL", "0") == "1")
        sig = self.exit_mgr.evaluate(
            ticker=ticker, current_price=cur_price, current_edge=cur_edge,
            data_degraded=self.health.degraded, forced=forced,
        )
        if not sig.should_exit:
            return
        pos = self.exit_mgr.positions.get(ticker)
        if not pos:
            return
        exit_side = "NO" if pos.side == "YES" else "YES"
        exit_price = max(1, min(99, sig.target_price or cur_price))
        coid = self.executor.make_client_order_id(
            ticker, exit_side, exit_price, pos.size, ts_bucket_ms=500
        )
        order = OrderRequestV2(
            market_ticker=ticker, side=exit_side, contracts=pos.size,
            price_cents=exit_price, client_order_id=coid, reason=sig.reason,
        )
        report = await self.executor.submit(order)
        # PnL aproximado en ¢ por contrato (paper / live aproximado)
        pnl_per = (exit_price - pos.entry_price) if pos.side == "YES" \
            else (pos.entry_price - exit_price)
        pnl = int(pnl_per * pos.size - 0.07 * pos.size)  # fees ~7 bps
        self.risk.on_close(ticker, pos.size * pos.entry_price, pnl)
        self.exit_mgr.close(ticker)
        self.journal.write("exits", {
            "ticker": ticker, "reason": sig.reason,
            "entry": pos.entry_price, "exit": exit_price,
            "size": pos.size, "pnl_cents": pnl,
            "fees_cents": int(0.07 * pos.size),
            "slippage_cents": report.slippage_cents,
        })

    # ------------------------------------------------------------------
    @staticmethod
    def _quote_age_ms(book: OrderBookSnapshot, ev: MarketEvent) -> int:
        """Antigüedad de la cotización: preferir book.ts (momento del snapshot)."""
        try:
            ref: datetime = book.ts
            if getattr(ref, "tzinfo", None) is None:
                ref = ref.replace(tzinfo=timezone.utc)
            return max(0, int((time.time() - ref.timestamp()) * 1000))
        except Exception:
            try:
                return max(
                    0, int((time.time() - ev.ts.timestamp()) * 1000)
                )
            except Exception:
                return 0

    @staticmethod
    def _to_cents(value: object) -> Optional[int]:
        if value is None:
            return None
        try:
            v = float(value)
        except Exception:
            return None
        cents = int(round(v * 100)) if abs(v) <= 1.0 else int(round(v))
        return max(0, min(99, cents))

    def _book_from_ticker(self, ev: MarketEvent) -> Optional[OrderBookSnapshot]:
        payload = ev.payload or {}
        bid = self._to_cents(payload.get("yes_bid"))
        if bid is None:
            bid = self._to_cents(payload.get("yes_bid_dollars"))
        ask = self._to_cents(payload.get("yes_ask"))
        if ask is None:
            ask = self._to_cents(payload.get("yes_ask_dollars"))
        if bid is None and ask is None:
            return None
        if bid is None:
            bid = max(1, int(ask or 50) - 1)
        if ask is None:
            ask = min(99, int(bid) + 1)
        qty_bid = int(payload.get("yes_bid_size") or 1)
        qty_ask = int(payload.get("yes_ask_size") or 1)
        no_bid = max(1, min(99, 100 - ask))
        no_ask = max(1, min(99, 100 - bid))
        return OrderBookSnapshot(
            market_ticker=ev.market_ticker,
            yes_bids=[(bid, max(1, qty_bid))],
            yes_asks=[(ask, max(1, qty_ask))],
            no_bids=[(no_bid, max(1, qty_ask))],
            no_asks=[(no_ask, max(1, qty_bid))],
            ts=datetime.now(timezone.utc),
        )

    # ------------------------------------------------------------------
    def _sizing_to_position(self, sizing):
        from .risk import PositionSize  # compat para dashboard.push_order
        return PositionSize(
            market_ticker=sizing.market_ticker, side=sizing.side,
            price_cents=sizing.price_cents, contracts=sizing.contracts,
            notional_cents=sizing.notional_cents,
            kelly_fraction=sizing.f_full,
            fractional_kelly=sizing.f_capped,
            capped_fraction=sizing.f_capped,
            rationale=sizing.rationale,
        )

    def _fill_to_result(self, report: FillReport):
        from .executor import OrderResult
        return OrderResult(
            ok=report.ok, order_id=report.order_id,
            status=report.status, raw=report.raw, error=report.error,
        )

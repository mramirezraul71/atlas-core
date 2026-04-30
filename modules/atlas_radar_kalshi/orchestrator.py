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

Watchdog + heartbeat + auto-recovery están integrados; si falla un
proveedor, el sistema entra en *modo degradado* (sin nuevas entradas,
sólo gestión de salidas y cancelaciones) hasta que vuelva a estar
sano.
"""
from __future__ import annotations

import asyncio
import os
import time
from dataclasses import dataclass, field
from typing import Optional

from .brain import RadarBrain
from .calibration import Calibrator, fit_from_disk
from .config import RadarSettings, get_settings
from .dashboard import RadarState
from .arbitrage_engine import ArbitrageEngine
from .alerts import AlertEngine
from .exit_manager import ExitConfig, ExitManager, Position
from .executor_v2 import ExecConfig, FillReport, OrderRequestV2
from .execution.reconciliation import reconcile_kalshi_from_router
from .execution_router import ExecutionModeRouter
from .ingestion.health import ConnectorRegistry
from .gating import GateConfig, Gating
from .learning_engine import LearningEngine
from .polymarket_scanner import PolymarketScanner
from .risk_engine import RiskEngine, RiskLimits, RiskState
from .scanner import KalshiScanner, MarketEvent, OrderBookSnapshot
from .signals import EnsembleWeights, SignalEnsemble, SignalReadout
from .state.journal import Journal
from .utils.logger import get_logger


# ===========================================================================
@dataclass
class Health:
    last_event_ts: float = 0.0
    last_decision_ts: float = 0.0
    last_order_ts: float = 0.0
    degraded: bool = False

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
            edge_net_min=float(os.getenv("RADAR_EDGE_NET_MIN", 0.03)),
            confidence_min=float(os.getenv("RADAR_CONFIDENCE_MIN", 0.62)),
            spread_max_ticks=int(os.getenv("RADAR_SPREAD_MAX", 5)),
            min_depth_yes=int(os.getenv("RADAR_MIN_DEPTH_YES", 50)),
            min_depth_no=int(os.getenv("RADAR_MIN_DEPTH_NO", 50)),
            max_quote_age_ms=int(os.getenv("RADAR_MAX_QUOTE_AGE_MS", 3000)),
            max_latency_ms=int(os.getenv("RADAR_MAX_LATENCY_MS", 1500)),
            cooldown_seconds=int(os.getenv("RADAR_COOLDOWN_S", 30)),
        ))
        self.risk = RiskEngine(risk_limits or RiskLimits(
            kelly_fraction=float(os.getenv("RADAR_KELLY_FRACTION", 0.25)),
            max_position_pct=float(os.getenv("RADAR_MAX_POSITION_PCT", 0.05)),
            max_market_exposure_pct=float(os.getenv("RADAR_MAX_MARKET_EXP", 0.10)),
            max_total_exposure_pct=float(os.getenv("RADAR_MAX_TOTAL_EXP", 0.50)),
            max_kalshi_venue_exposure_pct=float(
                os.getenv("RADAR_MAX_VENUE_EXP_KALSHI", 0.50)
            ),
            max_polymarket_venue_exposure_pct=float(
                os.getenv("RADAR_MAX_VENUE_EXP_POLY", 0.50)
            ),
            daily_dd_limit_pct=float(os.getenv("RADAR_DAILY_DD", 0.05)),
            weekly_dd_limit_pct=float(os.getenv("RADAR_WEEKLY_DD", 0.10)),
            max_consecutive_losses=int(os.getenv("RADAR_MAX_CL", 5)),
            max_open_positions=int(os.getenv("RADAR_MAX_OPEN", 8)),
            max_orders_per_minute=int(os.getenv("RADAR_MAX_OPM", 30)),
        ))
        self.exit_mgr = ExitManager(exit_cfg or ExitConfig(
            tp_capture_pct=float(os.getenv("RADAR_TP_PCT", 0.6)),
            sl_ticks=int(os.getenv("RADAR_SL_TICKS", 4)),
            sl_edge_revert=float(os.getenv("RADAR_SL_EDGE", -0.02)),
            time_stop_seconds=int(os.getenv("RADAR_TIME_STOP_S", 1800)),
        ))
        self.poly_scanner = PolymarketScanner(self.settings)
        self.executor = ExecutionModeRouter(self.settings, exec_cfg or ExecConfig(
            enable_live=self.settings.execution_mode == "live",
            prefer_maker=os.getenv("RADAR_PREFER_MAKER", "1") == "1",
            max_chase_ticks=int(os.getenv("RADAR_MAX_CHASE", 1)),
            max_retries=int(os.getenv("RADAR_MAX_RETRIES", 3)),
        ))
        self.journal = Journal(self.settings.log_dir)
        self.learning = LearningEngine(self.settings.log_dir / "radar_learning_state.json")
        self.arbitrage = ArbitrageEngine(
            min_profit=float(os.getenv("RADAR_ARB_MIN_PROFIT", "0.02"))
        )
        self.alerts = AlertEngine(
            log_dir=str(self.settings.log_dir),
            log_level=self.settings.log_level,
            min_hedge_rate_1h=float(os.getenv("RADAR_ALERT_MIN_HEDGE_RATE_1H", "0.60")),
            cooldown_s=int(os.getenv("RADAR_ALERT_COOLDOWN_S", "300")),
        )
        self._arb_exec_cooldown_s = int(os.getenv("RADAR_ARB_COOLDOWN_S", "60"))
        self._arb_contracts = int(os.getenv("RADAR_ARB_CONTRACTS", "1"))
        self._arb_leg2_retries = int(os.getenv("RADAR_ARB_LEG2_RETRIES", "2"))
        self._arb_leg2_retry_delay_ms = int(os.getenv("RADAR_ARB_LEG2_RETRY_DELAY_MS", "250"))
        self._last_arb_exec_by_key: dict[str, float] = {}
        self._arb_activity: list[dict] = []
        self._arb_stats: dict[str, int] = {
            "total": 0,
            "hedged": 0,
            "unwinded": 0,
            "partial": 0,
            "failed": 0,
        }
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
        self._last_source_event: dict[str, float] = {}
        # Último mid YES por ticker (portfolio UPNL coherente entre eventos).
        self._last_yes_mid: dict[str, float] = {}
        self._connectors = ConnectorRegistry()
        self._watchdog_ticks = 0
        self._reconcile_every_n_ticks = max(1, int(os.getenv("RADAR_RECONCILE_SEC", "60")) // 5)

    # ------------------------------------------------------------------
    async def start(self) -> None:
        """Punto de entrada: boot + watchdog + scan loop."""
        self.log.info("Orchestrator boot — env=%s live=%s",
                      self.settings.kalshi_environment,
                      self.executor.cfg.enable_live)
        scanner_tasks: list[asyncio.Task] = []
        try:
            self.risk.update_balance(await self.executor.balance_cents())
        except Exception as exc:
            self.log.warning("Balance lookup failed: %s", exc)
            self.risk.update_balance(100_000)  # paper fallback
        self.state.balance_cents = self.risk.state.balance_cents
        try:
            self.state.arbitrage = []  # type: ignore[attr-defined]
            self.state.arb_activity = []  # type: ignore[attr-defined]
            self.state.arb_stats = dict(self._arb_stats)  # type: ignore[attr-defined]
        except Exception:
            pass

        self._worker_tasks = [
            asyncio.create_task(
                self._event_worker(idx), name=f"radar-event-worker-{idx}"
            )
            for idx in range(self._event_workers)
        ]
        watchdog = asyncio.create_task(self._watchdog(), name="radar-watchdog")
        async def _pump(scanner_stream, source: str):
            try:
                async for ev in scanner_stream:
                    self.health.tick("event")
                    self._last_source_event[source] = time.time()
                    self._connectors.touch_event(source)
                    self._enqueue_event(ev)
                    if self._stop.is_set():
                        break
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                self._connectors.record_reconnect(source, str(exc))
                self.log.warning("Pump %s detenido: %s", source, exc)

        scanner_tasks.append(
            asyncio.create_task(
                _pump(self.scanner.stream(), "kalshi"), name="radar-kalshi-pump"
            )
        )
        if self.settings.polymarket_enabled:
            scanner_tasks.append(
                asyncio.create_task(
                    _pump(self.poly_scanner.stream(), "polymarket"), name="radar-poly-pump"
                )
            )

        try:
            await self._stop.wait()
        finally:
            for st in scanner_tasks:
                st.cancel()
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
            self._watchdog_ticks += 1
            stale = time.time() - max(self.health.last_event_ts, 1) > 60
            self.health.degraded = stale
            # publicar al state para que /api/radar/{metrics,risk,health} sirvan datos
            try:
                self.state.exec_metrics = self.executor.metrics.to_dict()  # type: ignore[attr-defined]
                self.state.risk_status = self.risk.status()  # type: ignore[attr-defined]
                self.state.health = self.health  # type: ignore[attr-defined]
                self.state.queue_depth = int(self._event_queue.qsize())  # type: ignore[attr-defined]
                self.state.queue_dropped = int(self._dropped_events)  # type: ignore[attr-defined]
                self.state.connector_health = self._connectors.snapshot()  # type: ignore[attr-defined]
                if (
                    self.settings.execution_mode == "live"
                    and self._watchdog_ticks % self._reconcile_every_n_ticks == 0
                ):
                    self.state.last_reconcile = await reconcile_kalshi_from_router(  # type: ignore[attr-defined]
                        self.executor
                    )
                now = time.time()
                self.state.venue_status = {  # type: ignore[attr-defined]
                    "kalshi": {
                        "enabled": True,
                        "connected": (now - self._last_source_event.get("kalshi", 0.0)) <= 60,
                        "stale": (now - self._last_source_event.get("kalshi", 0.0)) > 60,
                    },
                    "polymarket": {
                        "enabled": bool(self.settings.polymarket_enabled),
                        "connected": (now - self._last_source_event.get("polymarket", 0.0)) <= 60
                        if self.settings.polymarket_enabled
                        else False,
                        "stale": (now - self._last_source_event.get("polymarket", 0.0)) > 60
                        if self.settings.polymarket_enabled
                        else True,
                    },
                }
                await self._run_alert_checks()
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

        self._last_yes_mid[ev.market_ticker] = float(book.yes_mid or 0.5)
        history = self.scanner.history(ev.market_ticker)
        self.state.update_market(ev.market_ticker, {
            "p_market": book.yes_mid or 0.5,
            "last_book_ts": ev.ts.isoformat(),
        })
        await self._update_arbitrage_book(ev.market_ticker, book)

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
        self.learning.on_decision(ev.market_ticker, decision.edge)
        p_calibrated = self.calibrator.predict(readout.p_ensemble)
        readout = readout.model_copy(update={"p_ensemble": p_calibrated})
        self.health.tick("decision")
        self.state.push_decision(decision)

        # 3) gestionar salidas primero (tiene prioridad)
        await self._manage_exits(ev.market_ticker, book, readout)

        # 4) gating sobre nueva entrada
        gate = self.gating.evaluate(
            ev.market_ticker, readout, p_market=book.yes_mid or 0.5,
            quote_age_ms=int((time.time() - ev.ts.timestamp()) * 1000),
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
        try:
            self.state.learning = self.learning.snapshot()  # type: ignore[attr-defined]
        except Exception:
            pass
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
        self._mark_unrealized_portfolio()
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
        self.learning.on_exit(ticker, pnl)
        # Ajuste autónomo de pesos con guardrails.
        new_weights = self.learning.suggest_weights(self.ensemble.weights)
        self.ensemble.weights = new_weights.normalized()
        self.learning.save()
        self.exit_mgr.close(ticker)
        self.journal.write("exits", {
            "ticker": ticker, "reason": sig.reason,
            "entry": pos.entry_price, "exit": exit_price,
            "size": pos.size, "pnl_cents": pnl,
            "fees_cents": int(0.07 * pos.size),
            "slippage_cents": report.slippage_cents,
        })

    def _mark_unrealized_portfolio(self) -> None:
        """Suma UPNL sobre todas las posiciones usando último mid cacheado por ticker."""
        total = 0
        for pos in self.exit_mgr.positions.values():
            mid = self._last_yes_mid.get(pos.ticker)
            if mid is None:
                cur = pos.entry_price
            else:
                cur = int(round(float(mid) * 100))
            pnl_per = (cur - pos.entry_price) if pos.side == "YES" else (pos.entry_price - cur)
            total += pnl_per * pos.size
        self.risk.mark_unrealized(int(total))

    @staticmethod
    def _book_ask_probs(book: OrderBookSnapshot) -> tuple[Optional[float], Optional[float]]:
        if not book.yes_asks or not book.no_asks:
            return None, None
        try:
            y_ask = min(p for p, _ in book.yes_asks) / 100.0
            n_ask = min(p for p, _ in book.no_asks) / 100.0
            return y_ask, n_ask
        except Exception:
            return None, None

    async def _update_arbitrage_book(self, ticker: str, book: OrderBookSnapshot) -> None:
        row = self.state.markets.get(ticker, {})
        title = str(row.get("title") or ticker)
        close_time = row.get("close_time")
        source = "polymarket" if ticker.upper().startswith("POLY:") else "kalshi"
        y_ask, n_ask = self._book_ask_probs(book)
        self.arbitrage.upsert_quote(
            ticker=ticker,
            source=source,
            title=title,
            close_time=close_time,
            yes_mid=book.yes_mid or 0.5,
            yes_ask=y_ask,
            no_ask=n_ask,
        )
        arb_rows = self.arbitrage.detect()
        try:
            self.state.arbitrage = arb_rows  # type: ignore[attr-defined]
        except Exception:
            pass
        await self._maybe_execute_arbitrage(arb_rows)

    async def _maybe_execute_arbitrage(self, arb_rows: list[dict]) -> None:
        if not arb_rows:
            return
        if self.risk.state.kill_switch or self.risk.state.safe_mode:
            return
        now = time.time()
        top = arb_rows[0]
        key = str(top.get("canonical_key", "na"))
        last = self._last_arb_exec_by_key.get(key, 0.0)
        if now - last < self._arb_exec_cooldown_s:
            return
        intents = self.arbitrage.build_execution_intents(top, self._arb_contracts)
        if len(intents) != 2:
            return
        reports = await self._execute_arbitrage_atomic(top, intents)
        if len(reports) >= 2 and all(r.ok for r in reports[:2]):
            self._last_arb_exec_by_key[key] = now
        arb_status = self._derive_arb_status(reports)
        self._append_arb_activity(
            {
                "ts": now,
                "canonical_key": key,
                "strategy": top.get("strategy"),
                "profit_estimate": top.get("profit_estimate"),
                "status": arb_status,
                "reports": [r.model_dump(mode="json") for r in reports],
            }
        )
        self._arb_stats["total"] += 1
        self._arb_stats[arb_status] = self._arb_stats.get(arb_status, 0) + 1
        try:
            self.state.arb_stats = dict(self._arb_stats)  # type: ignore[attr-defined]
        except Exception:
            pass
        await self.state.broadcast(
            {
                "type": "arbitrage_execution",
                "canonical_key": key,
                "strategy": top.get("strategy"),
                "profit_estimate": top.get("profit_estimate"),
                "status": arb_status,
                "arb_stats": dict(self._arb_stats),
                "reports": [r.model_dump(mode="json") for r in reports],
            }
        )

    async def _execute_arbitrage_atomic(self, top: dict, intents: list[dict]) -> list[FillReport]:
        """Ejecuta arbitraje en modo atomic-safe.

        1) Ejecuta pierna 1.
        2) Ajusta pierna 2 al fill real de pierna 1.
        3) Si falla cobertura, intenta unwind defensivo de pierna 1.
        """
        reports: list[FillReport] = []
        first = intents[0]
        first_report = await self._submit_intent(first, top, reason="arb_leg1")
        reports.append(first_report)
        filled_leg1 = int(first_report.filled_contracts or 0)
        if not first_report.ok or filled_leg1 <= 0:
            return reports

        second = dict(intents[1])
        second["contracts"] = min(int(second.get("contracts", 0)), filled_leg1)
        if int(second["contracts"]) <= 0:
            return reports
        second_report = await self._submit_leg2_with_retry(second, top)
        reports.append(second_report)
        if second_report.ok and int(second_report.filled_contracts or 0) > 0:
            return reports

        # Hedge falló: unwind defensivo de la pierna 1 para limitar riesgo direccional.
        unwind = dict(first)
        unwind["side"] = "NO" if str(first.get("side")) == "YES" else "YES"
        unwind["contracts"] = filled_leg1
        unwind_report = await self._submit_intent(unwind, top, reason="arb_unwind")
        reports.append(unwind_report)
        return reports

    async def _submit_leg2_with_retry(self, second: dict, arb: dict) -> FillReport:
        attempts = max(0, self._arb_leg2_retries) + 1
        last: FillReport | None = None
        for idx in range(attempts):
            report = await self._submit_intent(second, arb, reason=f"arb_leg2_try{idx+1}")
            if report.ok and int(report.filled_contracts or 0) > 0:
                return report
            last = report
            if idx < attempts - 1:
                await asyncio.sleep(max(0, self._arb_leg2_retry_delay_ms) / 1000.0)
        return last or FillReport(ok=False, status="arb_leg2_no_attempt")

    async def _submit_intent(self, intent: dict, arb: dict, reason: str) -> FillReport:
        ticker = str(intent["ticker"])
        side = str(intent["side"])
        price = int(intent["price_cents"])
        contracts = int(intent["contracts"])
        coid = self.executor.make_client_order_id(
            ticker, side, price, contracts, ts_bucket_ms=500
        )
        order = OrderRequestV2(
            market_ticker=ticker,
            side=side,
            contracts=contracts,
            price_cents=price,
            client_order_id=coid,
            reason=reason,
        )
        report = await self.executor.submit(order)
        self.journal.write(
            "orders",
            {
                "order": order.model_dump(),
                "report": report.model_dump(mode="json"),
                "arb": arb,
                "arb_reason": reason,
            },
        )
        return report

    def _derive_arb_status(self, reports: list[FillReport]) -> str:
        if not reports:
            return "failed"
        leg1_ok = len(reports) >= 1 and reports[0].ok and int(reports[0].filled_contracts or 0) > 0
        leg2_ok = len(reports) >= 2 and reports[1].ok and int(reports[1].filled_contracts or 0) > 0
        unwind_ok = len(reports) >= 3 and reports[2].ok and int(reports[2].filled_contracts or 0) > 0
        if leg1_ok and leg2_ok:
            return "hedged"
        if leg1_ok and unwind_ok:
            return "unwinded"
        if leg1_ok:
            return "partial"
        return "failed"

    def _append_arb_activity(self, row: dict) -> None:
        self._arb_activity.append(row)
        self._arb_activity = self._arb_activity[-200:]
        # calcula latencia media de ejecución para panel operacional.
        if "reports" in row and isinstance(row["reports"], list):
            lats = [
                int(r.get("latency_ms", 0))
                for r in row["reports"]
                if isinstance(r, dict)
            ]
            row["avg_latency_ms"] = int(sum(lats) / max(1, len(lats))) if lats else 0
        try:
            self.state.arb_activity = list(self._arb_activity)  # type: ignore[attr-defined]
        except Exception:
            pass
        # Persistencia para análisis histórico y ventanas temporales.
        self.journal.write("arb_activity", row)

    async def _run_alert_checks(self) -> None:
        now = time.time()
        rows = self.journal.read("arb_activity", limit=5000)
        one_hour_ago = now - 3600
        scoped = [r for r in rows if self._row_ts(r) >= one_hour_ago]
        total = len(scoped)
        hedged = sum(1 for r in scoped if str(r.get("status", "")).lower() == "hedged")
        hedge_rate_1h = (hedged / total) if total else 0.0
        sent = await self.alerts.evaluate_and_alert(
            degraded=bool(self.health.degraded),
            hedge_rate_1h=hedge_rate_1h,
            total_1h=total,
        )
        if sent:
            self.journal.write(
                "alerts",
                {
                    "degraded": bool(self.health.degraded),
                    "hedge_rate_1h": hedge_rate_1h,
                    "total_1h": total,
                    "sent": sent,
                },
            )
            try:
                self.state.last_alerts = sent  # type: ignore[attr-defined]
            except Exception:
                pass

    @staticmethod
    def _row_ts(row: dict) -> float:
        ts = row.get("ts")
        if isinstance(ts, (int, float)):
            return float(ts)
        try:
            from datetime import datetime, timezone

            dt = datetime.fromisoformat(str(ts).replace("Z", "+00:00"))
            return dt.astimezone(timezone.utc).timestamp()
        except Exception:
            return 0.0

    # ------------------------------------------------------------------
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
            ts=ev.ts,
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

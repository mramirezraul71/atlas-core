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
from .exit_manager import ExitConfig, ExitManager, Position
from .executor_v2 import (ExecConfig, FillReport, KalshiExecutorV2,
                          OrderRequestV2)
from .gating import GateConfig, Gating
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
        self.executor = KalshiExecutorV2(self.settings, exec_cfg or ExecConfig(
            enable_live=os.getenv("ATLAS_RADAR_LIVE", "0") == "1",
            prefer_maker=os.getenv("RADAR_PREFER_MAKER", "1") == "1",
            max_chase_ticks=int(os.getenv("RADAR_MAX_CHASE", 1)),
            max_retries=int(os.getenv("RADAR_MAX_RETRIES", 3)),
        ))
        self.journal = Journal(self.settings.log_dir)
        self.health = Health()
        self._stop = asyncio.Event()

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

        watchdog = asyncio.create_task(self._watchdog(), name="radar-watchdog")
        try:
            async for ev in self.scanner.stream():
                await self._on_event(ev)
                if self._stop.is_set():
                    break
        finally:
            watchdog.cancel()

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
            stale = time.time() - max(self.health.last_event_ts, 1) > 60
            self.health.degraded = stale
            # publicar al state para que /api/radar/{metrics,risk,health} sirvan datos
            try:
                self.state.exec_metrics = self.executor.metrics.to_dict()  # type: ignore[attr-defined]
                self.state.risk_status = self.risk.status()  # type: ignore[attr-defined]
                self.state.health = self.health  # type: ignore[attr-defined]
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

    # ------------------------------------------------------------------
    async def _on_event(self, ev: MarketEvent) -> None:
        self.health.tick("event")
        if ev.kind == "new_market":
            self.brain.update_market_meta(ev.market_ticker, ev.payload)
            self.state.update_market(ev.market_ticker, {
                "title": ev.payload.get("title"),
                "close_time": ev.payload.get("close_time"),
            })
            return
        if ev.kind != "orderbook":
            return

        book = OrderBookSnapshot(**ev.payload)
        history = self.scanner.history(ev.market_ticker)

        # 1) brain (LLM + MC) — devuelve también p_market
        decision = await self.brain.evaluate(ev.market_ticker, book, history)
        # 2) ensemble + calibración
        readout = self.ensemble.evaluate(book, history,
                                         p_llm=decision.p_model,
                                         llm_confidence=decision.confidence)
        p_calibrated = self.calibrator.predict(readout.p_ensemble)
        readout = readout.model_copy(update={"p_ensemble": p_calibrated})
        self.health.tick("decision")
        self.journal.write("decisions", {
            "ticker": ev.market_ticker,
            "decision": decision.model_dump(mode="json"),
            "readout": readout.model_dump(mode="json"),
        })

        # 3) gestionar salidas primero (tiene prioridad)
        await self._manage_exits(ev.market_ticker, book, readout)

        # 4) gating sobre nueva entrada
        gate = self.gating.evaluate(
            ev.market_ticker, readout, p_market=book.yes_mid or 0.5,
            quote_age_ms=int((time.time() - ev.ts.timestamp()) * 1000),
            latency_ms=0,
        )

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

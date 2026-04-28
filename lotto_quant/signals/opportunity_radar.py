"""
lotto_quant.signals.opportunity_radar
=====================================

Main orchestration loop for Atlas Lotto-Quant.

Pipeline (per cycle)
--------------------
    1.  NCEL scrape (primary + secondary cross-check)
    2.  EV calculation across every game
    3.  Markov anomaly analysis on the top-N depletion candidates
    4.  Jackpot EV refresh (Powerball / Mega Millions)
    5.  Signal gate (hysteresis + thresholds)
    6.  Kelly sizing
    7.  Alert dispatch
    8.  Persistence to DuckDB
    9.  Sleep until next cycle
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import signal as sigmod
from datetime import datetime
from typing import Any, Dict, List, Optional

from .. import config
from ..data.database import LottoQuantDB
from ..data.ncel_scraper import NCELScraper
from ..data.scratchsmarter_scraper import ScratchOddsScraper
from ..models.ev_calculator import EVCalculator, EVResult, ScratchOffGame
from ..models.jackpot_simulator import JackpotSimulator
from ..models.kelly_allocator import KellyAllocator
from ..models.markov_scratchoff import GameState, ScratchOffMarkovModel
from .alert_engine import AlertEngine
from .signal_gate import SignalGate

logger = logging.getLogger(__name__)


class LottoQuantRadar:
    """Background radar loop. Designed to be embedded in atlas-core."""

    def __init__(
        self,
        bankroll: float,
        db_path: Optional[str] = None,
        config_override: Optional[Dict[str, Any]] = None,
    ):
        if bankroll <= 0:
            raise ValueError("bankroll must be > 0")
        self.bankroll = bankroll
        self.config_override = config_override or {}
        self.db = LottoQuantDB(db_path)
        self.ev_calc = EVCalculator()
        self.kelly = KellyAllocator()
        self.jackpot_sim = JackpotSimulator(self.ev_calc)
        self.gate = SignalGate()
        self.alerts = AlertEngine(db=self.db)
        self._running = False
        self._cycle_count = 0
        self._current_exposure = 0.0

    # ── single cycle ───────────────────────────────────────────────
    async def run_scan_cycle(self) -> List[Dict[str, Any]]:
        """Execute one complete scan cycle. Returns list of fired signals."""
        self._cycle_count += 1
        logger.info("=== Lotto-Quant cycle #%d @ %s ===",
                    self._cycle_count, datetime.utcnow().isoformat())
        fired: List[Dict[str, Any]] = []

        # 1. Scrape NCEL
        async with NCELScraper() as primary:
            games = await primary.fetch_all_games()
            jackpots = await primary.fetch_draw_game_jackpots()

        # 1b. Cross-check with secondary
        try:
            secondary = await ScratchOddsScraper().fetch_all_games()
            recon = ScratchOddsScraper.reconcile(games, secondary)
            logger.info(
                "Reconciliation: primary=%d secondary=%d common=%d discrepancies=%d",
                recon["n_primary"], recon["n_secondary"],
                recon["n_common"], len(recon["discrepancies"]),
            )
        except Exception as e:  # pragma: no cover
            logger.warning("Secondary reconciliation failed: %s", e)

        # 2. Process scratch-offs
        for game in games:
            try:
                fired.extend(await self._process_scratch_off(game))
            except Exception as e:
                logger.exception("Error processing %s: %s", game.game_id, e)

        # 4. Process jackpots
        for name, jp in jackpots.items():
            try:
                fired.extend(await self._process_jackpot(name, jp.jackpot_amount))
            except Exception as e:
                logger.exception("Error processing %s jackpot: %s", name, e)

        logger.info("Cycle #%d done — %d signals fired", self._cycle_count, len(fired))
        return fired

    async def _process_scratch_off(self, game: ScratchOffGame) -> List[Dict[str, Any]]:
        ev = self.ev_calc.calculate_adjusted_ev(game)

        self.db.insert_snapshot(
            game_id=ev.game_id,
            game_name=ev.game_name,
            ticket_price=ev.ticket_price,
            data={"prize_tiers": [t.__dict__ for t in game.prize_tiers]},
            ev_gross=ev.gross_ev,
            ev_adjusted=ev.adjusted_ev_nc,
            depletion_ratio=ev.depletion_ratio,
            anomaly_score=ev.anomaly_score,
        )

        if ev.is_anomaly:
            try:
                model = ScratchOffMarkovModel(game, n_simulations=2_000)
                tickets_to_pos, conf = model.predict_ev_turning_point(max_horizon=20_000)
                self.db.insert_markov_prediction(
                    game_id=ev.game_id,
                    tickets_until_ev_positive=tickets_to_pos,
                    confidence=conf,
                    anomaly_score=ev.anomaly_score,
                )
                ev.diagnostics["markov_tickets_to_positive"] = tickets_to_pos
                ev.diagnostics["markov_confidence"] = conf
            except Exception as e:
                logger.warning("Markov modeling failed for %s: %s", ev.game_id, e)

        decision = self.gate.evaluate(ev, signal_type="STALE_PRIZE")
        if not decision.pass_through:
            logger.debug("Gate rejected %s: %s", ev.game_id, decision.reason)
            return []

        rec = self.kelly.calculate_lottery_kelly(
            ev_result=ev,
            bankroll=self.bankroll,
            current_total_exposure=self._current_exposure,
            game=game,
        )
        self.db.insert_kelly(
            game_id=ev.game_id,
            bankroll=self.bankroll,
            full_kelly=rec.full_kelly,
            final_fraction=rec.final_fraction,
            position_usd=rec.recommended_position,
            n_tickets=rec.n_tickets,
            capped_by=rec.capped_by,
        )

        self.db.insert_signal(
            game_id=ev.game_id,
            signal_type=decision.signal_type,
            signal_strength=decision.signal_strength,
            ev_net=ev.adjusted_ev_nc,
            recommended_allocation=rec.recommended_position,
            alert_sent=False,
        )
        await self.alerts.dispatch(
            ev=ev,
            signal_type=decision.signal_type,
            recommended_position=rec.recommended_position,
        )
        self._current_exposure += rec.recommended_position
        return [{
            "game_id": ev.game_id,
            "game_name": ev.game_name,
            "signal_type": decision.signal_type,
            "signal_strength": ev.signal_strength,
            "ev_per_dollar": ev.ev_per_dollar,
            "recommended_position": rec.recommended_position,
            "n_tickets": rec.n_tickets,
        }]

    async def _process_jackpot(
        self, name: str, jackpot: float
    ) -> List[Dict[str, Any]]:
        threshold = (
            config.POWERBALL_EV_POSITIVE_JACKPOT
            if name == "powerball"
            else config.MEGA_MILLIONS_EV_POSITIVE_JACKPOT
        )
        if jackpot < threshold * 0.6:
            # Too small to bother modelling
            return []

        # Approximate ticket-sales volume from jackpot size (rough heuristic)
        n_tickets_est = max(50_000_000, jackpot / 4.0)

        if name == "powerball":
            res = self.jackpot_sim.powerball(jackpot, n_tickets_est, n_simulations=20_000)
        else:
            res = self.jackpot_sim.mega_millions(jackpot, n_tickets_est, n_simulations=20_000)

        if res.ev_per_dollar <= config.MIN_EV_NET_POSITIVE:
            return []

        # Create a synthetic EVResult so we can reuse the alert pipeline
        synthetic_ev = EVResult(
            game_id=name.upper(),
            game_name=res.game_name,
            ticket_price=res.ev_per_ticket_adjusted + res.diagnostics.get("ticket_price", 2.0),
            gross_ev=res.ev_per_ticket_gross,
            adjusted_ev_nc=res.ev_per_ticket_adjusted,
            ev_per_dollar=res.ev_per_dollar,
            signal_strength=res.signal_strength,
            depletion_ratio=0.0,
            major_prize_retention=1.0,
            anomaly_score=0.0,
            is_anomaly=False,
            diagnostics={
                "jackpot": jackpot,
                "expected_winners": res.expected_winners,
                "breakeven_prob": res.monte_carlo_breakeven_prob,
            },
        )
        decision = self.gate.evaluate(synthetic_ev, signal_type="JACKPOT_EV_POSITIVE")
        if not decision.pass_through:
            return []
        await self.alerts.dispatch(synthetic_ev, signal_type="JACKPOT_EV_POSITIVE")
        self.db.insert_signal(
            game_id=name.upper(),
            signal_type="JACKPOT_EV_POSITIVE",
            signal_strength=res.signal_strength,
            ev_net=res.ev_per_ticket_adjusted,
            recommended_allocation=0.0,
            alert_sent=True,
        )
        return [{
            "game_id": name.upper(),
            "game_name": res.game_name,
            "signal_type": "JACKPOT_EV_POSITIVE",
            "ev_per_dollar": res.ev_per_dollar,
            "expected_winners": res.expected_winners,
        }]

    # ── main loop ──────────────────────────────────────────────────
    async def start(self, interval_seconds: int = config.SCRAPE_INTERVAL_SECONDS):
        self._running = True
        logger.info("Lotto-Quant radar started — interval=%ds", interval_seconds)
        try:
            while self._running:
                try:
                    await self.run_scan_cycle()
                except Exception as e:
                    logger.exception("Cycle failed: %s", e)
                await asyncio.sleep(interval_seconds)
        finally:
            self.db.close()
            logger.info("Lotto-Quant radar stopped.")

    def stop(self) -> None:
        logger.info("Stop requested.")
        self._running = False


# ─────────────────────────────────────────────────────────────────────
# CLI entry point
# ─────────────────────────────────────────────────────────────────────
def _setup_logging() -> None:
    logging.basicConfig(level=getattr(logging, config.LOG_LEVEL, logging.INFO),
                        format=config.LOG_FORMAT)


def main() -> None:
    parser = argparse.ArgumentParser(description="Atlas Lotto-Quant radar")
    parser.add_argument("--bankroll", type=float, required=True)
    parser.add_argument("--interval", type=int, default=config.SCRAPE_INTERVAL_SECONDS)
    parser.add_argument("--db", type=str, default=None)
    parser.add_argument("--once", action="store_true", help="Run a single cycle and exit")
    args = parser.parse_args()

    _setup_logging()

    radar = LottoQuantRadar(bankroll=args.bankroll, db_path=args.db)

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    def _signal_handler(*_):
        radar.stop()

    for s in (sigmod.SIGINT, sigmod.SIGTERM):
        try:
            loop.add_signal_handler(s, _signal_handler)
        except NotImplementedError:
            # Windows: signal handlers in event loop are unsupported
            pass

    if args.once:
        loop.run_until_complete(radar.run_scan_cycle())
    else:
        loop.run_until_complete(radar.start(interval_seconds=args.interval))


if __name__ == "__main__":
    main()

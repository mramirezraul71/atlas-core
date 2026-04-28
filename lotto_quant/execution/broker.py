"""
lotto_quant.execution.broker
============================

Broker abstractions for the two operating modes.

PaperBroker
-----------
Simulates ticket purchases and outcome draws using the **same** prize-tier
distribution stored in the radar's snapshot. We sample one outcome per ticket
from the empirical distribution
    p_i = remaining_prizes_i / remaining_tickets,
applying the tax model from EVCalculator.

LiveBroker
----------
Atlas does **not** auto-execute lottery purchases. The LiveBroker records the
intent (a "ticket" of a recommendation) and exposes a `confirm_outcome()` API
that a human (or the OCR pipeline) calls when the physical ticket is settled.

Both brokers persist Orders and Fills to DuckDB through `LottoQuantDB`.
"""

from __future__ import annotations

import logging
import uuid
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import List, Optional

import numpy as np

from ..data.database import LottoQuantDB
from ..models.ev_calculator import EVCalculator, ScratchOffGame
from .modes import OperatingMode

logger = logging.getLogger(__name__)


# ─────────────────────────────────────────────────────────────────────
# Data classes
# ─────────────────────────────────────────────────────────────────────
@dataclass
class Order:
    order_id: str
    game_id: str
    game_name: str
    ticket_price: float
    n_tickets: int
    expected_ev: float
    mode: OperatingMode
    created_iso: str
    status: str = "OPEN"  # 'OPEN' | 'FILLED' | 'CONFIRMED' | 'CANCELLED'

    def cost(self) -> float:
        return self.ticket_price * self.n_tickets


@dataclass
class Fill:
    fill_id: str
    order_id: str
    game_id: str
    n_tickets: int
    cost: float
    gross_payout: float    # before tax
    net_payout: float      # after NC + federal tax
    pnl_net: float         # net_payout − cost
    mode: OperatingMode
    filled_iso: str
    detail_per_ticket: List[float] = field(default_factory=list)


# ─────────────────────────────────────────────────────────────────────
# Schema extension (idempotent)
# ─────────────────────────────────────────────────────────────────────
_BROKER_TABLES_SQL = [
    """
    CREATE TABLE IF NOT EXISTS exec_orders (
        order_id      VARCHAR PRIMARY KEY,
        game_id       VARCHAR,
        game_name     VARCHAR,
        ticket_price  DOUBLE,
        n_tickets     INTEGER,
        expected_ev   DOUBLE,
        mode          VARCHAR,
        created_iso   VARCHAR,
        status        VARCHAR
    )
    """,
    """
    CREATE TABLE IF NOT EXISTS exec_fills (
        fill_id       VARCHAR PRIMARY KEY,
        order_id      VARCHAR,
        game_id       VARCHAR,
        n_tickets     INTEGER,
        cost          DOUBLE,
        gross_payout  DOUBLE,
        net_payout    DOUBLE,
        pnl_net       DOUBLE,
        mode          VARCHAR,
        filled_iso    VARCHAR
    )
    """,
]


def ensure_broker_tables(db: LottoQuantDB) -> None:
    cur = db._cursor()
    for sql in _BROKER_TABLES_SQL:
        cur.execute(sql)
    db._commit()


# ─────────────────────────────────────────────────────────────────────
# Base
# ─────────────────────────────────────────────────────────────────────
class BrokerBase(ABC):
    """Abstract broker. Both paper and live implement the same interface."""

    mode: OperatingMode

    def __init__(self, db: LottoQuantDB):
        self.db = db
        ensure_broker_tables(self.db)

    @abstractmethod
    def submit_order(
        self,
        *,
        game: ScratchOffGame,
        n_tickets: int,
        expected_ev: float,
    ) -> Order: ...

    @abstractmethod
    def settle(self, order: Order) -> Optional[Fill]: ...

    # ── persistence helpers ──────────────────────────────────────
    def _persist_order(self, o: Order) -> None:
        self.db._cursor().execute(
            "INSERT INTO exec_orders VALUES (?,?,?,?,?,?,?,?,?)",
            (
                o.order_id, o.game_id, o.game_name, o.ticket_price, o.n_tickets,
                o.expected_ev, o.mode.value, o.created_iso, o.status,
            ),
        )
        self.db._commit()

    def _update_order_status(self, order_id: str, status: str) -> None:
        self.db._cursor().execute(
            "UPDATE exec_orders SET status = ? WHERE order_id = ?",
            (status, order_id),
        )
        self.db._commit()

    def _persist_fill(self, f: Fill) -> None:
        self.db._cursor().execute(
            "INSERT INTO exec_fills VALUES (?,?,?,?,?,?,?,?,?,?)",
            (
                f.fill_id, f.order_id, f.game_id, f.n_tickets, f.cost,
                f.gross_payout, f.net_payout, f.pnl_net, f.mode.value, f.filled_iso,
            ),
        )
        self.db._commit()

    @staticmethod
    def _now_iso() -> str:
        return datetime.now(timezone.utc).isoformat(timespec="seconds")


# ─────────────────────────────────────────────────────────────────────
# Paper broker
# ─────────────────────────────────────────────────────────────────────
class PaperBroker(BrokerBase):
    """Simulates ticket purchases by sampling from the prize distribution."""

    mode = OperatingMode.PAPER

    def __init__(self, db: LottoQuantDB, rng_seed: Optional[int] = None):
        super().__init__(db)
        self.rng = np.random.default_rng(rng_seed)
        self.calc = EVCalculator()

    def submit_order(
        self,
        *,
        game: ScratchOffGame,
        n_tickets: int,
        expected_ev: float,
    ) -> Order:
        if n_tickets <= 0:
            raise ValueError("n_tickets must be > 0")
        order = Order(
            order_id=str(uuid.uuid4()),
            game_id=game.game_id,
            game_name=game.name,
            ticket_price=game.ticket_price,
            n_tickets=n_tickets,
            expected_ev=expected_ev,
            mode=self.mode,
            created_iso=self._now_iso(),
            status="OPEN",
        )
        self._persist_order(order)
        # Paper mode: we settle immediately
        fill = self.settle(order, game=game)
        return order

    def settle(
        self,
        order: Order,
        *,
        game: Optional[ScratchOffGame] = None,
    ) -> Optional[Fill]:
        if game is None:
            logger.warning("PaperBroker.settle requires the game snapshot")
            return None

        remaining = self.calc.calculate_remaining_tickets(game)
        remaining = max(remaining, sum(t.remaining_prizes for t in game.prize_tiers) + 1)

        # Probability vector over (each prize tier) + blank
        n_tiers = len(game.prize_tiers)
        probs = np.zeros(n_tiers + 1, dtype=np.float64)
        for i, tier in enumerate(game.prize_tiers):
            probs[i] = tier.remaining_prizes / remaining
        probs[n_tiers] = 1.0 - probs[:n_tiers].sum()
        probs = np.clip(probs, 0.0, 1.0)
        if probs.sum() <= 0:
            probs = np.zeros_like(probs)
            probs[-1] = 1.0
        probs /= probs.sum()

        outcomes = self.rng.choice(n_tiers + 1, size=order.n_tickets, p=probs)
        per_ticket_gross: List[float] = []
        per_ticket_net: List[float] = []
        for o in outcomes:
            if o < n_tiers:
                v = float(game.prize_tiers[o].value)
                per_ticket_gross.append(v)
                per_ticket_net.append(self.calc.adjust_prize_for_taxes(v))
            else:
                per_ticket_gross.append(0.0)
                per_ticket_net.append(0.0)

        gross_payout = float(sum(per_ticket_gross))
        net_payout = float(sum(per_ticket_net))
        cost = order.cost()
        pnl_net = net_payout - cost

        fill = Fill(
            fill_id=str(uuid.uuid4()),
            order_id=order.order_id,
            game_id=order.game_id,
            n_tickets=order.n_tickets,
            cost=cost,
            gross_payout=gross_payout,
            net_payout=net_payout,
            pnl_net=pnl_net,
            mode=self.mode,
            filled_iso=self._now_iso(),
            detail_per_ticket=per_ticket_net,
        )
        self._persist_fill(fill)
        order.status = "FILLED"
        self._update_order_status(order.order_id, "FILLED")
        logger.info(
            "PAPER FILL %s: %d×$%.2f → gross=$%.2f net=$%.2f pnl=$%.2f",
            order.game_name, order.n_tickets, order.ticket_price,
            gross_payout, net_payout, pnl_net,
        )
        return fill


# ─────────────────────────────────────────────────────────────────────
# Live broker
# ─────────────────────────────────────────────────────────────────────
class LiveBroker(BrokerBase):
    """
    Records purchase intents in LIVE mode. A human must confirm the actual
    purchase + outcome via `confirm_outcome()`. Atlas never auto-executes.
    """

    mode = OperatingMode.LIVE

    def submit_order(
        self,
        *,
        game: ScratchOffGame,
        n_tickets: int,
        expected_ev: float,
    ) -> Order:
        if n_tickets <= 0:
            raise ValueError("n_tickets must be > 0")
        order = Order(
            order_id=str(uuid.uuid4()),
            game_id=game.game_id,
            game_name=game.name,
            ticket_price=game.ticket_price,
            n_tickets=n_tickets,
            expected_ev=expected_ev,
            mode=self.mode,
            created_iso=self._now_iso(),
            status="OPEN",
        )
        self._persist_order(order)
        logger.warning(
            "LIVE INTENT recorded: %s ×%d @ $%.2f. "
            "Awaiting human confirmation via confirm_outcome().",
            order.game_name, n_tickets, order.ticket_price,
        )
        return order

    def settle(self, order: Order) -> Optional[Fill]:
        # In live mode, settle is a no-op until confirm_outcome() is called.
        return None

    # ── helpers for HUD form ─────────────────────────────────────
    def list_open_orders(self, limit: int = 50) -> List[Order]:
        """Return LIVE orders that are still OPEN (awaiting human confirmation)."""
        cur = self.db._cursor()
        cur.execute(
            "SELECT order_id, game_id, game_name, ticket_price, n_tickets, "
            "expected_ev, mode, created_iso, status "
            "FROM exec_orders WHERE mode = ? AND status = 'OPEN' "
            "ORDER BY created_iso DESC LIMIT ?",
            (self.mode.value, limit),
        )
        rows = cur.fetchall()
        out: List[Order] = []
        for r in rows:
            out.append(Order(
                order_id=r[0], game_id=r[1], game_name=r[2],
                ticket_price=float(r[3]), n_tickets=int(r[4]),
                expected_ev=float(r[5]), mode=OperatingMode.from_string(r[6]),
                created_iso=str(r[7]), status=str(r[8]),
            ))
        return out

    def get_order(self, order_id: str) -> Optional[Order]:
        """Fetch a single order by id (any status)."""
        cur = self.db._cursor()
        cur.execute(
            "SELECT order_id, game_id, game_name, ticket_price, n_tickets, "
            "expected_ev, mode, created_iso, status "
            "FROM exec_orders WHERE order_id = ?",
            (order_id,),
        )
        r = cur.fetchone()
        if not r:
            return None
        return Order(
            order_id=r[0], game_id=r[1], game_name=r[2],
            ticket_price=float(r[3]), n_tickets=int(r[4]),
            expected_ev=float(r[5]), mode=OperatingMode.from_string(r[6]),
            created_iso=str(r[7]), status=str(r[8]),
        )

    def cancel_order(self, order_id: str) -> bool:
        """Mark a LIVE order CANCELLED (e.g. user decided not to buy the ticket)."""
        order = self.get_order(order_id)
        if not order or order.status != "OPEN":
            return False
        self._update_order_status(order_id, "CANCELLED")
        logger.info("LIVE order %s cancelled by user", order_id)
        return True

    def confirm_outcome(
        self,
        order: Order,
        *,
        gross_payout: float,
    ) -> Fill:
        """Human-confirmed outcome. Tax adjustment computed by Atlas."""
        calc = EVCalculator()
        net_payout = calc.adjust_prize_for_taxes(gross_payout)
        cost = order.cost()
        pnl_net = net_payout - cost
        fill = Fill(
            fill_id=str(uuid.uuid4()),
            order_id=order.order_id,
            game_id=order.game_id,
            n_tickets=order.n_tickets,
            cost=cost,
            gross_payout=gross_payout,
            net_payout=net_payout,
            pnl_net=pnl_net,
            mode=self.mode,
            filled_iso=self._now_iso(),
        )
        self._persist_fill(fill)
        order.status = "CONFIRMED"
        self._update_order_status(order.order_id, "CONFIRMED")
        logger.info(
            "LIVE CONFIRMED %s: gross=$%.2f net=$%.2f pnl=$%.2f",
            order.game_name, gross_payout, net_payout, pnl_net,
        )
        return fill


# ─────────────────────────────────────────────────────────────────────
# Factory
# ─────────────────────────────────────────────────────────────────────
def get_broker(mode: OperatingMode, db: LottoQuantDB) -> BrokerBase:
    if mode == OperatingMode.PAPER:
        return PaperBroker(db)
    return LiveBroker(db)

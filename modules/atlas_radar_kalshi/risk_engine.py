"""
risk_engine.py — Risk Engine institucional.

Funciones:

1. **Sizing** vía Kelly fraccionario con hard caps:
   - ``max_position_pct``
   - ``max_market_exposure_pct`` (suma por ticker)
   - ``max_total_exposure_pct`` (suma global)
2. **Guardrails diarios/semanales**:
   - ``daily_dd_limit_pct``
   - ``weekly_dd_limit_pct``
   - ``max_consecutive_losses``
   - ``max_open_positions``
   - ``max_orders_per_minute``
3. **Circuit breakers** (`safe_mode`): cuando se dispara, se prohibe
   abrir nuevas posiciones; sólo se permite gestionar salidas.
4. **Kill-switch global** vía env ``ATLAS_RADAR_KILL=1`` o flag.
"""
from __future__ import annotations

import os
import time
from collections import defaultdict, deque
from dataclasses import dataclass, field
from typing import Deque, Optional

from pydantic import BaseModel, Field

from .gating import GateDecision
from .signals import SignalReadout


# ===========================================================================
class RiskLimits(BaseModel):
    max_position_pct: float = 0.05
    max_market_exposure_pct: float = 0.10
    max_total_exposure_pct: float = 0.50
    # Tope de notional abierto agregado por venue (multivenue).
    max_kalshi_venue_exposure_pct: float = 0.50
    max_polymarket_venue_exposure_pct: float = 0.50
    daily_dd_limit_pct: float = 0.05
    weekly_dd_limit_pct: float = 0.10
    max_consecutive_losses: int = 5
    max_open_positions: int = 8
    max_orders_per_minute: int = 30
    kelly_fraction: float = 0.25


class SizingDecision(BaseModel):
    market_ticker: str
    side: str
    price_cents: int
    contracts: int
    notional_cents: int
    f_full: float
    f_capped: float
    rationale: str
    safe_mode: bool = False


# ===========================================================================
@dataclass
class RiskState:
    balance_cents: int = 0
    equity_high_day: int = 0
    equity_high_week: int = 0
    realized_pnl_day: int = 0
    realized_pnl_week: int = 0
    unrealized_pnl_cents: int = 0
    consecutive_losses: int = 0
    open_positions: dict[str, int] = field(default_factory=dict)  # ticker -> notional
    order_times: Deque[float] = field(default_factory=lambda: deque(maxlen=120))
    safe_mode: bool = False
    kill_switch: bool = False
    breakers: list[str] = field(default_factory=list)
    day_start: float = field(default_factory=time.time)
    week_start: float = field(default_factory=time.time)

    @property
    def total_exposure_cents(self) -> int:
        return sum(self.open_positions.values())

    @property
    def equity_cents(self) -> int:
        return int(self.balance_cents + self.realized_pnl_day + self.unrealized_pnl_cents)

    @property
    def drawdown_day_cents(self) -> int:
        s = self
        if s.equity_high_day <= 0:
            return 0
        return max(0, int(s.equity_high_day) - int(s.equity_cents))

    @property
    def drawdown_week_cents(self) -> int:
        s = self
        if s.equity_high_week <= 0:
            return 0
        return max(0, int(s.equity_high_week) - int(s.equity_cents))


# ===========================================================================
def venue_of_ticker(ticker: str) -> str:
    t = (ticker or "").strip().upper()
    if t.startswith("POLY:"):
        return "polymarket"
    return "kalshi"


# ===========================================================================
class RiskEngine:
    """Aplica todas las reglas. Stateless en config; estado en :class:`RiskState`."""

    def __init__(self, limits: Optional[RiskLimits] = None,
                 state: Optional[RiskState] = None) -> None:
        self.limits = limits or RiskLimits()
        self.state = state or RiskState()

    def exposure_by_venue_cents(self) -> dict[str, int]:
        out: dict[str, int] = defaultdict(int)
        for t, cents in self.state.open_positions.items():
            out[venue_of_ticker(t)] += int(cents)
        return {k: int(v) for k, v in out.items()}

    # ------------------------------------------------------------------
    def kill(self, reason: str = "manual") -> None:
        self.state.kill_switch = True
        self.state.breakers.append(f"KILL:{reason}")

    def reset_kill(self) -> None:
        self.state.kill_switch = False

    # ------------------------------------------------------------------
    def _check_breakers(self) -> Optional[str]:
        s, l = self.state, self.limits
        if s.kill_switch or os.getenv("ATLAS_RADAR_KILL", "0") == "1":
            return "kill_switch"
        # daily DD
        cur_equity = max(s.equity_cents, 0)
        if s.equity_high_day > 0:
            dd = (s.equity_high_day - cur_equity) / s.equity_high_day
            if dd >= l.daily_dd_limit_pct:
                return f"daily_dd={dd:.3f}"
        if s.equity_high_week > 0:
            dd = (s.equity_high_week - cur_equity) / s.equity_high_week
            if dd >= l.weekly_dd_limit_pct:
                return f"weekly_dd={dd:.3f}"
        if s.consecutive_losses >= l.max_consecutive_losses:
            return f"losses={s.consecutive_losses}"
        if len([k for k, v in s.open_positions.items() if v > 0]) >= l.max_open_positions:
            return "max_open"
        # rate limit
        now = time.time()
        recent = sum(1 for t in s.order_times if now - t <= 60)
        if recent >= l.max_orders_per_minute:
            return "rate_limit"
        return None

    def status(self) -> dict:
        """Snapshot completo del estado de riesgo (consumido por dashboard)."""
        s = self.state
        return {
            "safe_mode": s.safe_mode,
            "kill_switch": s.kill_switch,
            "breakers": list(s.breakers)[-10:],
            "balance_cents": s.balance_cents,
            "exposure_cents": s.total_exposure_cents,
            "exposure_by_venue": self.exposure_by_venue_cents(),
            "open_positions": dict(s.open_positions),
            "consecutive_losses": s.consecutive_losses,
            "realized_pnl_day": s.realized_pnl_day,
            "realized_pnl_week": s.realized_pnl_week,
            "unrealized_pnl_cents": s.unrealized_pnl_cents,
            "equity_cents": s.equity_cents,
            "equity_high_day": s.equity_high_day,
            "equity_high_week": s.equity_high_week,
            "drawdown_day_cents": s.drawdown_day_cents,
            "drawdown_week_cents": s.drawdown_week_cents,
            "limits": self.limits.model_dump(),
        }

    # ------------------------------------------------------------------
    @staticmethod
    def kelly(p: float, b: float) -> float:
        if b <= 0:
            return 0.0
        f = (p * (b + 1.0) - 1.0) / b
        return max(0.0, min(1.0, f))

    # ------------------------------------------------------------------
    def size(self, gate: GateDecision, readout: SignalReadout,
             ticker: str) -> SizingDecision:
        """Devuelve un :class:`SizingDecision`. Si breakers activos -> 0."""
        breaker = self._check_breakers()
        if breaker:
            self.state.safe_mode = True
            if breaker not in self.state.breakers[-3:]:
                self.state.breakers.append(breaker)
            return SizingDecision(
                market_ticker=ticker, side=gate.side or "YES",
                price_cents=gate.price_cents, contracts=0, notional_cents=0,
                f_full=0.0, f_capped=0.0,
                rationale=f"safe_mode:{breaker}", safe_mode=True,
            )

        if not gate.accepted or gate.side is None:
            return SizingDecision(
                market_ticker=ticker, side=gate.side or "YES",
                price_cents=gate.price_cents, contracts=0, notional_cents=0,
                f_full=0.0, f_capped=0.0,
                rationale=f"gate:{gate.reason}",
            )

        c = max(1, min(99, gate.price_cents))
        b = (100 - c) / c
        p = readout.p_ensemble if gate.side == "YES" else (1 - readout.p_ensemble)
        f_full = self.kelly(p, b)
        f_frac = f_full * self.limits.kelly_fraction
        f_pos = min(f_frac, self.limits.max_position_pct)

        # caps de exposición agregada
        bal = max(1, self.state.balance_cents)
        market_exposure = self.state.open_positions.get(ticker, 0)
        market_cap = self.limits.max_market_exposure_pct * bal - market_exposure
        total_cap = self.limits.max_total_exposure_pct * bal - self.state.total_exposure_cents
        vkey = venue_of_ticker(ticker)
        v_used = self.exposure_by_venue_cents().get(vkey, 0)
        vlim = (
            self.limits.max_polymarket_venue_exposure_pct
            if vkey == "polymarket"
            else self.limits.max_kalshi_venue_exposure_pct
        )
        venue_cap = vlim * bal - v_used
        cap_cents = max(0, int(min(f_pos * bal, market_cap, total_cap, venue_cap)))

        contracts = cap_cents // c if c > 0 else 0
        notional = contracts * c
        rationale = (
            f"p={p:.3f} b={b:.3f} f*={f_full:.4f} frac={f_frac:.4f} "
            f"pos_cap={f_pos:.4f} mkt_cap={market_cap} tot_cap={total_cap} "
            f"venue={vkey} v_cap={venue_cap} "
            f"-> {contracts}@{c}¢"
        )
        return SizingDecision(
            market_ticker=ticker, side=gate.side, price_cents=c,
            contracts=int(contracts), notional_cents=int(notional),
            f_full=f_full, f_capped=f_pos, rationale=rationale,
        )

    # ------------------------------------------------------------------
    def on_order(self, ticker: str, notional_cents: int) -> None:
        self.state.order_times.append(time.time())
        self.state.open_positions[ticker] = (
            self.state.open_positions.get(ticker, 0) + notional_cents
        )

    def on_close(self, ticker: str, notional_cents: int, pnl_cents: int) -> None:
        self.state.open_positions[ticker] = max(
            0, self.state.open_positions.get(ticker, 0) - notional_cents
        )
        self.state.realized_pnl_day += pnl_cents
        self.state.realized_pnl_week += pnl_cents
        if pnl_cents < 0:
            self.state.consecutive_losses += 1
        else:
            self.state.consecutive_losses = 0
        # Marcas de equity con PnL realizado + no realizado (curva completa).
        eq = self.state.equity_cents
        self.state.equity_high_day = max(self.state.equity_high_day, eq)
        self.state.equity_high_week = max(self.state.equity_high_week, eq)

    def update_balance(self, balance_cents: int) -> None:
        self.state.balance_cents = balance_cents
        self.state.equity_high_day = max(self.state.equity_high_day, self.state.equity_cents)
        self.state.equity_high_week = max(self.state.equity_high_week, self.state.equity_cents)

    def mark_unrealized(self, pnl_cents: int) -> None:
        self.state.unrealized_pnl_cents = int(pnl_cents)
        eq = self.state.equity_cents
        self.state.equity_high_day = max(self.state.equity_high_day, eq)
        self.state.equity_high_week = max(self.state.equity_high_week, eq)

    # ------------------------------------------------------------------
    def roll_day(self) -> None:
        self.state.realized_pnl_day = 0
        self.state.unrealized_pnl_cents = 0
        self.state.equity_high_day = self.state.equity_cents
        self.state.day_start = time.time()
        self.state.consecutive_losses = 0
        self.state.safe_mode = False
        self.state.breakers.clear()

    def roll_week(self) -> None:
        self.state.realized_pnl_week = 0
        self.state.equity_high_week = self.state.equity_cents
        self.state.week_start = time.time()

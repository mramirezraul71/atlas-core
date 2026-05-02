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
from collections import deque
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
    consecutive_losses: int = 0
    open_positions: dict[str, int] = field(default_factory=dict)  # ticker -> notional
    order_times: Deque[float] = field(default_factory=lambda: deque(maxlen=120))
    safe_mode: bool = False
    kill_switch: bool = False
    breakers: list[str] = field(default_factory=list)
    safe_mode_reason: str = ""
    manual_recover_requested: bool = False
    probation_until_ts: float = 0.0
    day_start: float = field(default_factory=time.time)
    week_start: float = field(default_factory=time.time)

    @property
    def total_exposure_cents(self) -> int:
        return sum(self.open_positions.values())


# ===========================================================================
class RiskEngine:
    """Aplica todas las reglas. Stateless en config; estado en :class:`RiskState`."""

    def __init__(self, limits: Optional[RiskLimits] = None,
                 state: Optional[RiskState] = None) -> None:
        self.limits = limits or RiskLimits()
        self.state = state or RiskState()

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
        if s.equity_high_day > 0:
            dd = (s.equity_high_day - max(s.balance_cents, 0)) / s.equity_high_day
            if dd >= l.daily_dd_limit_pct:
                return f"daily_dd={dd:.3f}"
        if s.equity_high_week > 0:
            dd = (s.equity_high_week - max(s.balance_cents, 0)) / s.equity_high_week
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
            "safe_mode_reason": s.safe_mode_reason,
            "manual_recover_requested": s.manual_recover_requested,
            "probation_until_ts": s.probation_until_ts,
            "balance_cents": s.balance_cents,
            "exposure_cents": s.total_exposure_cents,
            "open_positions": dict(s.open_positions),
            "consecutive_losses": s.consecutive_losses,
            "realized_pnl_day": s.realized_pnl_day,
            "realized_pnl_week": s.realized_pnl_week,
            "equity_high_day": s.equity_high_day,
            "equity_high_week": s.equity_high_week,
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
        if breaker is None:
            # Limpia safe_mode “pegado” cuando el breaker ya no aplica (p. ej. rate_limit vencido).
            self._maybe_clear_safe_mode()

        if breaker == "rate_limit":
            # Transitorio: no activar safe_mode persistente (antes bloqueaba órdenes hasta recover manual).
            if breaker not in self.state.breakers[-3:]:
                self.state.breakers.append(breaker)
            return SizingDecision(
                market_ticker=ticker, side=gate.side or "YES",
                price_cents=gate.price_cents, contracts=0, notional_cents=0,
                f_full=0.0, f_capped=0.0,
                rationale="rate_limit:retry_later",
                safe_mode=False,
            )

        if breaker:
            self.state.safe_mode = True
            self.state.safe_mode_reason = breaker
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
        cap_cents = max(0, int(min(f_pos * bal, market_cap, total_cap)))

        contracts = cap_cents // c if c > 0 else 0
        notional = contracts * c
        rationale = (
            f"p={p:.3f} b={b:.3f} f*={f_full:.4f} frac={f_frac:.4f} "
            f"pos_cap={f_pos:.4f} mkt_cap={market_cap} tot_cap={total_cap} "
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
        self.state.balance_cents += pnl_cents
        if pnl_cents < 0:
            self.state.consecutive_losses += 1
        else:
            self.state.consecutive_losses = 0
        # equity high water marks
        eq = self.state.balance_cents
        self.state.equity_high_day = max(self.state.equity_high_day, eq)
        self.state.equity_high_week = max(self.state.equity_high_week, eq)
        breaker = self._check_breakers()
        if breaker:
            self.state.safe_mode = True
            self.state.safe_mode_reason = breaker
            if breaker not in self.state.breakers[-3:]:
                self.state.breakers.append(breaker)
        else:
            self._maybe_clear_safe_mode()

    def update_balance(self, balance_cents: int, reset_high_water: bool = False) -> None:
        self.state.balance_cents = balance_cents
        if reset_high_water:
            self.state.equity_high_day = balance_cents
            self.state.equity_high_week = balance_cents
            self._maybe_clear_safe_mode(force=True)
            return
        self.state.equity_high_day = max(self.state.equity_high_day, balance_cents)
        self.state.equity_high_week = max(self.state.equity_high_week, balance_cents)
        self._maybe_clear_safe_mode()

    def request_safe_mode_recovery(self, probation_seconds: int = 900) -> bool:
        """
        Recuperación supervisada: limpia safe_mode si breakers están en verde
        y aplica una ventana de probation.
        """
        self.state.manual_recover_requested = True
        breaker = self._check_breakers()
        if breaker:
            self.state.safe_mode = True
            self.state.safe_mode_reason = breaker
            return False
        self.state.safe_mode = False
        self.state.safe_mode_reason = ""
        self.state.breakers.clear()
        self.state.manual_recover_requested = False
        self.state.probation_until_ts = time.time() + max(60, int(probation_seconds))
        return True

    # ------------------------------------------------------------------
    def roll_day(self) -> None:
        self.state.realized_pnl_day = 0
        self.state.equity_high_day = self.state.balance_cents
        self.state.day_start = time.time()
        self.state.consecutive_losses = 0
        self._maybe_clear_safe_mode(force=True)

    def roll_week(self) -> None:
        self.state.realized_pnl_week = 0
        self.state.equity_high_week = self.state.balance_cents
        self.state.week_start = time.time()

    def _maybe_clear_safe_mode(self, force: bool = False) -> None:
        if force:
            self.state.safe_mode = False
            self.state.safe_mode_reason = ""
            self.state.breakers.clear()
            self.state.manual_recover_requested = False
            return
        if not self.state.safe_mode:
            return
        if self._check_breakers() is None:
            self.state.safe_mode = False
            self.state.safe_mode_reason = ""
            self.state.breakers.clear()
            self.state.manual_recover_requested = False

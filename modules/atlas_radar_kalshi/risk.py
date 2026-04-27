"""
risk.py — Capa 3: Gestión de capital y dimensionamiento por Kelly.

Fórmula
-------

.. math::

    f^{*} = \\frac{p(b+1) - 1}{b}

donde:

- ``p`` = probabilidad estimada de éxito (sale del :mod:`brain`).
- ``b`` = ratio de payoff neto sobre el capital arriesgado.
  Para Kalshi, comprando YES a ``c`` ¢ con liquidación 100¢:

  .. math:: b = \\frac{100 - c}{c}

- ``f*`` = fracción óptima del capital a apostar.

Aplicamos:

- **fractional Kelly** (``kelly_fraction`` en settings).
- **cap absoluto** ``max_position_pct`` sobre el balance.
- Saneo por ``f* < 0`` (no apostar) y ``f* > 1`` (cap a 1).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from pydantic import BaseModel, Field

from .brain import BrainDecision
from .config import RadarSettings, get_settings
from .utils.logger import get_logger


class PositionSize(BaseModel):
    """Resultado del cálculo de tamaño de posición."""
    market_ticker: str
    side: str
    price_cents: int = Field(..., ge=1, le=99)
    contracts: int = Field(..., ge=0)
    notional_cents: int = Field(..., ge=0)
    kelly_fraction: float
    fractional_kelly: float
    capped_fraction: float
    rationale: str = ""

    @property
    def notional_usd(self) -> float:
        return self.notional_cents / 100.0


class KellyRiskManager:
    """Calcula tamaño de posición a partir de :class:`BrainDecision`."""

    def __init__(self, settings: Optional[RadarSettings] = None) -> None:
        self.settings = settings or get_settings()
        self.log = get_logger("risk", self.settings.log_dir,
                              self.settings.log_level)

    # ------------------------------------------------------------------
    @staticmethod
    def kelly_fraction(p: float, b: float) -> float:
        """
        Kelly canónico ``f* = (p(b+1) - 1) / b``.
        Devuelve 0 si b<=0 o si f*<0.
        """
        if b <= 0:
            return 0.0
        f = (p * (b + 1.0) - 1.0) / b
        return max(0.0, min(1.0, f))

    # ------------------------------------------------------------------
    def size(
        self,
        decision: BrainDecision,
        balance_cents: int,
        price_cents: int,
    ) -> PositionSize:
        """
        Parameters
        ----------
        decision : BrainDecision
        balance_cents : int
            Balance disponible en centavos (Kalshi opera en ¢).
        price_cents : int
            Precio actual del lado elegido (1-99 ¢).
        """
        if decision.side is None:
            return self._zero(decision, price_cents,
                              "No actionable: edge bajo umbral.")

        p = decision.p_model if decision.side == "YES" else (1 - decision.p_model)
        # b = (payout - cost) / cost. payout=100¢, cost=price_cents.
        c = max(1, min(99, price_cents))
        b = (100 - c) / c

        f_full = self.kelly_fraction(p, b)
        f_frac = f_full * self.settings.kelly_fraction
        f_capped = min(f_frac, self.settings.max_position_pct)

        notional = int(balance_cents * f_capped)
        contracts = max(0, notional // c)
        notional_eff = contracts * c

        rationale = (
            f"p={p:.3f} b={b:.3f} f*={f_full:.4f} "
            f"frac={f_frac:.4f} capped={f_capped:.4f} "
            f"contracts={contracts} @ {c}¢"
        )
        self.log.info("Sizing %s %s -> %s", decision.market_ticker,
                      decision.side, rationale)

        return PositionSize(
            market_ticker=decision.market_ticker,
            side=decision.side,
            price_cents=c,
            contracts=contracts,
            notional_cents=notional_eff,
            kelly_fraction=f_full,
            fractional_kelly=f_frac,
            capped_fraction=f_capped,
            rationale=rationale,
        )

    # ------------------------------------------------------------------
    def _zero(self, d: BrainDecision, price_cents: int,
              msg: str) -> PositionSize:
        return PositionSize(
            market_ticker=d.market_ticker,
            side=d.side or "YES",
            price_cents=max(1, min(99, price_cents)) or 50,
            contracts=0,
            notional_cents=0,
            kelly_fraction=0.0,
            fractional_kelly=0.0,
            capped_fraction=0.0,
            rationale=msg,
        )

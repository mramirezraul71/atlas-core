"""Atlas Code Quant — StrategyIntent canónico (F11).

F11 introduce la **interfaz canónica** que media entre las oportunidades
oficiales del Radar (``RadarOpportunityInternal``) y cualquier consumidor
posterior de estrategias de opciones.

Política F11 (verbatim):

    * Scanner propone, Radar decide.
    * Las estrategias y la ejecución sólo operan sobre oportunidades
      aprobadas por Radar.
    * F11 **NO** ejecuta nada, **NO** llama LEAN, **NO** llama Tradier,
      **NO** consulta precios reales. Sólo declara la forma del intent
      y constructores puros con strikes/expiries **relativos**.

Reglas duras:

    * NO se importa desde ``execution/``, ``autonomy/``, ``risk/``,
      ``operations/``, ``broker_router``, ``Tradier``, ``vision``,
      ``atlas_adapter`` ni ``api/main.py``.
    * NO toca ``locks`` (``paper_only``, ``full_live_globally_locked``).
    * Constructores **defensivos**: nunca lanzan; ante input raro
      devuelven lista vacía.
    * Strikes / expiries siempre **relativos** (offsets enteros y DTE),
      nunca precios o fechas absolutas.

Ver también:
    * docs/ATLAS_CODE_QUANT_F11_STRATEGY_INTENT_OVER_RADAR.md
    * atlas_code_quant/intake/opportunity.py
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field, replace
from typing import Any, Iterable, Literal, Mapping

from atlas_code_quant.intake.opportunity import RadarOpportunityInternal

logger = logging.getLogger("atlas.code_quant.strategies.options.intent")


__all__ = [
    "StrategyType",
    "VALID_STRATEGY_TYPES",
    "OptionRight",
    "OptionSide",
    "StrategyLeg",
    "RiskLimits",
    "OpportunityRef",
    "StrategyIntent",
    "build_opportunity_ref",
]


# ---------------------------------------------------------------------------
# Tipos canónicos
# ---------------------------------------------------------------------------

StrategyType = Literal[
    "vertical_spread",
    "iron_condor",
    "iron_butterfly",
    "straddle",
    "strangle",
    "long_call",
    "long_put",
]


VALID_STRATEGY_TYPES: frozenset[str] = frozenset(
    {
        "vertical_spread",
        "iron_condor",
        "iron_butterfly",
        "straddle",
        "strangle",
        "long_call",
        "long_put",
    }
)


OptionRight = Literal["call", "put"]
OptionSide = Literal["buy", "sell"]


# ---------------------------------------------------------------------------
# Componentes
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class StrategyLeg:
    """Pata individual de una estrategia de opciones.

    Strikes y expiries se expresan en **forma relativa**:

    * ``strike_offset_steps``: número entero de "pasos" alrededor del
      ATM (positivo = OTM call / ITM put, negativo = ITM call / OTM put).
      F11 no traduce esto a un precio: sólo declara la geometría.
    * ``expiry_rel_dte``: días al vencimiento. F11 no resuelve a fecha
      calendario; F13+ podrá hacerlo cuando se conecte LEAN/Tradier.
    """

    side: OptionSide
    right: OptionRight
    strike_offset_steps: int
    expiry_rel_dte: int
    quantity: int = 1
    notes: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "side": self.side,
            "right": self.right,
            "strike_offset_steps": int(self.strike_offset_steps),
            "expiry_rel_dte": int(self.expiry_rel_dte),
            "quantity": int(self.quantity),
            "notes": self.notes,
        }


@dataclass(frozen=True)
class RiskLimits:
    """Límites de riesgo declarativos por strategy intent.

    Todos los campos son relativos / por intent. F11 no aplica
    sizing real (eso es responsabilidad de F14+ con fitness y de F15+
    con la capa de ejecución).
    """

    max_loss_usd: float | None = None
    target_r_multiple: float | None = None
    sizing_hint: str = "single_contract"

    def to_dict(self) -> dict[str, Any]:
        return {
            "max_loss_usd": self.max_loss_usd,
            "target_r_multiple": self.target_r_multiple,
            "sizing_hint": self.sizing_hint,
        }


@dataclass(frozen=True)
class OpportunityRef:
    """Referencia mínima a la oportunidad Radar de origen.

    Mantenemos sólo lo imprescindible para auditar la cadena
    Radar → strategy intent → … sin acoplar StrategyIntent al objeto
    completo ``RadarOpportunityInternal`` (que es más pesado y
    contiene el ``raw`` payload del Radar).
    """

    symbol: str
    asset_class: str
    direction: str
    horizon_min: int | None
    classification: str
    score: float
    trace_id: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "symbol": self.symbol,
            "asset_class": self.asset_class,
            "direction": self.direction,
            "horizon_min": self.horizon_min,
            "classification": self.classification,
            "score": self.score,
            "trace_id": self.trace_id,
        }


def build_opportunity_ref(opp: RadarOpportunityInternal) -> OpportunityRef:
    """Construye ``OpportunityRef`` a partir de la oportunidad Radar.

    Defensivo: si recibe un objeto raro, fuerza un ref con campos
    "vacíos" para no romper builders río arriba.
    """
    if not isinstance(opp, RadarOpportunityInternal):
        return OpportunityRef(
            symbol="",
            asset_class="unknown",
            direction="neutral",
            horizon_min=None,
            classification="reject",
            score=0.0,
            trace_id="",
        )
    return OpportunityRef(
        symbol=opp.symbol,
        asset_class=opp.asset_class,
        direction=opp.direction,
        horizon_min=opp.horizon_min,
        classification=opp.classification,
        score=float(opp.score),
        trace_id=opp.trace_id,
    )


# ---------------------------------------------------------------------------
# StrategyIntent
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class StrategyIntent:
    """Intent canónico de estrategia derivado de una oportunidad Radar.

    Es un objeto **declarativo**: describe qué estrategia se propondría
    ejecutar contra ``opportunity``, con qué patas y bajo qué límites.
    No realiza pricing ni envía órdenes.
    """

    opportunity: OpportunityRef
    strategy_type: StrategyType
    legs: tuple[StrategyLeg, ...]
    risk_limits: RiskLimits = field(default_factory=RiskLimits)
    metadata: Mapping[str, Any] = field(default_factory=dict)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @property
    def num_legs(self) -> int:
        return len(self.legs)

    @property
    def is_multi_leg(self) -> bool:
        return self.num_legs > 1

    def with_metadata(self, **extra: Any) -> "StrategyIntent":
        merged: dict[str, Any] = dict(self.metadata)
        merged.update(extra)
        return replace(self, metadata=merged)

    def to_dict(self) -> dict[str, Any]:
        return {
            "opportunity": self.opportunity.to_dict(),
            "strategy_type": self.strategy_type,
            "legs": [leg.to_dict() for leg in self.legs],
            "risk_limits": self.risk_limits.to_dict(),
            "metadata": dict(self.metadata),
        }


# ---------------------------------------------------------------------------
# Validación auxiliar
# ---------------------------------------------------------------------------


def _coerce_legs(legs: Iterable[StrategyLeg]) -> tuple[StrategyLeg, ...]:
    """Filtra legs malformadas; nunca lanza."""
    out: list[StrategyLeg] = []
    for leg in legs or ():
        if not isinstance(leg, StrategyLeg):
            continue
        if leg.right not in ("call", "put"):
            continue
        if leg.side not in ("buy", "sell"):
            continue
        if int(leg.quantity) <= 0:
            continue
        out.append(leg)
    return tuple(out)

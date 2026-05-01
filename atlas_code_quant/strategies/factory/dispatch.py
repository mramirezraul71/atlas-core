"""Atlas Code Quant — Strategy Factory (F12).

F12 introduce el **dispatcher único** que, dada una oportunidad Radar
ya aprobada (``RadarOpportunityInternal``), decide qué estrategias
(`StrategyIntent`) son aplicables y las construye combinando los
builders de F11.

Política F12:

    * Scanner propone, Radar decide (F10), F12 selecciona estructuras.
    * Factory **pura**: sin red, sin precios, sin I/O.
    * Defensiva: nunca lanza; ante input inválido devuelve ``[]``.
    * No introduce execution / Tradier / LEAN.

Reglas de dispatch (orden estable, sin duplicados):

    1. Si la oportunidad NO está aprobada por Radar
       (``classification == "reject"`` o ``score < min_score``),
       devolver ``[]``.
    2. Si NO es ``optionable``, devolver ``[]`` (las estructuras de
       F11 son option-based).
    3. Si ``asset_class`` no está en {equity, etf, index}, devolver
       ``[]`` (F11 sólo cubre subyacentes con cadena de opciones
       conocida).
    4. Direccional (long/short):
        * vertical_spread (defined-risk).
        * long_single (long_call o long_put).
    5. Neutral:
        * iron_condor.
        * Si DTE != LONG → iron_butterfly.
        * straddle.
        * Si DTE != SHORT → strangle.
    6. Filtro por ``classification``:
        * ``high_conviction`` → todas las anteriores.
        * ``watchlist`` → solo estructuras de **defined risk**
          (verticals + condor + butterfly). Esto evita exponer la
          cuenta a vol-plays largos sin convicción alta.

Ver también:
    * docs/ATLAS_CODE_QUANT_F12_STRATEGY_FACTORY.md
    * atlas_code_quant/strategies/options/intent.py
    * atlas_code_quant/strategies/options/builder.py
"""

from __future__ import annotations

import logging
from typing import Iterable, List

from atlas_code_quant.intake.opportunity import RadarOpportunityInternal
from atlas_code_quant.strategies.options.builder import (
    build_iron_butterfly_intents_from_opportunity,
    build_iron_condor_intents_from_opportunity,
    build_long_single_intents_from_opportunity,
    build_straddle_intents_from_opportunity,
    build_strangle_intents_from_opportunity,
    build_vertical_spread_intents_from_opportunity,
)
from atlas_code_quant.strategies.options.intent import StrategyIntent

logger = logging.getLogger("atlas.code_quant.strategies.factory")


__all__ = [
    "DEFAULT_MIN_SCORE",
    "VALID_ASSET_CLASSES",
    "DEFINED_RISK_TYPES",
    "build_strategies_for_opportunity",
    "build_strategies_for_opportunities",
]


DEFAULT_MIN_SCORE: float = 70.0
VALID_ASSET_CLASSES: frozenset[str] = frozenset({"equity", "etf", "index"})
DEFINED_RISK_TYPES: frozenset[str] = frozenset(
    {"vertical_spread", "iron_condor", "iron_butterfly"}
)


# ---------------------------------------------------------------------------
# Helpers internos
# ---------------------------------------------------------------------------


def _is_radar_approved(
    opp: RadarOpportunityInternal, *, min_score: float
) -> bool:
    """Replica el contrato del filtro F7 a nivel factory.

    NOTA: F7 también usa ``score>=70`` y ``classification != reject``,
    pero esta función opera sobre oportunidades **ya** filtradas por
    F10. Aplicar el chequeo aquí es defensa-en-profundidad: si alguien
    inyecta una oportunidad sin pasar el gate, la factory no debería
    proponer estructuras.
    """
    if opp.classification == "reject":
        return False
    if opp.score < float(min_score):
        return False
    return True


# ---------------------------------------------------------------------------
# API pública
# ---------------------------------------------------------------------------


def build_strategies_for_opportunity(
    opp: RadarOpportunityInternal | None,
    *,
    min_score: float = DEFAULT_MIN_SCORE,
) -> list[StrategyIntent]:
    """Construye la lista de StrategyIntent aplicables a ``opp``.

    Pure function. No I/O. No excepciones hacia fuera.
    """
    try:
        return list(_dispatch(opp, min_score=min_score))
    except Exception:  # noqa: BLE001 — defensa "no matar el proceso"
        logger.warning(
            "strategies.factory: fallo construyendo intents para %s",
            getattr(opp, "symbol", "?"),
        )
        return []


def _dispatch(
    opp: RadarOpportunityInternal | None,
    *,
    min_score: float,
) -> Iterable[StrategyIntent]:
    if opp is None or not isinstance(opp, RadarOpportunityInternal):
        return []
    if not opp.symbol:
        return []
    if not opp.optionable:
        return []
    if opp.asset_class not in VALID_ASSET_CLASSES:
        return []
    if not _is_radar_approved(opp, min_score=min_score):
        return []

    direction = opp.direction
    intents: List[StrategyIntent] = []

    if direction in ("long", "short"):
        intents.extend(build_vertical_spread_intents_from_opportunity(opp))
        intents.extend(build_long_single_intents_from_opportunity(opp))
    elif direction == "neutral":
        intents.extend(build_iron_condor_intents_from_opportunity(opp))
        intents.extend(build_iron_butterfly_intents_from_opportunity(opp))
        intents.extend(build_straddle_intents_from_opportunity(opp))
        intents.extend(build_strangle_intents_from_opportunity(opp))
    else:
        # direction desconocida → no proponer
        return []

    # Filtro por classification.
    if opp.classification == "watchlist":
        intents = [it for it in intents if it.strategy_type in DEFINED_RISK_TYPES]

    # Estabilidad: orden por strategy_type (alfabético) y luego por num_legs.
    intents.sort(key=lambda i: (i.strategy_type, i.num_legs))
    return intents


def build_strategies_for_opportunities(
    opps: Iterable[RadarOpportunityInternal],
    *,
    min_score: float = DEFAULT_MIN_SCORE,
) -> list[StrategyIntent]:
    """Mismo contrato pero sobre un iterable de oportunidades."""
    out: list[StrategyIntent] = []
    if opps is None:
        return out
    for opp in opps:
        out.extend(build_strategies_for_opportunity(opp, min_score=min_score))
    return out

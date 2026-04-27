"""Atlas Code Quant — Builders de StrategyIntent (F11).

Funciones puras que, dada una :class:`RadarOpportunityInternal` ya
aprobada por Radar, construyen una lista de :class:`StrategyIntent`
candidatos para esa oportunidad.

Reglas duras (F11):

    * **Funciones puras** — sin I/O, sin red, sin precios reales.
    * **Defensivas** — nunca lanzan; ante input inválido devuelven
      ``[]``.
    * **Strikes y expiries relativos** — sólo offsets / DTE; ningún
      precio o fecha calendario.
    * **No conocen Tradier ni LEAN** — esa conexión llega en F13+/F15+.

La elección de strikes / DTE refleja heurísticas conservadoras del
estudio interno:

    * DTE objetivo según ``horizon_min`` del Radar (lo más corto cae
      en 7 DTE; lo largo se acerca a 45 DTE).
    * Strikes alrededor del ATM (offsets ``±1..3``) según direction.
    * En "neutral" se ofrecen estructuras simétricas (condor /
      butterfly / straddle / strangle).

Política de robustez:

    * Cada builder se envuelve en un try/except top-level y, ante
      cualquier error, registra un ``logger.warning`` y devuelve ``[]``.
    * Builders nunca devuelven legs incompletas (validación con
      ``_coerce_legs``).

Ver también:
    * docs/ATLAS_CODE_QUANT_F11_STRATEGY_INTENT_OVER_RADAR.md
"""

from __future__ import annotations

import logging
from typing import Iterable

from atlas_code_quant.intake.opportunity import RadarOpportunityInternal
from atlas_code_quant.strategies.options.intent import (
    OpportunityRef,
    RiskLimits,
    StrategyIntent,
    StrategyLeg,
    _coerce_legs,
    build_opportunity_ref,
)

logger = logging.getLogger("atlas.code_quant.strategies.options.builder")


__all__ = [
    "DTE_SHORT",
    "DTE_MEDIUM",
    "DTE_LONG",
    "choose_dte_for_horizon",
    "build_vertical_spread_intents_from_opportunity",
    "build_iron_condor_intents_from_opportunity",
    "build_iron_butterfly_intents_from_opportunity",
    "build_straddle_intents_from_opportunity",
    "build_strangle_intents_from_opportunity",
    "build_long_single_intents_from_opportunity",
]


# DTE buckets canónicos para F11 (siempre relativos, no calendario).
DTE_SHORT: int = 7
DTE_MEDIUM: int = 21
DTE_LONG: int = 45


def choose_dte_for_horizon(horizon_min: int | None) -> int:
    """Mapea ``horizon_min`` (minutos) a un bucket DTE canónico.

    Política conservadora: ante None / inválido devuelve ``DTE_MEDIUM``.
    """
    if horizon_min is None:
        return DTE_MEDIUM
    try:
        h = int(horizon_min)
    except (TypeError, ValueError):
        return DTE_MEDIUM
    if h <= 0:
        return DTE_MEDIUM
    # heurística: minutos → bucket DTE.
    if h <= 60 * 4:  # ≤ 4h intraday
        return DTE_SHORT
    if h <= 60 * 24 * 5:  # ≤ 5 días
        return DTE_MEDIUM
    return DTE_LONG


def _safe_ref(opp: RadarOpportunityInternal | None) -> OpportunityRef | None:
    if opp is None or not isinstance(opp, RadarOpportunityInternal):
        return None
    if not opp.symbol:
        return None
    return build_opportunity_ref(opp)


def _wrap(
    builder_name: str,
    opp: RadarOpportunityInternal | None,
    fn,
) -> list[StrategyIntent]:
    """Ejecuta un builder bajo try/except y registra fallos como warning.

    Mantiene la promesa "nunca lanza".
    """
    ref = _safe_ref(opp)
    if ref is None:
        return []
    try:
        intents = list(fn(opp, ref))
    except Exception:  # noqa: BLE001 — defensa "no matar el proceso"
        logger.warning(
            "strategies.builder.%s: fallo construyendo intents para %s",
            builder_name,
            getattr(opp, "symbol", "?"),
        )
        return []
    # Filtrado final defensivo: descarta intents sin legs validas.
    out: list[StrategyIntent] = []
    for intent in intents:
        if not isinstance(intent, StrategyIntent):
            continue
        legs = _coerce_legs(intent.legs)
        if not legs:
            continue
        if legs is intent.legs:
            out.append(intent)
        else:
            out.append(
                StrategyIntent(
                    opportunity=intent.opportunity,
                    strategy_type=intent.strategy_type,
                    legs=legs,
                    risk_limits=intent.risk_limits,
                    metadata=intent.metadata,
                )
            )
    return out


# ---------------------------------------------------------------------------
# Vertical spreads (debit / credit, call / put)
# ---------------------------------------------------------------------------


def _vertical_spread_legs(
    direction: str, dte: int
) -> tuple[tuple[StrategyLeg, ...], str, str]:
    """Devuelve (legs, structure_label, sizing_hint) para un vertical."""
    if direction == "long":
        # Long call vertical (debit): buy ATM, sell +2 OTM
        legs = (
            StrategyLeg(
                side="buy",
                right="call",
                strike_offset_steps=0,
                expiry_rel_dte=dte,
                notes="long_atm",
            ),
            StrategyLeg(
                side="sell",
                right="call",
                strike_offset_steps=2,
                expiry_rel_dte=dte,
                notes="short_otm_+2",
            ),
        )
        return legs, "bull_call_debit", "defined_risk_long"
    if direction == "short":
        # Long put vertical (debit): buy ATM put, sell -2 OTM put
        legs = (
            StrategyLeg(
                side="buy",
                right="put",
                strike_offset_steps=0,
                expiry_rel_dte=dte,
                notes="long_atm",
            ),
            StrategyLeg(
                side="sell",
                right="put",
                strike_offset_steps=-2,
                expiry_rel_dte=dte,
                notes="short_otm_-2",
            ),
        )
        return legs, "bear_put_debit", "defined_risk_long"
    # neutral o desconocido: no aplica vertical direccional
    return (), "n/a", "skip"


def build_vertical_spread_intents_from_opportunity(
    opp: RadarOpportunityInternal | None,
) -> list[StrategyIntent]:
    """Construye verticals (debit) si la dirección es long/short.

    En "neutral" devuelve ``[]`` (los condors / strangles cubren ese
    régimen).
    """

    def _impl(opp: RadarOpportunityInternal, ref: OpportunityRef):
        dte = choose_dte_for_horizon(opp.horizon_min)
        legs, label, sizing = _vertical_spread_legs(opp.direction, dte)
        if not legs:
            return []
        return [
            StrategyIntent(
                opportunity=ref,
                strategy_type="vertical_spread",
                legs=legs,
                risk_limits=RiskLimits(
                    max_loss_usd=None,  # F11 no resuelve dollar risk.
                    target_r_multiple=1.5,
                    sizing_hint=sizing,
                ),
                metadata={"structure": label, "dte": dte},
            )
        ]

    return _wrap("vertical_spread", opp, _impl)


# ---------------------------------------------------------------------------
# Iron condor (4 legs, neutral)
# ---------------------------------------------------------------------------


def build_iron_condor_intents_from_opportunity(
    opp: RadarOpportunityInternal | None,
) -> list[StrategyIntent]:
    """Iron condor centrado en ATM, ancho ±1/±3 strikes.

    Apto sobre todo para ``direction=neutral`` o asset_class=index con
    horizon medio/largo. Para direction direccional se prefieren los
    verticals (no devolvemos condor).
    """

    def _impl(opp: RadarOpportunityInternal, ref: OpportunityRef):
        if opp.direction != "neutral":
            return []
        dte = choose_dte_for_horizon(opp.horizon_min)
        legs = (
            # Put wing
            StrategyLeg(
                side="sell",
                right="put",
                strike_offset_steps=-1,
                expiry_rel_dte=dte,
                notes="short_put_-1",
            ),
            StrategyLeg(
                side="buy",
                right="put",
                strike_offset_steps=-3,
                expiry_rel_dte=dte,
                notes="long_put_-3",
            ),
            # Call wing
            StrategyLeg(
                side="sell",
                right="call",
                strike_offset_steps=1,
                expiry_rel_dte=dte,
                notes="short_call_+1",
            ),
            StrategyLeg(
                side="buy",
                right="call",
                strike_offset_steps=3,
                expiry_rel_dte=dte,
                notes="long_call_+3",
            ),
        )
        return [
            StrategyIntent(
                opportunity=ref,
                strategy_type="iron_condor",
                legs=legs,
                risk_limits=RiskLimits(
                    target_r_multiple=0.4, sizing_hint="defined_risk_neutral"
                ),
                metadata={"structure": "iron_condor_1_3", "dte": dte},
            )
        ]

    return _wrap("iron_condor", opp, _impl)


# ---------------------------------------------------------------------------
# Iron butterfly (4 legs, neutral, body ATM)
# ---------------------------------------------------------------------------


def build_iron_butterfly_intents_from_opportunity(
    opp: RadarOpportunityInternal | None,
) -> list[StrategyIntent]:
    """Iron butterfly: short straddle ATM + alas largas a ±2 strikes.

    Sólo se propone para ``direction=neutral`` y horizon corto/medio
    (un butterfly muy largo aporta poco sin precio).
    """

    def _impl(opp: RadarOpportunityInternal, ref: OpportunityRef):
        if opp.direction != "neutral":
            return []
        dte = choose_dte_for_horizon(opp.horizon_min)
        if dte == DTE_LONG:
            # política: butterflies no se proponen para horizonte muy largo.
            return []
        legs = (
            StrategyLeg(
                side="sell",
                right="put",
                strike_offset_steps=0,
                expiry_rel_dte=dte,
                notes="short_atm_put",
            ),
            StrategyLeg(
                side="sell",
                right="call",
                strike_offset_steps=0,
                expiry_rel_dte=dte,
                notes="short_atm_call",
            ),
            StrategyLeg(
                side="buy",
                right="put",
                strike_offset_steps=-2,
                expiry_rel_dte=dte,
                notes="long_put_-2",
            ),
            StrategyLeg(
                side="buy",
                right="call",
                strike_offset_steps=2,
                expiry_rel_dte=dte,
                notes="long_call_+2",
            ),
        )
        return [
            StrategyIntent(
                opportunity=ref,
                strategy_type="iron_butterfly",
                legs=legs,
                risk_limits=RiskLimits(
                    target_r_multiple=0.6, sizing_hint="defined_risk_neutral"
                ),
                metadata={"structure": "iron_butterfly_atm_2", "dte": dte},
            )
        ]

    return _wrap("iron_butterfly", opp, _impl)


# ---------------------------------------------------------------------------
# Straddle / Strangle (vol plays, neutral)
# ---------------------------------------------------------------------------


def build_straddle_intents_from_opportunity(
    opp: RadarOpportunityInternal | None,
) -> list[StrategyIntent]:
    """Long straddle ATM (vol-play). Propone sólo en neutral.

    El sentido es comprar volatilidad cuando Radar marca un setup
    no-direccional pero con score alto.
    """

    def _impl(opp: RadarOpportunityInternal, ref: OpportunityRef):
        if opp.direction != "neutral":
            return []
        dte = choose_dte_for_horizon(opp.horizon_min)
        legs = (
            StrategyLeg(
                side="buy",
                right="call",
                strike_offset_steps=0,
                expiry_rel_dte=dte,
                notes="long_atm_call",
            ),
            StrategyLeg(
                side="buy",
                right="put",
                strike_offset_steps=0,
                expiry_rel_dte=dte,
                notes="long_atm_put",
            ),
        )
        return [
            StrategyIntent(
                opportunity=ref,
                strategy_type="straddle",
                legs=legs,
                risk_limits=RiskLimits(
                    target_r_multiple=2.0, sizing_hint="vol_long"
                ),
                metadata={"structure": "long_straddle_atm", "dte": dte},
            )
        ]

    return _wrap("straddle", opp, _impl)


def build_strangle_intents_from_opportunity(
    opp: RadarOpportunityInternal | None,
) -> list[StrategyIntent]:
    """Long strangle OTM (±2 strikes). Más barato que straddle.

    F11 sólo lo propone en neutral con horizon medio/largo.
    """

    def _impl(opp: RadarOpportunityInternal, ref: OpportunityRef):
        if opp.direction != "neutral":
            return []
        dte = choose_dte_for_horizon(opp.horizon_min)
        if dte == DTE_SHORT:
            return []
        legs = (
            StrategyLeg(
                side="buy",
                right="call",
                strike_offset_steps=2,
                expiry_rel_dte=dte,
                notes="long_call_+2",
            ),
            StrategyLeg(
                side="buy",
                right="put",
                strike_offset_steps=-2,
                expiry_rel_dte=dte,
                notes="long_put_-2",
            ),
        )
        return [
            StrategyIntent(
                opportunity=ref,
                strategy_type="strangle",
                legs=legs,
                risk_limits=RiskLimits(
                    target_r_multiple=2.5, sizing_hint="vol_long"
                ),
                metadata={"structure": "long_strangle_2", "dte": dte},
            )
        ]

    return _wrap("strangle", opp, _impl)


# ---------------------------------------------------------------------------
# Long single (call o put solo) — fallback direccional sencillo
# ---------------------------------------------------------------------------


def build_long_single_intents_from_opportunity(
    opp: RadarOpportunityInternal | None,
) -> list[StrategyIntent]:
    """Long call o long put ATM como expresión direccional simple.

    No es defined-risk como el vertical pero es la versión "más barata
    de declarar" para la oportunidad. Útil cuando el resto de builders
    deciden no proponer estructura (p. ej. butterfly bloqueado por
    DTE_LONG).
    """

    def _impl(opp: RadarOpportunityInternal, ref: OpportunityRef):
        dte = choose_dte_for_horizon(opp.horizon_min)
        if opp.direction == "long":
            legs = (
                StrategyLeg(
                    side="buy",
                    right="call",
                    strike_offset_steps=0,
                    expiry_rel_dte=dte,
                    notes="long_atm_call",
                ),
            )
            stype = "long_call"
        elif opp.direction == "short":
            legs = (
                StrategyLeg(
                    side="buy",
                    right="put",
                    strike_offset_steps=0,
                    expiry_rel_dte=dte,
                    notes="long_atm_put",
                ),
            )
            stype = "long_put"
        else:
            return []
        return [
            StrategyIntent(
                opportunity=ref,
                strategy_type=stype,
                legs=legs,
                risk_limits=RiskLimits(
                    target_r_multiple=1.0, sizing_hint="single_long"
                ),
                metadata={"structure": stype, "dte": dte},
            )
        ]

    return _wrap("long_single", opp, _impl)

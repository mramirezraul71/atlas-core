"""Atlas Code Quant — Strategy fitness pipeline (F14).

F14 conecta F12 (StrategyFactory) con F13 (LEAN adapter):

    Radar opportunity (gated F10)
        → factory.build_strategies_for_opportunity (F12)
            → lean.runner.run_backtest_for_strategy_intent (F13)
                → StrategyWithFitness (ordenado por métrica)

El pipeline es **puro de orquestación**: no abre red, no ejecuta
órdenes, no consulta precios. Sólo combina los outputs de F12 y F13.

Reglas duras:

    * NO importar execution / autonomy / risk / vision / atlas_adapter
      / tradier / broker_router / live_*.
    * Defensivo: nunca lanza. Cada intent fallido produce un
      ``StrategyWithFitness`` con fitness ``success=False`` y sigue
      iterando.
    * Inyección de dependencias: el ``backtest_fn`` se puede
      sobreescribir en tests para no llamar a LEAN.
    * Métrica de orden por defecto: Sharpe descendente, luego
      expectancy descendente, luego ``win_rate`` descendente.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Callable, Iterable, Optional

from atlas_code_quant.intake.opportunity import RadarOpportunityInternal
from atlas_code_quant.lean.runner.launcher import (
    StrategyFitnessResult,
    run_backtest_for_strategy_intent,
)
from atlas_code_quant.strategies.factory.dispatch import (
    DEFAULT_MIN_SCORE,
    build_strategies_for_opportunity,
)
from atlas_code_quant.strategies.options.intent import StrategyIntent

logger = logging.getLogger("atlas.code_quant.strategies.evaluation")


__all__ = [
    "StrategyWithFitness",
    "BacktestFn",
    "evaluate_strategies_for_opportunity",
    "evaluate_strategies_for_opportunities",
    "rank_by_default_metric",
]


BacktestFn = Callable[[StrategyIntent], StrategyFitnessResult]


@dataclass(frozen=True)
class StrategyWithFitness:
    """Pareja ``(StrategyIntent, StrategyFitnessResult)`` consumible por F16+."""

    intent: StrategyIntent
    fitness: StrategyFitnessResult

    @property
    def is_evaluated(self) -> bool:
        return self.fitness.success

    @property
    def primary_metric(self) -> float:
        return float(self.fitness.sharpe)

    def to_dict(self) -> dict:
        return {
            "intent": self.intent.to_dict(),
            "fitness": self.fitness.to_dict(),
        }


def _default_backtest(intent: StrategyIntent) -> StrategyFitnessResult:
    return run_backtest_for_strategy_intent(intent)


def _safe_backtest(
    intent: StrategyIntent, fn: BacktestFn
) -> StrategyFitnessResult:
    try:
        result = fn(intent)
    except Exception as exc:  # noqa: BLE001
        logger.warning(
            "evaluation.backtest: fallo evaluando intent (%s, %s): %s",
            intent.opportunity.symbol,
            intent.strategy_type,
            exc,
        )
        return StrategyFitnessResult(
            success=False,
            mode="error",
            error_code="EVALUATION_BACKTEST_RAISED",
            error_message=str(exc),
        )
    if not isinstance(result, StrategyFitnessResult):
        return StrategyFitnessResult(
            success=False,
            mode="error",
            error_code="EVALUATION_BACKTEST_INVALID_RETURN",
            error_message="backtest_fn did not return StrategyFitnessResult",
        )
    return result


def rank_by_default_metric(
    items: Iterable[StrategyWithFitness],
) -> list[StrategyWithFitness]:
    """Ordena descendente por (sharpe, expectancy, win_rate).

    Items con ``fitness.success=False`` se llevan al final.
    """

    def key(it: StrategyWithFitness):
        f = it.fitness
        return (
            0 if f.success else 1,
            -float(f.sharpe),
            -float(f.expectancy),
            -float(f.win_rate),
        )

    return sorted(list(items or []), key=key)


def evaluate_strategies_for_opportunity(
    opp: RadarOpportunityInternal | None,
    *,
    min_score: float = DEFAULT_MIN_SCORE,
    backtest_fn: Optional[BacktestFn] = None,
    rank: bool = True,
) -> list[StrategyWithFitness]:
    """Pipeline F14 para una oportunidad Radar.

    1. F12: factory genera intents.
    2. F13 (o ``backtest_fn`` inyectado): fitness por intent.
    3. (opcional) ranking por sharpe/expectancy/win_rate.
    """
    fn = backtest_fn or _default_backtest
    try:
        intents = build_strategies_for_opportunity(opp, min_score=min_score)
    except Exception:  # noqa: BLE001 — factory ya es defensiva, doble red
        logger.warning(
            "evaluation: factory falló para %s",
            getattr(opp, "symbol", "?"),
        )
        return []
    if not intents:
        return []

    out: list[StrategyWithFitness] = []
    for intent in intents:
        fitness = _safe_backtest(intent, fn)
        out.append(StrategyWithFitness(intent=intent, fitness=fitness))

    if rank:
        return rank_by_default_metric(out)
    return out


def evaluate_strategies_for_opportunities(
    opps: Iterable[RadarOpportunityInternal] | None,
    *,
    min_score: float = DEFAULT_MIN_SCORE,
    backtest_fn: Optional[BacktestFn] = None,
    rank: bool = True,
) -> list[StrategyWithFitness]:
    """Versión batch del pipeline F14."""
    if opps is None:
        return []
    out: list[StrategyWithFitness] = []
    for opp in opps:
        out.extend(
            evaluate_strategies_for_opportunity(
                opp,
                min_score=min_score,
                backtest_fn=backtest_fn,
                rank=False,
            )
        )
    if rank:
        return rank_by_default_metric(out)
    return out

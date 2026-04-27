"""Tests F14 — Strategy fitness pipeline (factory + LEAN).

Cubre:

* Pipeline single-opp con ``backtest_fn`` mock determinista.
* Ranking por (sharpe, expectancy, win_rate) y fallidos al final.
* Pipeline batch sobre múltiples oportunidades.
* Defensa: ``backtest_fn`` que lanza → fitness fallido sin propagar.
* Defensa: ``backtest_fn`` que devuelve tipo erróneo → fitness fallido.
* Modo no-rank preserva orden original.
* AST guard: módulo F14 NO importa execution / autonomy / risk /
  vision / atlas_adapter / tradier / broker_router / live_*.
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

from atlas_code_quant.intake.opportunity import RadarOpportunityInternal
from atlas_code_quant.lean.runner.launcher import StrategyFitnessResult
from atlas_code_quant.strategies.evaluation import (
    StrategyWithFitness,
    evaluate_strategies_for_opportunities,
    evaluate_strategies_for_opportunity,
    rank_by_default_metric,
)
from atlas_code_quant.strategies.options.intent import StrategyIntent


def _make_opp(
    *,
    symbol: str = "SPY",
    direction: str = "long",
    asset_class: str = "etf",
    horizon_min: int | None = 240,
    classification: str = "high_conviction",
    score: float = 80.0,
) -> RadarOpportunityInternal:
    return RadarOpportunityInternal.from_payload(
        {
            "symbol": symbol,
            "asset_class": asset_class,
            "sector": None,
            "optionable": True,
            "score": score,
            "classification": classification,
            "direction": direction,
            "horizon_min": horizon_min,
            "snapshot": {},
            "degradations_active": [],
            "source": "quant",
            "trace_id": "trace-f14",
            "timestamp": "2026-04-27T13:00:00+00:00",
        }
    )


def _success_fitness(*, sharpe: float, expectancy: float = 0.1, win_rate: float = 50.0) -> StrategyFitnessResult:
    return StrategyFitnessResult(
        success=True,
        mode="mock",
        sharpe=sharpe,
        expectancy=expectancy,
        win_rate=win_rate,
        max_drawdown=-5.0,
        total_return=10.0,
        num_orders=3,
    )


# ---------------------------------------------------------------------------
# Sección 1 — Pipeline single
# ---------------------------------------------------------------------------


class TestEvaluateSingle:
    def test_returns_strategy_with_fitness(self):
        opp = _make_opp(direction="long")
        sharpe_by_type = {
            "vertical_spread": 1.5,
            "long_call": 0.7,
        }

        def fake_backtest(intent: StrategyIntent) -> StrategyFitnessResult:
            return _success_fitness(sharpe=sharpe_by_type.get(intent.strategy_type, 0.0))

        out = evaluate_strategies_for_opportunity(opp, backtest_fn=fake_backtest)
        assert all(isinstance(it, StrategyWithFitness) for it in out)
        assert {it.intent.strategy_type for it in out} >= {
            "vertical_spread",
            "long_call",
        }
        # Ranking: vertical_spread (1.5) antes que long_call (0.7).
        first = out[0]
        assert first.intent.strategy_type == "vertical_spread"
        assert first.fitness.sharpe == pytest.approx(1.5)

    def test_no_intents_returns_empty(self):
        # reject → factory devuelve [] → pipeline devuelve [].
        opp = _make_opp(classification="reject")
        out = evaluate_strategies_for_opportunity(opp, backtest_fn=lambda i: _success_fitness(sharpe=0))
        assert out == []

    def test_none_input(self):
        out = evaluate_strategies_for_opportunity(None)
        assert out == []

    def test_backtest_raises_does_not_propagate(self):
        opp = _make_opp(direction="long")

        def boom(intent):
            raise RuntimeError("synthetic boom")

        out = evaluate_strategies_for_opportunity(opp, backtest_fn=boom)
        assert out  # se construyen intents y se asocian fitness fallidos
        for item in out:
            assert item.is_evaluated is False
            assert item.fitness.error_code == "EVALUATION_BACKTEST_RAISED"

    def test_backtest_returns_invalid(self):
        opp = _make_opp(direction="long")
        out = evaluate_strategies_for_opportunity(
            opp,
            backtest_fn=lambda i: "not-a-fitness",  # type: ignore[arg-type]
        )
        for item in out:
            assert item.is_evaluated is False
            assert item.fitness.error_code == "EVALUATION_BACKTEST_INVALID_RETURN"

    def test_rank_false_preserves_order(self):
        opp = _make_opp(direction="long")
        # alternamos sharpe entre tipos para evidenciar ausencia de ranking
        order: list[str] = []

        def fake(intent):
            order.append(intent.strategy_type)
            return _success_fitness(sharpe=0.1)

        out = evaluate_strategies_for_opportunity(opp, backtest_fn=fake, rank=False)
        assert [it.intent.strategy_type for it in out] == order


# ---------------------------------------------------------------------------
# Sección 2 — Ranking
# ---------------------------------------------------------------------------


class TestRanking:
    def test_rank_by_default_metric(self):
        opp = _make_opp(direction="long")
        # construimos manualmente StrategyWithFitness con varios sharpe.
        from atlas_code_quant.strategies.factory.dispatch import (
            build_strategies_for_opportunity,
        )

        intents = build_strategies_for_opportunity(opp)
        items = []
        for i, intent in enumerate(intents):
            items.append(
                StrategyWithFitness(
                    intent=intent, fitness=_success_fitness(sharpe=float(i))
                )
            )
        # añadimos uno fallido al inicio
        items.insert(
            0,
            StrategyWithFitness(
                intent=intents[0],
                fitness=StrategyFitnessResult(
                    success=False, mode="error", error_code="X"
                ),
            ),
        )
        ranked = rank_by_default_metric(items)
        # el fallido debe ir al final
        assert ranked[-1].fitness.success is False
        # el primero debe ser el de sharpe más alto
        assert ranked[0].fitness.sharpe == max(
            it.fitness.sharpe for it in items if it.fitness.success
        )

    def test_rank_empty(self):
        assert rank_by_default_metric([]) == []
        assert rank_by_default_metric(None) == []  # type: ignore[arg-type]


# ---------------------------------------------------------------------------
# Sección 3 — Batch
# ---------------------------------------------------------------------------


class TestEvaluateBatch:
    def test_multiple_opps(self):
        opps = [
            _make_opp(symbol="SPY", direction="long"),
            _make_opp(symbol="QQQ", asset_class="index", direction="neutral"),
            _make_opp(symbol="X", classification="reject"),
        ]

        def fake(intent):
            # sharpe = score base + offset por símbolo
            base = {"SPY": 1.0, "QQQ": 0.5}
            return _success_fitness(sharpe=base.get(intent.opportunity.symbol, 0.0))

        out = evaluate_strategies_for_opportunities(opps, backtest_fn=fake)
        symbols = {it.intent.opportunity.symbol for it in out}
        assert symbols == {"SPY", "QQQ"}  # X (reject) no aparece
        # SPY (sharpe=1.0) primero
        assert out[0].intent.opportunity.symbol == "SPY"

    def test_batch_none(self):
        assert evaluate_strategies_for_opportunities(None) == []


# ---------------------------------------------------------------------------
# Sección 4 — Aislamiento AST
# ---------------------------------------------------------------------------


_F14_MODULE = Path("atlas_code_quant/strategies/evaluation.py")


_PROHIBITED_IMPORTS = (
    "atlas_code_quant.execution",
    "atlas_code_quant.operations",
    "atlas_code_quant.autonomy",
    "atlas_code_quant.risk",
    "atlas_code_quant.vision",
    "atlas_adapter",
    "tradier",
    "broker_router",
    "live_loop",
    "live_activation",
)


def test_evaluation_has_no_prohibited_imports():
    repo_root = Path(__file__).resolve().parents[2]
    src = (repo_root / _F14_MODULE).read_text("utf-8")
    tree = ast.parse(src)
    bad: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            mod = node.module or ""
            for prohibited in _PROHIBITED_IMPORTS:
                if mod == prohibited or mod.startswith(prohibited + "."):
                    bad.append(mod)
        elif isinstance(node, ast.Import):
            for alias in node.names:
                for prohibited in _PROHIBITED_IMPORTS:
                    if alias.name == prohibited or alias.name.startswith(
                        prohibited + "."
                    ):
                        bad.append(alias.name)
    assert not bad, f"evaluation.py importa prohibidos: {bad}"


def test_evaluation_uses_factory_and_lean():
    repo_root = Path(__file__).resolve().parents[2]
    src = (repo_root / _F14_MODULE).read_text("utf-8")
    assert "from atlas_code_quant.strategies.factory.dispatch import" in src
    assert "from atlas_code_quant.lean.runner.launcher import" in src

"""Tests F16 — Paper execution pipeline (Radar → Strategy → fitness → Tradier).

Cubre:

* Pipeline single-opp con ``evaluator_fn`` y ``adapter`` mock.
* Forzado dry_run=True incluso si el adapter inyectado viene live.
* Filtros: ``min_sharpe``, ``only_evaluated``, ``max_strategies_per_opportunity``.
* Pipeline batch sobre múltiples oportunidades (incluye None / inválidas).
* Defensa: ``evaluator_fn`` que lanza → records vacíos sin propagar.
* Defensa: ``adapter.submit`` que lanza → record con ``submit_ok=False``.
* Mapping ``StrategyIntent → OrderIntent``: multileg vs option, side
  ``buy_to_open``, legs preservadas, trace_id propagado.
* AST guard: módulo F16 NO importa autonomy / live_* / broker_router /
  signal_executor / operation_center / tradier_execution canónico /
  atlas_adapter; ningún identificador ``paper_only`` /
  ``full_live_globally_locked`` como Name/Attribute.
"""

from __future__ import annotations

import ast
from pathlib import Path
from typing import Any

import pytest

from atlas_code_quant.execution.tradier_adapter import (
    OrderIntent,
    OrderResult,
    TradierAdapter,
    TradierAdapterConfig,
)
from atlas_code_quant.intake.opportunity import RadarOpportunityInternal
from atlas_code_quant.lean.runner.launcher import StrategyFitnessResult
from atlas_code_quant.operations.paper_pipeline import (
    PaperPipelineConfig,
    PaperPipelineRecord,
    PaperPipelineReport,
    _strategy_intent_to_order_intent,
    run_paper_pipeline_for_opportunities,
    run_paper_pipeline_for_opportunity,
)
from atlas_code_quant.strategies.evaluation import StrategyWithFitness
from atlas_code_quant.strategies.factory.dispatch import (
    build_strategies_for_opportunity,
)
from atlas_code_quant.strategies.options.intent import StrategyIntent


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_opp(
    *,
    symbol: str = "SPY",
    direction: str = "long",
    asset_class: str = "etf",
    horizon_min: int | None = 240,
    classification: str = "high_conviction",
    score: float = 80.0,
    trace_id: str = "trace-f16",
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
            "trace_id": trace_id,
            "timestamp": "2026-04-27T13:00:00+00:00",
        }
    )


def _success_fitness(
    *, sharpe: float = 1.0, expectancy: float = 0.1, win_rate: float = 50.0
) -> StrategyFitnessResult:
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


def _fail_fitness(*, code: str = "X") -> StrategyFitnessResult:
    return StrategyFitnessResult(
        success=False, mode="error", error_code=code, error_message="boom"
    )


class _RecordingAdapter(TradierAdapter):
    """Adapter espía: registra los OrderIntent recibidos y devuelve OK dry-run."""

    def __init__(self, *, dry_run: bool = True) -> None:
        super().__init__(TradierAdapterConfig(dry_run=dry_run))
        self.calls: list[OrderIntent] = []

    def submit(self, intent: OrderIntent) -> OrderResult:  # type: ignore[override]
        self.calls.append(intent)
        return OrderResult(
            ok=True,
            dry_run=True,
            order_id=f"DRY-{intent.idempotency_key()[:8]}",
            attempts=1,
            idempotency_key=intent.idempotency_key(),
        )


class _RaisingAdapter(TradierAdapter):
    def __init__(self) -> None:
        super().__init__(TradierAdapterConfig(dry_run=True))

    def submit(self, intent: OrderIntent) -> OrderResult:  # type: ignore[override]
        raise RuntimeError("simulated transport blow-up")


# ---------------------------------------------------------------------------
# Sección 1 — Pipeline single-opp
# ---------------------------------------------------------------------------


class TestPipelineSingle:
    def test_happy_path_one_record(self):
        opp = _make_opp()
        adapter = _RecordingAdapter()

        def evaluator(opp_, *, min_score, backtest_fn=None, rank=True):
            intents = build_strategies_for_opportunity(opp_, min_score=min_score)
            return [
                StrategyWithFitness(
                    intent=intents[0], fitness=_success_fitness(sharpe=1.5)
                )
            ]

        report = run_paper_pipeline_for_opportunity(
            opp, adapter=adapter, evaluator_fn=evaluator
        )
        assert isinstance(report, PaperPipelineReport)
        assert report.total_opportunities == 1
        assert report.total_intents == 1
        assert report.total_submitted == 1
        assert report.total_ok == 1
        assert len(report.records) == 1
        rec = report.records[0]
        assert rec.opportunity_symbol == "SPY"
        assert rec.submit_ok is True
        assert rec.dry_run is True
        assert rec.fitness_success is True
        assert rec.sharpe == pytest.approx(1.5)
        assert rec.trace_id == "trace-f16"
        assert rec.order_id and rec.order_id.startswith("DRY-")

    def test_none_opp_returns_empty_report(self):
        rep = run_paper_pipeline_for_opportunity(None)
        assert rep.total_opportunities == 0
        assert rep.total_intents == 0
        assert rep.total_submitted == 0
        assert rep.records == ()

    def test_invalid_opp_type_returns_empty_report(self):
        rep = run_paper_pipeline_for_opportunity({"symbol": "SPY"})  # type: ignore[arg-type]
        assert rep.total_opportunities == 0
        assert rep.records == ()

    def test_no_intents_returns_empty_records(self):
        opp = _make_opp(classification="reject")  # F12 lo descarta
        rep = run_paper_pipeline_for_opportunity(opp)
        assert rep.total_intents == 0
        assert rep.total_submitted == 0
        assert rep.records == ()


# ---------------------------------------------------------------------------
# Sección 2 — Forzado dry-run
# ---------------------------------------------------------------------------


class TestDryRunEnforced:
    def test_default_adapter_is_dry_run(self):
        opp = _make_opp()
        # sin pasar adapter: F16 construye uno default dry-run
        called: dict[str, Any] = {}

        def evaluator(opp_, *, min_score, backtest_fn=None, rank=True):
            intents = build_strategies_for_opportunity(opp_, min_score=min_score)
            return [
                StrategyWithFitness(
                    intent=intents[0], fitness=_success_fitness()
                )
            ]

        rep = run_paper_pipeline_for_opportunity(opp, evaluator_fn=evaluator)
        assert rep.records[0].dry_run is True

    def test_live_adapter_is_forced_to_dry_run(self):
        opp = _make_opp()
        live = _RecordingAdapter(dry_run=False)
        assert live.config.dry_run is False  # entrada live

        def evaluator(opp_, *, min_score, backtest_fn=None, rank=True):
            intents = build_strategies_for_opportunity(opp_, min_score=min_score)
            return [
                StrategyWithFitness(
                    intent=intents[0], fitness=_success_fitness()
                )
            ]

        rep = run_paper_pipeline_for_opportunity(
            opp, adapter=live, evaluator_fn=evaluator
        )
        # F16 no debe haber usado el live adapter; debe haber re-instanciado
        # uno dry-run y por tanto el espía live NO debe haber recibido nada.
        assert live.calls == []
        assert all(r.dry_run is True for r in rep.records)


# ---------------------------------------------------------------------------
# Sección 3 — Filtros
# ---------------------------------------------------------------------------


class TestFilters:
    def _evaluator_three_intents(self, sharpes=(2.0, 1.0, -0.5), success=(True, True, True)):
        def evaluator(opp_, *, min_score, backtest_fn=None, rank=True):
            intents = build_strategies_for_opportunity(opp_, min_score=min_score)
            # repetimos el primer intent 3 veces si hace falta
            base = intents * (1 + (3 // max(len(intents), 1)))
            base = base[:3]
            out = []
            for intent, sh, ok in zip(base, sharpes, success):
                fit = (
                    _success_fitness(sharpe=sh)
                    if ok
                    else _fail_fitness(code="EVAL_FAILED")
                )
                out.append(StrategyWithFitness(intent=intent, fitness=fit))
            return out

        return evaluator

    def test_only_evaluated_filters_failed(self):
        opp = _make_opp()
        ev = self._evaluator_three_intents(
            sharpes=(2.0, 1.0, -0.5), success=(True, False, True)
        )
        rep = run_paper_pipeline_for_opportunity(
            opp,
            adapter=_RecordingAdapter(),
            evaluator_fn=ev,
            config=PaperPipelineConfig(
                only_evaluated=True,
                min_sharpe=None,
                max_strategies_per_opportunity=None,
            ),
        )
        # 1 fallido descartado → quedan 2
        assert len(rep.records) == 2
        assert all(r.fitness_success for r in rep.records)

    def test_min_sharpe_filters_low(self):
        opp = _make_opp()
        ev = self._evaluator_three_intents(
            sharpes=(2.0, 1.0, -0.5), success=(True, True, True)
        )
        rep = run_paper_pipeline_for_opportunity(
            opp,
            adapter=_RecordingAdapter(),
            evaluator_fn=ev,
            config=PaperPipelineConfig(
                only_evaluated=True,
                min_sharpe=0.5,
                max_strategies_per_opportunity=None,
            ),
        )
        # -0.5 descartado
        assert len(rep.records) == 2
        assert all(r.sharpe >= 0.5 for r in rep.records)

    def test_max_strategies_per_opportunity_caps(self):
        opp = _make_opp()
        ev = self._evaluator_three_intents(sharpes=(2.0, 1.0, 0.5))
        rep = run_paper_pipeline_for_opportunity(
            opp,
            adapter=_RecordingAdapter(),
            evaluator_fn=ev,
            config=PaperPipelineConfig(
                only_evaluated=True,
                min_sharpe=None,
                max_strategies_per_opportunity=1,
            ),
        )
        assert len(rep.records) == 1
        # debería tomar el primero del listado evaluado (ya rankeado por F14)
        assert rep.records[0].sharpe == pytest.approx(2.0)


# ---------------------------------------------------------------------------
# Sección 4 — Batch
# ---------------------------------------------------------------------------


class TestBatch:
    def test_multiple_opps(self):
        opps = [
            _make_opp(symbol="SPY", direction="long"),
            _make_opp(symbol="QQQ", asset_class="index", direction="neutral"),
            _make_opp(symbol="JUNK", classification="reject"),
        ]

        def ev(opp_, *, min_score, backtest_fn=None, rank=True):
            intents = build_strategies_for_opportunity(opp_, min_score=min_score)
            return [
                StrategyWithFitness(intent=i, fitness=_success_fitness())
                for i in intents
            ]

        adapter = _RecordingAdapter()
        rep = run_paper_pipeline_for_opportunities(
            opps,
            adapter=adapter,
            evaluator_fn=ev,
            config=PaperPipelineConfig(
                max_strategies_per_opportunity=1, min_sharpe=None
            ),
        )
        symbols = {r.opportunity_symbol for r in rep.records}
        assert "JUNK" not in symbols
        assert {"SPY", "QQQ"}.issubset(symbols)
        # los 3 opps son válidos; JUNK no genera intents pero cuenta como visto
        assert rep.total_opportunities == 3
        assert rep.total_submitted == len(rep.records)
        # JUNK genera 0 intents y 0 submissions; SPY+QQQ cubren todos los records
        assert all(r.opportunity_symbol in {"SPY", "QQQ"} for r in rep.records)

    def test_batch_none(self):
        rep = run_paper_pipeline_for_opportunities(None)
        assert rep.total_opportunities == 0
        assert rep.records == ()


# ---------------------------------------------------------------------------
# Sección 5 — Defensa: errores upstream/downstream
# ---------------------------------------------------------------------------


class TestDefensive:
    def test_evaluator_raises_yields_empty_records(self):
        opp = _make_opp()

        def bad_eval(*args, **kwargs):
            raise RuntimeError("evaluator boom")

        rep = run_paper_pipeline_for_opportunity(opp, evaluator_fn=bad_eval)
        assert rep.total_intents == 0
        assert rep.total_submitted == 0
        assert rep.records == ()

    def test_adapter_raises_records_failure(self):
        opp = _make_opp()

        def ev(opp_, *, min_score, backtest_fn=None, rank=True):
            intents = build_strategies_for_opportunity(opp_, min_score=min_score)
            return [
                StrategyWithFitness(
                    intent=intents[0], fitness=_success_fitness()
                )
            ]

        rep = run_paper_pipeline_for_opportunity(
            opp, adapter=_RaisingAdapter(), evaluator_fn=ev
        )
        assert len(rep.records) == 1
        rec = rep.records[0]
        assert rec.submit_ok is False
        assert rec.error_code == "PAPER_PIPELINE_ADAPTER_RAISED"
        assert "blow-up" in (rec.error_message or "")


# ---------------------------------------------------------------------------
# Sección 6 — Mapping StrategyIntent → OrderIntent
# ---------------------------------------------------------------------------


class TestMapping:
    def _intent_for(self, opp: RadarOpportunityInternal) -> StrategyIntent:
        intents = build_strategies_for_opportunity(opp, min_score=70.0)
        assert intents, "factory debería generar al menos un intent"
        return intents[0]

    def test_multileg_when_more_than_one_leg(self):
        opp = _make_opp(direction="long")
        intent = self._intent_for(opp)
        # forzamos un intent multileg si no lo es ya
        if intent.num_legs <= 1:
            # busca uno multileg
            found = next(
                (
                    i
                    for i in build_strategies_for_opportunity(
                        opp, min_score=70.0
                    )
                    if i.num_legs > 1
                ),
                None,
            )
            if found is not None:
                intent = found
        order = _strategy_intent_to_order_intent(intent, default_quantity=1)
        if intent.num_legs > 1:
            assert order.asset_class == "multileg"
        else:
            assert order.asset_class == "option"
        assert order.side == "buy_to_open"
        assert order.symbol == intent.opportunity.symbol
        assert order.trace_id == intent.opportunity.trace_id
        assert order.order_type == "market"
        assert order.duration == "day"
        assert order.is_valid()

    def test_legs_preserved_as_dicts(self):
        opp = _make_opp(direction="long")
        # buscamos un multileg explícitamente
        intents = build_strategies_for_opportunity(opp, min_score=70.0)
        multileg = next((i for i in intents if i.num_legs > 1), None)
        if multileg is None:
            pytest.skip("no multileg generated for this opportunity")
        order = _strategy_intent_to_order_intent(multileg, default_quantity=1)
        assert len(order.legs) == multileg.num_legs
        for od_leg, raw_leg in zip(order.legs, multileg.legs):
            assert od_leg["side"] == raw_leg.side
            assert od_leg["right"] == raw_leg.right
            assert od_leg["strike_offset_steps"] == raw_leg.strike_offset_steps
            assert od_leg["expiry_rel_dte"] == raw_leg.expiry_rel_dte

    def test_metadata_propagates_strategy_type(self):
        opp = _make_opp()
        intent = self._intent_for(opp)
        order = _strategy_intent_to_order_intent(intent, default_quantity=2)
        assert order.metadata["strategy_type"] == intent.strategy_type
        assert order.quantity == 2


# ---------------------------------------------------------------------------
# Sección 7 — Aislamiento AST
# ---------------------------------------------------------------------------


_F16_MODULE = Path("atlas_code_quant/operations/paper_pipeline.py")


_PROHIBITED_IMPORTS = (
    "atlas_code_quant.operations.auton_executor",
    "atlas_code_quant.operations.live_authorization",
    "atlas_code_quant.operations.live_loop",
    "atlas_code_quant.operations.live_switch",
    "atlas_code_quant.operations.operation_center",
    "atlas_code_quant.operations.signal_executor",
    "atlas_code_quant.operations.start_paper_trading",
    "atlas_code_quant.production.live_activation",
    "atlas_code_quant.execution.broker_router",
    "atlas_code_quant.execution.tradier_execution",
    "atlas_code_quant.execution.tradier_controls",
    "atlas_code_quant.execution.tradier_pdt_ledger",
    "atlas_adapter",
    "live_loop",
    "live_activation",
)


def _read_module() -> str:
    repo_root = Path(__file__).resolve().parents[2]
    return (repo_root / _F16_MODULE).read_text("utf-8")


def test_paper_pipeline_has_no_prohibited_imports():
    src = _read_module()
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
    assert not bad, f"paper_pipeline.py importa prohibidos: {bad}"


def test_paper_pipeline_does_not_reference_locks_as_code():
    src = _read_module()
    tree = ast.parse(src)
    for node in ast.walk(tree):
        if isinstance(node, ast.Name):
            assert node.id not in {"paper_only", "full_live_globally_locked"}, (
                f"paper_pipeline.py usa lock como Name: {node.id}"
            )
        if isinstance(node, ast.Attribute):
            assert node.attr not in {"paper_only", "full_live_globally_locked"}, (
                f"paper_pipeline.py usa lock como Attribute: {node.attr}"
            )


def test_paper_pipeline_uses_expected_dependencies():
    src = _read_module()
    assert "from atlas_code_quant.execution.tradier_adapter import" in src
    assert "from atlas_code_quant.strategies.evaluation import" in src
    assert "from atlas_code_quant.strategies.options.intent import" in src
    assert "from atlas_code_quant.intake.opportunity import" in src

"""E2E paper trading cycle — F9.

Ejecuta los 11 pasos del experimento y guarda traza JSON.

trace_id: e2e_paper_trade_20260427_001

Uso:
    PYTHONPATH=. ATLAS_LIVE_TRADING_ENABLED=false ATLAS_TRADIER_DRY_RUN=true \\
        python3 scripts/e2e_paper_trade.py

Salida:
    reports/_e2e_paper_trade_trace.json
"""
from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from atlas_code_quant.backtest.engine import BacktestEngine  # noqa: E402
from atlas_code_quant.execution.position_monitor import PositionMonitor  # noqa: E402
from atlas_code_quant.execution.runtime import (  # noqa: E402
    get_paper_broker,
    get_trade_journal,
    reset_runtime,
)
from atlas_code_quant.strategies.contracts import (  # noqa: E402
    StrategyOpportunityRef,
    StrategyPlan,
)
from atlas_code_quant.strategies.factory import StrategyFactory  # noqa: E402
from atlas_code_quant.strategies.ranker import (  # noqa: E402
    RankerPolicy,
    rank_strategies,
)


TRACE_ID = "e2e_paper_trade_20260427_001"


def _log(event: str, **fields):
    rec = {"event": event, "trace_id": TRACE_ID, "ts": time.time(), **fields}
    print(json.dumps(rec, default=str))
    return rec


def main() -> int:
    os.environ.setdefault("ATLAS_LIVE_TRADING_ENABLED", "false")
    os.environ.setdefault("ATLAS_TRADIER_DRY_RUN", "true")
    os.environ.setdefault("ATLAS_LEAN_ENABLED", "false")

    reset_runtime()
    timeline = []

    # === PASO 1: Universo Radar ===
    from atlas_adapter.services.universe_provider import UniverseProvider
    universe = UniverseProvider().refresh()
    symbols = [u.symbol for u in universe]
    timeline.append(_log(
        "step_01_radar_universe",
        universe_size=len(symbols),
        symbols_sample=symbols[:15],
    ))
    assert len(symbols) >= 5, "universo debe tener ≥5 símbolos"

    # === PASO 2: Selección oportunidad SPY ===
    opp = StrategyOpportunityRef(
        symbol="SPY",
        score=82.0,
        direction="neutral",
        horizon_min=30,
        trace_id=TRACE_ID,
    )
    timeline.append(_log(
        "step_02_opportunity_selected",
        symbol=opp.symbol,
        score=opp.score,
        direction=opp.direction,
    ))

    # === PASO 3: build_candidates ===
    candidates: list[StrategyPlan] = StrategyFactory.build_candidates(opp)
    timeline.append(_log(
        "step_03_candidates_built",
        candidate_count=len(candidates),
        strategies=[c.strategy for c in candidates],
    ))
    assert len(candidates) >= 2, f"≥2 candidatas requeridas, got {len(candidates)}"

    # === PASO 4: Backtest comparativo ===
    bt_engine = BacktestEngine()
    bt_results = []
    raw_results = []
    for plan in candidates:
        r = bt_engine.evaluate(plan, opportunity_score=opp.score, trace_id=TRACE_ID)
        raw_results.append(r)
        bt_results.append({
            "strategy": r.strategy,
            "trades_count": r.trades_count,
            "profit_factor": round(r.profit_factor, 3),
            "win_rate": round(r.win_rate, 3),
            "sharpe": round(r.sharpe, 3),
            "max_drawdown": round(r.max_drawdown, 4),
            "expectancy": round(r.expectancy, 2),
            "total_pnl": round(r.total_pnl, 2),
            "source": r.source,
            "rejected": r.rejected,
        })
    timeline.append(_log(
        "step_04_backtest_comparative",
        results=bt_results,
    ))

    # === PASO 5: Strategy ranker ===
    policy = RankerPolicy()
    ranking_outcome = rank_strategies(raw_results, policy=policy, liquidity_score=0.85)
    ranking_summary = [
        {
            "strategy": r.strategy,
            "accepted": r.accepted,
            "fitness": round(r.fitness, 4),
            "rejection_reasons": list(r.rejection_reasons),
        }
        for r in ranking_outcome.ranked
    ]
    timeline.append(_log(
        "step_05_ranking",
        outcomes=ranking_summary,
        winner_strategy=ranking_outcome.winner.strategy if ranking_outcome.winner else None,
        justification=ranking_outcome.justification[:400],
    ))

    # === PASO 6: Estrategia ganadora ===
    if ranking_outcome.winner is None:
        # Fallback: tomar la mejor por fitness aunque no pase rúbrica
        ranked_sorted = sorted(
            ranking_outcome.ranked, key=lambda x: x.fitness, reverse=True
        )
        winner_meta = ranked_sorted[0] if ranked_sorted else None
        winner_reason = "fallback_best_fitness_no_acceptance"
    else:
        winner_meta = ranking_outcome.winner
        winner_reason = "passed_rubric_top_fitness"
    assert winner_meta is not None
    # Ubicar el plan correspondiente
    winning_plan = next(
        (p for p in candidates if p.strategy == winner_meta.strategy),
        candidates[0],
    )
    timeline.append(_log(
        "step_06_winner",
        strategy=winner_meta.strategy,
        fitness=round(winner_meta.fitness, 4),
        accepted=winner_meta.accepted,
        reason=winner_reason,
    ))

    # === PASO 7: Apertura paper ===
    assert winning_plan.is_actionable(), (
        f"plan no actionable status={winning_plan.status}"
    )
    entry_price = 2.00
    broker = get_paper_broker()
    journal = get_trade_journal()
    pos = broker.open_position(
        winning_plan,
        entry_price=entry_price,
        take_profit_pct=0.50,
        stop_loss_pct=1.00,
        time_stop_seconds=1800,
        trace_id=TRACE_ID,
    )
    journal.record_open(pos)
    timeline.append(_log(
        "step_07_paper_open",
        position_id=pos.position_id,
        symbol=pos.symbol,
        strategy=pos.strategy,
        entry_price=pos.entry_price,
        take_profit_price=pos.take_profit_price,
        stop_loss_price=pos.stop_loss_price,
        qty=pos.qty,
    ))

    # === PASO 8: Simulación de ticks → monitor ===
    monitor = PositionMonitor()
    tick_prices = [2.05, 2.15, 2.35, 2.65, 2.95, 3.05]
    decisions = []
    final_decision = None
    for tp_price in tick_prices:
        d = monitor.evaluate(pos, last_price=tp_price)
        decisions.append({
            "tick_price": tp_price,
            "action": d.action,
            "reason": d.reason,
        })
        if d.action == "close":
            final_decision = d
            break
    timeline.append(_log(
        "step_08_monitoring_ticks",
        ticks=decisions,
        final_action=final_decision.action if final_decision else "none",
        final_reason=final_decision.reason if final_decision else "none",
    ))
    assert final_decision is not None and final_decision.reason == "take_profit"

    # === PASO 9: Cierre paper con realized PnL ===
    closed = broker.close_position(
        pos.position_id,
        exit_price=final_decision.suggested_exit_price or 3.0,
        reason=final_decision.reason,
    )
    journal.record_close(closed)
    timeline.append(_log(
        "step_09_paper_close",
        position_id=closed.position_id,
        exit_price=closed.exit_price,
        exit_reason=closed.exit_reason,
        realized_pnl_usd=closed.realized_pnl_usd,
        status=closed.status,
    ))

    # === PASO 10: Verificar journal ===
    cycle_complete = journal.has_complete_cycle(TRACE_ID)
    entries = journal.entries(trace_id=TRACE_ID)
    timeline.append(_log(
        "step_10_journal_verification",
        complete_cycle=cycle_complete,
        entries_count=len(entries),
        events=[e.event for e in entries],
    ))
    assert cycle_complete

    # === PASO 11: Snapshot final ===
    timeline.append(_log(
        "step_11_runtime_snapshot",
        cash_usd=round(broker.cash_usd, 2),
        positions_open=len(broker.open_positions()),
        positions_closed=len(broker.closed_positions()),
        realized_pnl_total=round(
            sum(p.realized_pnl_usd or 0 for p in broker.closed_positions()), 2
        ),
        journal_total_entries=len(journal.entries()),
    ))

    # Persistir traza
    out_path = REPO / "reports" / "_e2e_paper_trade_trace.json"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump({
            "trace_id": TRACE_ID,
            "started_at": timeline[0]["ts"],
            "finished_at": timeline[-1]["ts"],
            "duration_sec": timeline[-1]["ts"] - timeline[0]["ts"],
            "winner_strategy": winner_meta.strategy,
            "winner_fitness": winner_meta.fitness,
            "winner_accepted": winner_meta.accepted,
            "winner_justification": ranking_outcome.justification,
            "realized_pnl_usd": closed.realized_pnl_usd,
            "complete_cycle": cycle_complete,
            "timeline": timeline,
            "backtest_table": bt_results,
            "ranking_table": ranking_summary,
        }, f, indent=2, default=str)
    print(f"\n[OK] trace persisted at {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

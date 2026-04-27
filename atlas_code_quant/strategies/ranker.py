"""Strategy Ranker — F9.2.

Combina ``BacktestResult`` con criterios de calidad y devuelve un ranking
ordenado por fitness. Política de rechazo:

- ``profit_factor < 1.10``
- ``win_rate < 0.53``
- ``max_drawdown > 0.08`` (8 %)
- ``trades_count < 10``

Fitness composite (suma de pesos = 1.0):
    0.25 * normalize(PF, 1.0..3.0)
  + 0.20 * normalize(Sharpe, -1.0..3.0)
  + 0.20 * win_rate
  + 0.15 * (1 - min(MaxDD/0.08, 1.0))
  + 0.10 * normalize(Expectancy, -50..200)
  + 0.10 * liquidity_score

``liquidity_score`` se pasa explícitamente desde el caller cuando se conoce.
Por defecto = 0.7 para entornos paper sin chain real.
"""
from __future__ import annotations

from dataclasses import dataclass, field, asdict
from typing import Any

from atlas_code_quant.backtest.engine import BacktestResult


# ---------------------------------------------------------------------------
# Política de rechazo
# ---------------------------------------------------------------------------


@dataclass(slots=True)
class RankerPolicy:
    min_profit_factor: float = 1.10
    min_win_rate: float = 0.53
    max_drawdown: float = 0.08
    min_trades: int = 10


def _normalize(x: float, lo: float, hi: float) -> float:
    if hi <= lo:
        return 0.0
    n = (x - lo) / (hi - lo)
    return max(0.0, min(1.0, n))


def compute_fitness(
    result: BacktestResult,
    *,
    liquidity_score: float = 0.7,
) -> float:
    """Devuelve el fitness composite [0..1] para un BacktestResult."""
    if result.rejected or result.trades_count == 0:
        return 0.0
    pf_n = _normalize(result.profit_factor, 1.0, 3.0)
    sh_n = _normalize(result.sharpe, -1.0, 3.0)
    wr_n = max(0.0, min(1.0, result.win_rate))
    dd_n = 1.0 - min(result.max_drawdown / 0.08, 1.0)
    exp_n = _normalize(result.expectancy, -50.0, 200.0)
    liq_n = max(0.0, min(1.0, liquidity_score))

    fitness = (
        0.25 * pf_n
        + 0.20 * sh_n
        + 0.20 * wr_n
        + 0.15 * dd_n
        + 0.10 * exp_n
        + 0.10 * liq_n
    )
    return round(fitness, 6)


# ---------------------------------------------------------------------------
# Resultado del ranking
# ---------------------------------------------------------------------------


@dataclass(slots=True)
class RankedStrategy:
    """Una estrategia evaluada con su fitness y motivos."""

    strategy: str
    symbol: str
    fitness: float
    accepted: bool
    rejection_reasons: list[str] = field(default_factory=list)
    backtest: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class RankerOutcome:
    """Salida completa del ranker."""

    ranked: list[RankedStrategy]
    winner: RankedStrategy | None
    justification: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "ranked": [r.to_dict() for r in self.ranked],
            "winner": self.winner.to_dict() if self.winner else None,
            "justification": self.justification,
        }


# ---------------------------------------------------------------------------
# Ranker
# ---------------------------------------------------------------------------


def _evaluate_acceptance(
    result: BacktestResult,
    policy: RankerPolicy,
) -> tuple[bool, list[str]]:
    if result.rejected:
        return False, list(result.rejection_reasons or ["plan_rejected"])
    reasons: list[str] = []
    if result.profit_factor < policy.min_profit_factor:
        reasons.append(
            f"profit_factor_low pf={result.profit_factor:.2f} "
            f"min={policy.min_profit_factor:.2f}"
        )
    if result.win_rate < policy.min_win_rate:
        reasons.append(
            f"win_rate_low wr={result.win_rate:.2f} "
            f"min={policy.min_win_rate:.2f}"
        )
    if result.max_drawdown > policy.max_drawdown:
        reasons.append(
            f"max_drawdown_high dd={result.max_drawdown:.2f} "
            f"max={policy.max_drawdown:.2f}"
        )
    if result.trades_count < policy.min_trades:
        reasons.append(
            f"trades_too_few n={result.trades_count} "
            f"min={policy.min_trades}"
        )
    return (not reasons), reasons


def rank_strategies(
    results: list[BacktestResult],
    *,
    policy: RankerPolicy | None = None,
    liquidity_score: float = 0.7,
) -> RankerOutcome:
    """Ordena resultados por fitness y devuelve ganadora + justificación.

    Si ningún candidato es aceptado, ``winner`` es ``None`` y la
    justificación explica por qué.
    """
    pol = policy or RankerPolicy()
    ranked: list[RankedStrategy] = []
    for r in results:
        accepted, reasons = _evaluate_acceptance(r, pol)
        fit = compute_fitness(r, liquidity_score=liquidity_score) if accepted else 0.0
        ranked.append(
            RankedStrategy(
                strategy=r.strategy,
                symbol=r.symbol,
                fitness=fit,
                accepted=accepted,
                rejection_reasons=reasons,
                backtest=r.to_dict(),
            )
        )

    ranked.sort(key=lambda x: (x.accepted, x.fitness), reverse=True)
    accepted_pool = [r for r in ranked if r.accepted]
    winner = accepted_pool[0] if accepted_pool else None

    if winner is None:
        if ranked:
            top = ranked[0]
            justification = (
                f"No candidate met acceptance policy. Best fail: "
                f"{top.strategy} on {top.symbol} ({'; '.join(top.rejection_reasons)})"
            )
        else:
            justification = "No candidates evaluated."
    else:
        loser_lines: list[str] = []
        for r in ranked:
            if r is winner or not r.accepted:
                continue
            delta = round(winner.fitness - r.fitness, 4)
            loser_lines.append(
                f"  - {r.strategy}: fitness={r.fitness:.4f} "
                f"(Δ={delta:+.4f} vs winner)"
            )
        rejected_lines = [
            f"  - {r.strategy}: rejected ({'; '.join(r.rejection_reasons)})"
            for r in ranked if not r.accepted
        ]
        bt = winner.backtest
        justification = (
            f"Winner: {winner.strategy} on {winner.symbol} "
            f"with fitness={winner.fitness:.4f}. "
            f"PF={bt.get('profit_factor'):.2f} | WR={bt.get('win_rate'):.2f} | "
            f"Sharpe={bt.get('sharpe'):.2f} | MaxDD={bt.get('max_drawdown'):.2f} | "
            f"trades={bt.get('trades_count')}.\n"
            + ("Other accepted:\n" + "\n".join(loser_lines) if loser_lines else "")
            + ("\nRejected:\n" + "\n".join(rejected_lines) if rejected_lines else "")
        )

    return RankerOutcome(
        ranked=ranked,
        winner=winner,
        justification=justification.strip(),
    )

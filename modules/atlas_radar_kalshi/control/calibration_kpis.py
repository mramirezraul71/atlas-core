"""KPIs de calibración post-trade (Brier, EV aprox, drift de edge)."""
from __future__ import annotations

from collections import defaultdict
from typing import Any

from pydantic import BaseModel, Field

from ..state.journal import Journal


class CalibrationKPIs(BaseModel):
    n_pairs: int = 0
    brier_score: float = 0.0
    ev_proxy_cents: float = 0.0
    pnl_realized_cents: float = 0.0
    ev_gap_cents: float = 0.0
    drift_edge_prefix: dict[str, float] = Field(default_factory=dict)


def _last_p_readout(decisions_rows: list[dict], ticker: str) -> tuple[float, str] | None:
    for row in reversed(decisions_rows):
        if str(row.get("ticker", "")) != ticker:
            continue
        gate = row.get("gate") or {}
        readout = row.get("readout") or {}
        p = readout.get("p_ensemble")
        if p is None:
            continue
        side = gate.get("side") or "YES"
        return float(p), str(side).upper()
    return None


def compute_calibration_kpis(
    journal: Journal, *, max_decisions: int = 20_000
) -> CalibrationKPIs:
    """Empareja `decisions` + `exits` por ticker (última decisión antes del cierre)."""
    exits = journal.read("exits", limit=10_000)
    decisions = journal.read("decisions", limit=max_decisions)
    if not exits or not decisions:
        return CalibrationKPIs()

    brier_terms: list[float] = []
    ev_proxy: list[float] = []
    pnl_sum: list[float] = []
    drift: dict[str, list[float]] = defaultdict(list)

    for ex in exits:
        t = str(ex.get("ticker", ""))
        pnl = float(int(ex.get("pnl_cents", 0)))
        pair = _last_p_readout(decisions, t)
        if pair is None:
            continue
        p, side = pair
        outcome = 1.0 if pnl > 0 else 0.0
        p_hat = p if side == "YES" else (1.0 - p)
        brier_terms.append((p_hat - outcome) ** 2)
        ev_proxy.append((2.0 * p_hat - 1.0) * 100.0)
        pnl_sum.append(pnl)
        drift[t[:6]].append(float(ex.get("entry", 50)) / 100.0 - p_hat)

    n = len(brier_terms)
    if n == 0:
        return CalibrationKPIs()

    brier = sum(brier_terms) / n
    sum_ev = float(sum(ev_proxy))
    sum_pnl = float(sum(pnl_sum))
    drift_m = {k: round(sum(v) / len(v), 5) for k, v in list(drift.items())[:20]}
    return CalibrationKPIs(
        n_pairs=n,
        brier_score=round(brier, 6),
        ev_proxy_cents=round(sum_ev, 2),
        pnl_realized_cents=round(sum_pnl, 2),
        ev_gap_cents=round(sum_pnl - sum_ev, 2),
        drift_edge_prefix=drift_m,
    )

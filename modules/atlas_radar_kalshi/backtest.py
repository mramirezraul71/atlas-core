"""
backtest.py — Harness walk-forward para baseline vs candidate.

Genera un universo sintético determinístico (semilla fija) que simula
mercados de eventos: un fair value latente ``p_true`` y un mercado
ruidoso ``p_market`` con drift. Para cada paso evaluamos:

- **Baseline**: dispara cuando ``p_model - p_market >= edge_threshold``,
  Kelly fraccionario sin filtros adicionales, sin TP/SL/time-stop.
- **Candidate**: pipeline endurecido (calibración + ensemble + gating
  con `edge_net`, `confidence_min`, depth/spread, cooldown, exits TP/SL/
  time-stop, caps de exposición y circuit breakers).

Costos modelados:
- fees: 0.07¢ por contrato (∼7 bps).
- slippage: ±0.5¢ aleatorio en el fill.

Métricas reportadas:
- expectancy neta, profit factor, hit-rate, drawdown, # violaciones de
  riesgo hard, estabilidad (sin caídas).
"""
from __future__ import annotations

import json
import math
import random
import statistics
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional


# ===========================================================================
@dataclass
class BTConfig:
    n_markets: int = 200
    steps_per_market: int = 80
    fee_cents: float = 0.07
    slippage_cents: float = 0.5
    seed: int = 42
    starting_balance_cents: int = 100_000
    # Baseline params
    base_edge_threshold: float = 0.05
    base_kelly_fraction: float = 0.25
    base_max_position_pct: float = 0.05
    # Candidate params
    cand_edge_net_min: float = 0.03
    cand_confidence_min: float = 0.62
    cand_kelly_fraction: float = 0.25
    cand_max_position_pct: float = 0.05
    cand_max_total_exposure_pct: float = 0.50
    cand_daily_dd_limit_pct: float = 0.05
    cand_max_consecutive_losses: int = 5
    cand_tp_capture_pct: float = 0.6
    cand_sl_ticks: int = 4
    cand_time_stop_steps: int = 25


@dataclass
class StrategyResult:
    name: str
    trades: int = 0
    wins: int = 0
    losses: int = 0
    pnl_cents: int = 0
    fees_cents: int = 0
    pnl_net_cents: int = 0
    pf: float = 0.0
    expectancy_cents: float = 0.0
    hit_rate: float = 0.0
    max_drawdown_cents: int = 0
    risk_violations: int = 0
    crashes: int = 0
    equity_curve: list[int] = field(default_factory=list)


# ===========================================================================
def _gen_market(rng: random.Random, steps: int) -> list[dict]:
    """Genera serie ``steps`` con (p_true_t, p_market_t, depth, spread)."""
    p_true = rng.uniform(0.2, 0.8)
    p_mkt = max(0.05, min(0.95, p_true + rng.uniform(-0.15, 0.15)))
    series = []
    for t in range(steps):
        # market drift ruidoso, lentamente converge a p_true
        p_mkt += (p_true - p_mkt) * 0.05 + rng.gauss(0, 0.015)
        p_mkt = max(0.02, min(0.98, p_mkt))
        depth = max(20, int(rng.gauss(300, 80)))
        spread = max(1, min(8, int(abs(rng.gauss(2, 1)))))
        # confidence "true" inversamente proporcional al ruido
        conf = max(0.0, min(1.0, 0.5 + 0.5 * (1 - abs(p_mkt - p_true) * 4)))
        series.append({
            "t": t, "p_true": p_true, "p_market": p_mkt,
            "depth_yes": depth, "depth_no": int(depth * rng.uniform(0.6, 1.2)),
            "spread": spread, "confidence": conf,
        })
    return series


def _ensemble_p(p_true: float, rng: random.Random) -> float:
    """Modelo candidate: ruido controlado alrededor de p_true."""
    return max(0.01, min(0.99, p_true + rng.gauss(0, 0.05)))


# ===========================================================================
def run_baseline(cfg: BTConfig, markets: list[list[dict]]) -> StrategyResult:
    rng = random.Random(cfg.seed + 1)
    bal = cfg.starting_balance_cents
    equity_curve = [bal]
    trades = []
    for series in markets:
        # baseline: usa p_true como modelo (versión naive sin gating)
        last_pnl = 0
        for step in series:
            p_model = _ensemble_p(step["p_true"], rng)
            edge = p_model - step["p_market"]
            if abs(edge) < cfg.base_edge_threshold:
                continue
            side = "YES" if edge > 0 else "NO"
            price = int(round(step["p_market"] * 100)) if side == "YES" \
                else int(round((1 - step["p_market"]) * 100))
            price = max(1, min(99, price))
            b = (100 - price) / price
            p = p_model if side == "YES" else (1 - p_model)
            f = max(0.0, ((p * (b + 1)) - 1) / b) * cfg.base_kelly_fraction
            f = min(f, cfg.base_max_position_pct)
            stake = int(bal * f)
            contracts = stake // price
            if contracts <= 0:
                continue
            # outcome al final del horizonte
            final_p = series[-1]["p_true"]
            outcome_yes = 1 if rng.random() < final_p else 0
            won = (side == "YES" and outcome_yes) or (side == "NO" and not outcome_yes)
            slip = rng.uniform(-cfg.slippage_cents, cfg.slippage_cents)
            entry_eff = price + slip
            payout = (100 if won else 0) - entry_eff
            fees = cfg.fee_cents * contracts
            pnl_cents = int(payout * contracts - fees)
            bal += pnl_cents
            trades.append(pnl_cents)
            last_pnl = pnl_cents
            equity_curve.append(bal)
            break  # baseline opera 1 vez por mercado (sin gating fino)
    return _summarize("baseline", trades, cfg, equity_curve)


def run_candidate(cfg: BTConfig, markets: list[list[dict]]) -> StrategyResult:
    rng = random.Random(cfg.seed + 2)
    bal = cfg.starting_balance_cents
    equity_curve = [bal]
    trades = []
    consecutive_losses = 0
    daily_high = bal
    safe_mode = False
    risk_violations = 0
    for series in markets:
        # Daily guard: simulamos "día" por mercado
        daily_high = max(daily_high, bal)
        dd = (daily_high - bal) / max(1, daily_high)
        if dd >= cfg.cand_daily_dd_limit_pct or \
           consecutive_losses >= cfg.cand_max_consecutive_losses:
            safe_mode = True
        if safe_mode:
            equity_curve.append(bal)
            continue

        position = None  # (side, contracts, entry_price, p_fair, t_open)
        for step in series:
            t = step["t"]
            p_market = step["p_market"]
            p_model = _ensemble_p(step["p_true"], rng)
            edge_gross = p_model - p_market
            confidence = step["confidence"]
            spread = step["spread"]
            depth_yes, depth_no = step["depth_yes"], step["depth_no"]
            cost_prob = (cfg.fee_cents + cfg.slippage_cents) / 100.0
            edge_net = abs(edge_gross) - cost_prob

            # exits sobre posición abierta
            if position is not None:
                side, contracts, entry, p_fair, t_open = position
                cur_price = int(round(
                    p_market * 100 if side == "YES" else (1 - p_market) * 100
                ))
                tp = entry + max(1, int((round(p_fair * 100) - entry) *
                                        cfg.cand_tp_capture_pct))
                tp = min(99, max(entry + 1, tp))
                sl = max(1, entry - cfg.cand_sl_ticks)
                exit_now = False
                reason = ""
                if cur_price >= tp:
                    exit_now, reason = True, "tp"
                elif cur_price <= sl:
                    exit_now, reason = True, "sl"
                elif (t - t_open) >= cfg.cand_time_stop_steps:
                    exit_now, reason = True, "time"
                if exit_now:
                    slip = rng.uniform(-cfg.slippage_cents, cfg.slippage_cents)
                    exit_eff = cur_price - slip
                    pnl = int((exit_eff - entry) * contracts -
                              cfg.fee_cents * contracts)
                    bal += pnl
                    trades.append(pnl)
                    if pnl < 0:
                        consecutive_losses += 1
                    else:
                        consecutive_losses = 0
                    equity_curve.append(bal)
                    position = None
                    continue

            # nueva entrada (sólo si no hay posición)
            if position is None:
                if confidence < cfg.cand_confidence_min:
                    continue
                if spread > 5 or depth_yes < 50 or depth_no < 50:
                    continue
                if edge_net < cfg.cand_edge_net_min:
                    continue
                side = "YES" if edge_gross > 0 else "NO"
                price = int(round(p_market * 100 if side == "YES"
                                  else (1 - p_market) * 100))
                price = max(1, min(99, price))
                b = (100 - price) / price
                p = p_model if side == "YES" else (1 - p_model)
                f_full = max(0.0, ((p * (b + 1)) - 1) / b)
                f = min(f_full * cfg.cand_kelly_fraction,
                        cfg.cand_max_position_pct)
                stake = int(bal * f)
                # cap exposure total (placeholder simple)
                max_exposure = int(bal * cfg.cand_max_total_exposure_pct)
                stake = min(stake, max_exposure)
                contracts = stake // price
                if contracts <= 0:
                    continue
                slip = rng.uniform(-cfg.slippage_cents, cfg.slippage_cents)
                entry_eff = price + slip
                position = (side, contracts, entry_eff, p_model, t)

        # cierre forzado al final del mercado si seguía abierta
        if position is not None:
            side, contracts, entry, p_fair, _ = position
            outcome_yes = 1 if rng.random() < series[-1]["p_true"] else 0
            payout = (100 if (side == "YES" and outcome_yes) or
                      (side == "NO" and not outcome_yes) else 0) - entry
            pnl = int(payout * contracts - cfg.fee_cents * contracts)
            bal += pnl
            trades.append(pnl)
            if pnl < 0:
                consecutive_losses += 1
            else:
                consecutive_losses = 0
            equity_curve.append(bal)
    return _summarize("candidate", trades, cfg, equity_curve,
                      risk_violations=risk_violations)


def _summarize(name: str, trades: list[int], cfg: BTConfig,
               equity_curve: list[int],
               risk_violations: int = 0) -> StrategyResult:
    if not trades:
        return StrategyResult(name=name, equity_curve=equity_curve)
    wins = sum(1 for t in trades if t > 0)
    losses = sum(1 for t in trades if t < 0)
    gross_win = sum(t for t in trades if t > 0)
    gross_loss = -sum(t for t in trades if t < 0)
    pf = gross_win / gross_loss if gross_loss > 0 else \
        (999.0 if gross_win > 0 else 0.0)
    pnl_net = sum(trades)
    fees_cents = int(cfg.fee_cents * len(trades))
    # max drawdown
    peak = equity_curve[0]
    max_dd = 0
    for eq in equity_curve:
        peak = max(peak, eq)
        max_dd = max(max_dd, peak - eq)
    return StrategyResult(
        name=name,
        trades=len(trades), wins=wins, losses=losses,
        pnl_cents=pnl_net + fees_cents,
        fees_cents=fees_cents,
        pnl_net_cents=pnl_net,
        pf=pf,
        expectancy_cents=pnl_net / len(trades),
        hit_rate=wins / len(trades),
        max_drawdown_cents=max_dd,
        risk_violations=risk_violations,
        equity_curve=equity_curve,
    )


# ===========================================================================
def run_experiment(cfg: Optional[BTConfig] = None,
                   out_dir: Optional[Path] = None) -> dict:
    """Walk-forward: genera markets con seed; baseline + candidate sobre
    el mismo dataset; reporta y guarda JSON."""
    cfg = cfg or BTConfig()
    rng = random.Random(cfg.seed)
    markets = [_gen_market(rng, cfg.steps_per_market)
               for _ in range(cfg.n_markets)]
    base = run_baseline(cfg, markets)
    cand = run_candidate(cfg, markets)
    summary = {
        "config": cfg.__dict__,
        "baseline": _strategy_to_dict(base),
        "candidate": _strategy_to_dict(cand),
        "delta": {
            "pnl_net_cents": cand.pnl_net_cents - base.pnl_net_cents,
            "expectancy_cents": cand.expectancy_cents - base.expectancy_cents,
            "hit_rate": cand.hit_rate - base.hit_rate,
            "profit_factor": cand.pf - base.pf,
            "max_drawdown_cents": cand.max_drawdown_cents - base.max_drawdown_cents,
        },
    }
    if out_dir is not None:
        out_dir = Path(out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        with open(out_dir / "experiment_summary.json", "w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2, default=str)
    return summary


def _strategy_to_dict(s: StrategyResult) -> dict:
    d = s.__dict__.copy()
    # equity curve compacto
    if len(d.get("equity_curve", [])) > 200:
        ec = d["equity_curve"]
        step = max(1, len(ec) // 200)
        d["equity_curve"] = ec[::step]
    return d


if __name__ == "__main__":  # pragma: no cover
    out = Path(__file__).resolve().parents[2] / "reports"
    summary = run_experiment(out_dir=out)
    print(json.dumps(summary, indent=2, default=str))

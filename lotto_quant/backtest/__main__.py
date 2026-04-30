"""
lotto_quant.backtest CLI
========================

Replays historical scratch-off snapshots in paper mode and prints a
summary of the resulting equity curve, drawdown, and per-game P&L.

Examples
--------
Run a backtest against the production DB, capping replay to the last
30 days:

    python -m lotto_quant.backtest --days-back 30 --bankroll 2000

Write the full result to JSON for downstream tooling:

    python -m lotto_quant.backtest --days-back 60 \
        --output-json /tmp/backtest.json

Use a custom source DB (read-only) and an ephemeral workspace DB so
the production database isn't polluted:

    python -m lotto_quant.backtest \
        --source-db data/lotto_quant.duckdb \
        --workspace-db /tmp/lotto_bt.sqlite

Atlas constraint reminder
-------------------------
The CLI runs only paper-mode replays. It never touches LIVE orders.
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import sys
import tempfile
from pathlib import Path
from typing import Any, Dict

from .engine import BacktestConfig, BacktestEngine, BacktestResult


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="python -m lotto_quant.backtest",
        description="Replay historical NCEL scratch-off snapshots in paper mode.",
    )
    p.add_argument("--source-db", default=None,
                   help="Path to the source DB (defaults to ATLAS_LOTTO_DB).")
    p.add_argument("--workspace-db", default=None,
                   help="Ephemeral DB for simulated orders/fills. "
                        "Default: a temp SQLite file.")
    p.add_argument("--bankroll", type=float, default=1_000.0,
                   help="Starting bankroll in USD (default: 1000).")
    p.add_argument("--kelly-fraction", type=float, default=0.25,
                   help="Fractional Kelly multiplier (default: 0.25).")
    p.add_argument("--max-position-pct", type=float, default=0.05,
                   help="Cap per-game position as fraction of bankroll (default: 0.05).")
    p.add_argument("--max-daily-exposure", type=float, default=0.20,
                   help="Cap daily total exposure as fraction of bankroll (default: 0.20).")
    p.add_argument("--min-ev-per-dollar", type=float, default=0.0,
                   help="Skip games with EV/$ below this (default: 0.0).")
    p.add_argument("--min-tickets", type=int, default=1,
                   help="Minimum order size in tickets (default: 1).")
    p.add_argument("--rng-seed", type=int, default=1234,
                   help="RNG seed for deterministic Monte-Carlo fills (default: 1234).")
    p.add_argument("--days-back", type=int, default=None,
                   help="Replay only the last N calendar days. Default: all available.")
    p.add_argument("--output-json", default=None,
                   help="If provided, write the full result JSON to this path.")
    p.add_argument("--quiet", action="store_true", help="Suppress per-day log output.")
    return p


def _summary_lines(res: BacktestResult) -> list[str]:
    cfg = res.config
    lines = [
        "─" * 60,
        "Atlas Lotto-Quant — Backtest Summary",
        "─" * 60,
        f"Days replayed       : {len(res.days)}",
        f"Bankroll start      : ${cfg.bankroll_start:,.2f}",
        f"Bankroll final      : ${res.final_bankroll:,.2f}",
        f"Realized PnL        : ${res.realized_pnl:,.2f}",
        f"Total cost          : ${res.total_cost:,.2f}",
        f"Total net payout    : ${res.total_net_payout:,.2f}",
        f"Orders / fills      : {res.n_orders} / {res.n_fills}",
        f"Max drawdown        : {res.max_drawdown * 100:.2f}%",
        f"Sharpe-like (×√252) : {res.sharpe_like:.3f}",
        f"Daily win-rate      : {res.win_rate * 100:.1f}%",
        f"Kelly fraction      : {cfg.kelly_fraction}",
        f"Max position pct    : {cfg.max_position_pct}",
        f"Max daily exposure  : {cfg.max_daily_exposure}",
        f"RNG seed            : {cfg.rng_seed}",
    ]
    if res.notes:
        lines.append("")
        lines.append("Notes:")
        for n in res.notes:
            lines.append(f"  • {n}")
    if res.per_game_pnl:
        top = sorted(res.per_game_pnl.items(), key=lambda kv: kv[1], reverse=True)
        lines.append("")
        lines.append("Per-game PnL (top 10):")
        for gid, pnl in top[:10]:
            lines.append(f"  {gid:<24s} ${pnl:>+10.2f}")
    return lines


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    logging.basicConfig(
        level=logging.WARNING if args.quiet else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s — %(message)s",
    )

    workspace_db = args.workspace_db
    cleanup_path: Path | None = None
    if workspace_db is None:
        # Default: temp file so we never write into the production DB
        fd, tmp_path = tempfile.mkstemp(prefix="lotto_bt_", suffix=".sqlite")
        os.close(fd)
        workspace_db = tmp_path
        cleanup_path = Path(tmp_path)

    cfg = BacktestConfig(
        source_db=args.source_db,
        workspace_db=workspace_db,
        bankroll_start=args.bankroll,
        kelly_fraction=args.kelly_fraction,
        max_position_pct=args.max_position_pct,
        max_daily_exposure=args.max_daily_exposure,
        min_ev_per_dollar=args.min_ev_per_dollar,
        min_tickets=args.min_tickets,
        rng_seed=args.rng_seed,
        days_back=args.days_back,
    )
    engine = BacktestEngine(cfg)
    result = engine.run()

    for line in _summary_lines(result):
        print(line)

    if args.output_json:
        out: Dict[str, Any] = {
            "summary": result.summary_dict(),
            "equity_curve": [
                {"day": day, "bankroll": br} for day, br in result.equity_curve
            ],
            "days": [d.to_dict() for d in result.days],
        }
        Path(args.output_json).parent.mkdir(parents=True, exist_ok=True)
        Path(args.output_json).write_text(json.dumps(out, indent=2, default=str))
        print(f"\nWrote {args.output_json}")

    if cleanup_path is not None:
        try:
            cleanup_path.unlink(missing_ok=True)
        except Exception:
            pass

    return 0


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())

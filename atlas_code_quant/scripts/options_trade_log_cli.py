"""CLI paper-only: registra entrada o cierre en OptionsPaperJournal (sin broker ni red)."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
from typing import Any

from atlas_code_quant.options import OptionsPaperJournal


def _utc_stamp_short() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Registra ejecuciones paper (entrada/cierre) en el journal JSONL por trace_id.",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    common_journal = argparse.ArgumentParser(add_help=False)
    common_journal.add_argument(
        "--journal-path",
        dest="journal_path",
        default=None,
        metavar="PATH",
        help="Ruta JSONL del journal; si se omite, usa el default de OptionsPaperJournal.",
    )

    entry_p = sub.add_parser(
        "entry",
        parents=[common_journal],
        help="Registrar ejecución de entrada paper.",
    )
    entry_p.add_argument("--trace-id", dest="trace_id", required=True, help="trace_id del session_plan.")
    entry_p.add_argument("--symbol", required=True, help="Subyacente, p. ej. SPX.")
    entry_p.add_argument("--structure", required=True, help="Estructura, p. ej. iron_condor.")
    entry_p.add_argument("--short-put", dest="short_put", type=float, default=None)
    entry_p.add_argument("--long-put", dest="long_put", type=float, default=None)
    entry_p.add_argument("--short-call", dest="short_call", type=float, default=None)
    entry_p.add_argument("--long-call", dest="long_call", type=float, default=None)
    entry_p.add_argument("--credit", type=float, default=None)
    entry_p.add_argument("--dte", type=int, default=None)
    entry_p.add_argument("--notes", action="append", default=None, help="Nota (repetible).")

    close_p = sub.add_parser(
        "close",
        parents=[common_journal],
        help="Registrar ejecución de cierre paper.",
    )
    close_p.add_argument("--trace-id", dest="trace_id", required=True)
    close_p.add_argument("--symbol", required=True)
    close_p.add_argument("--debit", type=float, default=None)
    close_p.add_argument("--exit-mid", dest="exit_mid", type=float, default=None)
    close_p.add_argument("--pnl", type=float, default=None, help="P&L realizado en USD.")
    close_p.add_argument("--notes", action="append", default=None, help="Nota (repetible).")

    return parser.parse_args(argv)


def run(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    journal = OptionsPaperJournal(path=args.journal_path) if args.journal_path else OptionsPaperJournal()
    now = datetime.now(timezone.utc)
    ts = _utc_stamp_short()

    if args.command == "entry":
        executed_entry: dict[str, Any] = {
            "structure": args.structure,
            "short_put": args.short_put,
            "long_put": args.long_put,
            "short_call": args.short_call,
            "long_call": args.long_call,
            "credit": args.credit,
            "dte": args.dte,
        }
        notes = args.notes if args.notes else ["paper_entry_cli"]
        journal.log_entry_execution(
            trace_id=args.trace_id,
            symbol=str(args.symbol).strip().upper(),
            entry_timestamp=now,
            planned_entry=None,
            executed_entry=executed_entry,
            notes=notes,
        )
        cr = executed_entry["credit"]
        dte = executed_entry["dte"]
        cr_s = f"{cr:.2f}" if isinstance(cr, (int, float)) else "-"
        dte_s = str(dte) if dte is not None else "-"
        print(
            f"[{ts}] ENTRY LOGGED — trace_id={args.trace_id} symbol={args.symbol.strip().upper()} "
            f"structure={args.structure} credit={cr_s} dte={dte_s}",
        )
        return 0

    if args.command == "close":
        executed_close: dict[str, Any] = {
            "debit": args.debit,
            "exit_mid": args.exit_mid,
        }
        notes = args.notes if args.notes else ["paper_close_cli"]
        journal.log_close_execution(
            trace_id=args.trace_id,
            symbol=str(args.symbol).strip().upper(),
            close_timestamp=now,
            executed_close=executed_close,
            pnl_realized=args.pnl,
            notes=notes,
        )
        pnl = args.pnl
        pnl_s = f"{pnl:.2f}" if isinstance(pnl, (int, float)) else "-"
        print(
            f"[{ts}] CLOSE LOGGED — trace_id={args.trace_id} symbol={args.symbol.strip().upper()} "
            f"pnl_realized={pnl_s}",
        )
        return 0

    raise SystemExit(f"Unknown command: {args.command}")


if __name__ == "__main__":
    raise SystemExit(run())

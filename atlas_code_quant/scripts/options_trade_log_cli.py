"""CLI paper-only: registra entrada o cierre en OptionsPaperJournal (sin broker ni red)."""
from __future__ import annotations

import argparse
import sys
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
    common_journal.add_argument(
        "--mode",
        default="paper",
        choices=["paper"],
        help="Modo del evento en journal (P0: solo paper).",
    )
    common_journal.add_argument(
        "--source",
        default="manual",
        metavar="SRC",
        help="Origen del evento (manual, planner, autoclose, etc.).",
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
    entry_p.add_argument("--debit", type=float, default=None)
    entry_p.add_argument("--entry-mid", dest="entry_mid", type=float, default=None)
    entry_p.add_argument("--max-loss", dest="max_loss", type=float, default=None)
    entry_p.add_argument("--max-profit", dest="max_profit", type=float, default=None)
    entry_p.add_argument("--dte", type=int, default=None)
    entry_p.add_argument("--qty", type=int, default=1)
    entry_p.add_argument("--expiration", default=None, help="Fecha YYYY-MM-DD opcional para piernas.")
    entry_p.add_argument("--notes", action="append", default=None, help="Nota (repetible).")

    close_p = sub.add_parser(
        "close",
        parents=[common_journal],
        help="Registrar ejecución de cierre paper.",
    )
    close_p.add_argument("--trace-id", dest="trace_id", required=True)
    close_p.add_argument("--symbol", required=True)
    close_p.add_argument(
        "--reason",
        default=None,
        metavar="TEXT",
        help="Motivo declarado del cierre (opcional). Si se omite, no se inventa razón en el payload.",
    )
    close_p.add_argument("--debit", type=float, default=None)
    close_p.add_argument("--credit", type=float, default=None)
    close_p.add_argument("--exit-mid", dest="exit_mid", type=float, default=None)
    close_p.add_argument(
        "--close-type",
        dest="close_type",
        default="full",
        metavar="TYPE",
        help="Tipo de cierre (full, partial, roll, etc.).",
    )
    close_p.add_argument("--pnl", type=float, default=None, help="P&L realizado en USD.")
    close_p.add_argument("--pnl-pct", dest="pnl_pct", type=float, default=None, help="P&L porcentual del cierre.")
    close_p.add_argument("--notes", action="append", default=None, help="Nota (repetible).")

    return parser.parse_args(argv)


def _close_decision_payload_from_cli(args: argparse.Namespace) -> dict[str, Any]:
    """Payload mínimo para ``log_close_decision`` (solo datos CLI reales; sin juicio de calidad)."""
    sym = str(args.symbol).strip().upper()
    out: dict[str, Any] = {
        "symbol": sym,
        "decision": "execute_close",
        "source": "options_trade_log_cli.close",
    }
    reason = args.reason
    if reason is not None and str(reason).strip():
        out["reason"] = str(reason).strip()
    if args.close_type is not None and str(args.close_type).strip():
        out["close_type"] = str(args.close_type).strip()
    ctx: dict[str, Any] = {}
    if args.debit is not None:
        ctx["debit"] = float(args.debit)
    if args.exit_mid is not None:
        ctx["exit_mid"] = float(args.exit_mid)
    if args.pnl is not None:
        ctx["pnl_realized_declared"] = float(args.pnl)
    if ctx:
        out["cli_context"] = ctx
    return out


def _validate_close_identifiers(args: argparse.Namespace) -> str | None:
    """Devuelve mensaje de error si ``trace_id`` o ``symbol`` no son logueables de forma segura."""
    tid = str(args.trace_id).strip()
    if not tid:
        return "trace_id vacío o solo espacios; no se registra cierre."
    sym = str(args.symbol).strip().upper()
    if not sym:
        return "symbol vacío o solo espacios; no se registra cierre."
    return None


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
            "debit": args.debit,
            "entry_mid": args.entry_mid,
            "max_loss": args.max_loss,
            "max_profit": args.max_profit,
            "dte": args.dte,
            "qty": args.qty,
            "expiration": args.expiration,
        }
        notes = args.notes if args.notes else ["paper_entry_cli"]
        journal.log_entry_execution(
            trace_id=args.trace_id,
            symbol=str(args.symbol).strip().upper(),
            entry_timestamp=now,
            planned_entry=None,
            executed_entry=executed_entry,
            mode=args.mode,
            source=args.source,
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
        err = _validate_close_identifiers(args)
        if err:
            print(f"[{ts}] ERROR — {err}", file=sys.stderr)
            return 2
        tid = str(args.trace_id).strip()
        sym = str(args.symbol).strip().upper()
        close_decision = _close_decision_payload_from_cli(args)
        notes = args.notes if args.notes else ["paper_close_cli"]
        journal.log_close_decision(
            trace_id=tid,
            close_decision=close_decision,
            timestamp=now,
            mode=args.mode,
            source=args.source,
            autoclose_applied=args.source.strip().lower() == "autoclose",
            notes=list(notes),
        )
        executed_close: dict[str, Any] = {
            "debit": args.debit,
            "credit": args.credit,
            "exit_mid": args.exit_mid,
            "close_type": args.close_type,
            "close_reason": args.reason,
            "pnl_pct": args.pnl_pct,
        }
        journal.log_close_execution(
            trace_id=tid,
            symbol=sym,
            close_timestamp=now,
            executed_close=executed_close,
            pnl_realized=args.pnl,
            mode=args.mode,
            source=args.source,
            close_type=args.close_type,
            close_reason=args.reason,
            pnl_pct=args.pnl_pct,
            autoclose_applied=args.source.strip().lower() == "autoclose",
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

"""CLI mínimo: plan de sesión opciones paper-only (briefing → intent → entrada). Sin broker live."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
from typing import Any

from atlas_code_quant.options import (
    OptionsIntentRouter,
    OptionsPaperJournal,
    PaperEntryPlanner,
    PaperSessionOrchestrator,
    SessionBriefingEngine,
)


def _default_briefing_engine() -> SessionBriefingEngine:
    """Misma composición que el endpoint paper-session-plan (Tradier paper + IV rank)."""
    from atlas_code_quant.backtesting.winning_probability import TradierClient
    from atlas_code_quant.execution.option_chain_cache import OptionChainCache
    from atlas_code_quant.options.iv_rank_calculator import IVRankCalculator

    client = TradierClient(scope="paper")
    calc = IVRankCalculator(client, chain_cache=OptionChainCache(), scope="paper")
    return SessionBriefingEngine(calc)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Genera un plan de sesión de opciones (paper-only): briefing, intent y plan de entrada.",
    )
    p.add_argument("--symbol", required=True, help="Subyacente, p. ej. SPX, SPY.")
    p.add_argument("--capital", type=float, required=True, help="Capital de referencia en USD para sizing.")
    p.add_argument(
        "--direction",
        default=None,
        choices=("bullish", "bearish", "neutral", "volatile"),
        help="Sesgo direccional (opcional).",
    )
    p.add_argument(
        "--regime",
        default=None,
        choices=("trending", "ranging", "volatile", "unclear"),
        help="Régimen de mercado (opcional).",
    )
    p.add_argument(
        "--gamma-regime",
        dest="gamma_regime",
        default=None,
        choices=("long_gamma", "short_gamma", "neutral", "unknown"),
        help="Régimen gamma (opcional).",
    )
    p.add_argument(
        "--dte-mode",
        dest="dte_mode",
        default=None,
        metavar="MODE",
        help="Modo DTE, p. ej. 0dte, 1to7, 8to21, 22plus (opcional).",
    )
    p.add_argument(
        "--journal-path",
        dest="journal_path",
        default=None,
        metavar="PATH",
        help="Ruta JSONL del journal; si se omite, usa el default de OptionsPaperJournal.",
    )
    return p.parse_args(argv)


def format_header(symbol: str, capital: float) -> str:
    now = datetime.now(timezone.utc).isoformat()
    return f"[{now}] OPTIONS SESSION PLAN — {symbol} (capital ref: {capital:,.2f} USD)"


def _fmt_bool(v: Any) -> str:
    if isinstance(v, bool):
        return str(v)
    return repr(v)


def _print_human_plan(plan: dict[str, Any], *, symbol: str, capital: float) -> None:
    briefing = plan.get("briefing") if isinstance(plan.get("briefing"), dict) else {}
    intent = plan.get("intent") if isinstance(plan.get("intent"), dict) else {}
    entry_plan = plan.get("entry_plan") if isinstance(plan.get("entry_plan"), dict) else {}

    trace_id = plan.get("trace_id", "")
    cands = intent.get("strategy_candidates") or []
    cand_str = ", ".join(str(x) for x in cands) if cands else "(ninguno)"

    notes = plan.get("pipeline_notes") or []
    flags = plan.get("pipeline_quality_flags") or []
    reason = entry_plan.get("reason")

    print(format_header(symbol, capital))
    print("=" * 80)
    print(f"trace_id           : {trace_id or '(no journal / sin id)'}")
    print(f"automation_mode    : {plan.get('automation_mode', '')}")
    print()
    print("Contexto de mercado (briefing):")
    print(f"  direction        : {briefing.get('direction', '')}")
    print(f"  regime           : {briefing.get('regime', '')}")
    print(f"  gamma_regime     : {briefing.get('gamma_regime', '')}")
    print(f"  dte_mode         : {briefing.get('dte_mode', '')}")
    print()
    print("Intent de opciones:")
    print(f"  allow_entry      : {_fmt_bool(intent.get('allow_entry'))}")
    print(f"  force_no_trade   : {_fmt_bool(intent.get('force_no_trade'))}")
    print(f"  recommended_strat: {intent.get('recommended_strategy')}")
    print(f"  candidates       : {cand_str}")
    print()
    print("Plan de entrada (paper):")
    print(f"  entry_allowed    : {_fmt_bool(plan.get('entry_allowed'))}")
    print(f"  entry_state      : {entry_plan.get('entry', '')}")
    print(f"  reason           : {reason if reason is not None else 'None'}")
    print(f"  strategy         : {entry_plan.get('recommended_strategy')}")
    print(f"  max_risk_pct     : {entry_plan.get('max_risk_budget_pct')}")
    print(f"  max_risk_dollars : {entry_plan.get('max_risk_budget_dollars')}")
    print()
    if flags:
        print("QUALITY FLAGS:")
        for f in flags:
            print(f"  - {f}")
        print()
    print("NOTES:")
    if notes:
        for n in notes:
            print(f"  - {n}")
    else:
        print("  (ninguna)")
    print()
    print("DECISIÓN PAPER:")
    if plan.get("entry_allowed"):
        print("  GO    → Plan apto para entrada paper, sujeto a tu revisión final.")
    else:
        print(
            "  NO-GO → El plan actual NO recomienda entrada automática (paper). Revisa motivo arriba.",
        )


def run(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    symbol = str(args.symbol).strip().upper()
    capital = float(args.capital)

    briefing_engine = _default_briefing_engine()
    intent_router = OptionsIntentRouter()
    entry_planner = PaperEntryPlanner()
    journal = OptionsPaperJournal(path=args.journal_path) if args.journal_path else OptionsPaperJournal()
    orchestrator = PaperSessionOrchestrator(
        briefing_engine=briefing_engine,
        intent_router=intent_router,
        entry_planner=entry_planner,
        journal=journal,
    )

    plan = orchestrator.build_session_plan(
        symbol=symbol,
        direction=args.direction,
        regime=args.regime,
        gamma_regime=args.gamma_regime,
        dte_mode=args.dte_mode,
        capital=capital,
    )

    _print_human_plan(plan, symbol=symbol, capital=capital)
    return 0


if __name__ == "__main__":
    raise SystemExit(run())

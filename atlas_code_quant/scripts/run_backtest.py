"""CLI para ejecutar AtlasBacktester (validacion Fase 2)."""
from __future__ import annotations

import argparse
import json
import logging
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
QUANT_ROOT = ROOT / "atlas_code_quant"
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from atlas_code_quant.backtester import AtlasBacktester  # noqa: E402

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("quant.backtest.cli")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="ATLAS Backtester - validacion Fase 2")
    p.add_argument("--start-date", required=True, help="Inicio YYYY-MM-DD")
    p.add_argument("--end-date", required=True, help="Fin YYYY-MM-DD")
    p.add_argument("--symbols", default="AAPL,MSFT,GOOGL,TSLA,NVDA", help="Lista separada por coma")
    p.add_argument("--timeframe", default="1h", help="Temporalidad (1h,4h,1d)")
    p.add_argument("--capital", type=float, default=10_000.0)
    p.add_argument("--risk-per-trade", type=float, default=0.02)
    p.add_argument("--max-trades-per-cycle", type=int, default=5)
    p.add_argument("--disable-search", action="store_true")
    p.add_argument("--report-path", default="", help="Ruta HTML de salida")
    p.add_argument("--json-out", default="", help="Ruta JSON resumen")
    return p.parse_args()


def _print_metrics(metrics: dict) -> None:
    logger.info(
        "[METRICS] Win rate: %.2f%% | Sharpe: %.2f | Max DD: %.2f%% | PF: %.2f | Trades: %d",
        float(metrics.get("win_rate_pct", 0.0)),
        float(metrics.get("sharpe_ratio", 0.0)),
        float(metrics.get("max_drawdown_pct", 0.0)),
        float(metrics.get("profit_factor", 0.0)),
        int(metrics.get("total_trades", 0)),
    )


def main() -> int:
    args = parse_args()
    symbols = [s.strip() for s in args.symbols.split(",") if s.strip()]
    bt = AtlasBacktester(
        start_date=args.start_date,
        end_date=args.end_date,
        initial_capital=args.capital,
        max_trades_per_cycle=args.max_trades_per_cycle,
        risk_per_trade=args.risk_per_trade,
        search_mode=not args.disable_search,
        timeframe=args.timeframe,
    )

    logger.info("Running backtest for %s symbols in %s", len(symbols), args.timeframe)
    result = bt.run(symbols=symbols, timeframe=args.timeframe)
    metrics = result["metrics"]
    _print_metrics(metrics)
    decision = result["decision"]
    if decision["go_live_approved"]:
        logger.info("[DECISION] GO-LIVE PHASE 2 APPROVED")
    else:
        logger.warning("[DECISION] PARAMETER SEARCH REQUIRED - reasons: %s", ", ".join(decision["reasons"]))
    html_out = bt.generate_backtest_report(output_path=args.report_path or None)
    logger.info("[REPORT] Generated: %s", html_out)

    if args.json_out:
        payload = {"result": result, "report_html": html_out}
        Path(args.json_out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.json_out).write_text(json.dumps(payload, indent=2, default=str), encoding="utf-8")
        logger.info("JSON summary: %s", args.json_out)
    logger.info("[OK] Backtest completed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""CLI para Iron Butterfly SPX 0DTE completo."""
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

from atlas_code_quant.iron_butterfly.iron_butterfly_complete import (  # noqa: E402
    DEFAULT_CONFIG,
    IronButterflyBacktester,
)

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("quant.iron_butterfly.cli")


def _args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="ATLAS Iron Butterfly SPX 0DTE")
    p.add_argument("--start-date", required=True)
    p.add_argument("--end-date", required=True)
    p.add_argument("--capital", type=float, default=10_000.0)
    p.add_argument("--magnet-threshold", type=float, default=float(DEFAULT_CONFIG["magnet_threshold"]))
    p.add_argument("--output-dir", default=str((QUANT_ROOT / "data" / "backtest" / "backtest_results" / "iron_butterfly")))
    p.add_argument("--html-report", action="store_true")
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--debug", action="store_true")
    p.add_argument("--mode", default="backtest", choices=["backtest", "live"])
    return p.parse_args()


def main() -> int:
    args = _args()
    cfg = {"magnet_threshold": args.magnet_threshold}
    bt = IronButterflyBacktester(
        start_date=args.start_date,
        end_date=args.end_date,
        capital=args.capital,
        config=cfg,
        output_dir=args.output_dir,
    )
    logger.info("Backtest completo iniciado")
    if args.dry_run:
        logger.info("Dry-run: validacion basica de carga y detector")
        bt.load_spx_data(interval="1h")
        if bt.spx_data.empty:
            logger.error("No se cargaron datos SPX")
            return 1
        logger.info("Dry-run OK. Bars: %d", len(bt.spx_data))
        return 0

    if args.debug:
        bt.debug_backtest()
        return 0

    result = bt.run(html_report=args.html_report, mode=args.mode, debug=args.debug)
    metrics = result["metrics"]
    logger.info(
        "Trades=%d | WinRate=%.2f%% | MagnetAcc=%.2f%% | ROI=%.2f%% | Sharpe=%.2f | MaxDD=%.2f%%",
        int(metrics.get("total_trades", 0)),
        float(metrics.get("win_rate_pct", 0.0)),
        float(metrics.get("magnet_accuracy_pct", 0.0)),
        float(metrics.get("roi_pct", 0.0)),
        float(metrics.get("sharpe_ratio", 0.0)),
        float(metrics.get("max_drawdown_pct", 0.0)),
    )
    logger.info("Archivos generados:")
    for _, path in result["files"].items():
        if path:
            logger.info(" - %s", path)
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

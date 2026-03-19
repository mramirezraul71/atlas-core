"""Atlas Code-Quant — CLI de backtesting.

Uso:
    python scripts/run_backtest.py --symbol BTC/USDT --timeframe 1h --limit 500
    python scripts/run_backtest.py --symbol AAPL --source yfinance --period 1y
    python scripts/run_backtest.py --symbol BTC/USDT --strategy ml_rf --model-name rf

Genera informe HTML en reports/ y exporta JSON.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("quant.cli")


def parse_args():
    p = argparse.ArgumentParser(description="Atlas Code-Quant — Backtesting CLI")
    p.add_argument("--symbol",     default="BTC/USDT",    help="Par/ticker (ej: BTC/USDT, AAPL)")
    p.add_argument("--source",     default="ccxt",        choices=["ccxt", "yfinance"])
    p.add_argument("--exchange",   default="binance",     help="Exchange ccxt (ej: binance)")
    p.add_argument("--timeframe",  default="1h",          help="Timeframe (ej: 1h, 4h, 1d)")
    p.add_argument("--limit",      default=500,  type=int, help="Número de velas")
    p.add_argument("--period",     default="1y",           help="Período yfinance (ej: 1y, 2y)")
    p.add_argument("--strategy",   default="ma_cross",    choices=["ma_cross", "ml_rf", "ml_gb"])
    p.add_argument("--fast",       default=10,   type=int, help="MA rápida (ma_cross)")
    p.add_argument("--slow",       default=50,   type=int, help="MA lenta (ma_cross)")
    p.add_argument("--capital",    default=10_000, type=float, help="Capital inicial")
    p.add_argument("--commission", default=0.001, type=float, help="Comisión (0.001 = 0.1%)")
    p.add_argument("--no-report",  action="store_true",   help="Omitir HTML report")
    p.add_argument("--model-name", default="rf",          help="Nombre modelo ML")
    return p.parse_args()


def main():
    args = parse_args()
    reports_dir = Path(__file__).resolve().parent.parent / "reports"
    reports_dir.mkdir(parents=True, exist_ok=True)

    # ── Cargar datos ─────────────────────────────────────────────────────────
    logger.info("Descargando datos: %s [%s] — %d velas", args.symbol, args.timeframe, args.limit)
    from data.feed import MarketFeed
    feed = MarketFeed(exchange_id=args.exchange)

    if args.source == "ccxt":
        df = feed.ohlcv_ccxt(args.symbol, args.timeframe, limit=args.limit)
    else:
        df = feed.ohlcv_yfinance(args.symbol, period=args.period, interval=args.timeframe)

    if df is None or df.empty:
        logger.error("No se pudo obtener datos para %s", args.symbol)
        sys.exit(1)

    logger.info("Datos obtenidos: %d velas", len(df))

    # ── Seleccionar estrategia ────────────────────────────────────────────────
    if args.strategy == "ma_cross":
        from strategies.ma_cross import MACrossStrategy
        strategy = MACrossStrategy(
            "ma_cross", [args.symbol],
            timeframe=args.timeframe,
            fast_period=args.fast,
            slow_period=args.slow,
        )
    else:
        from models.signals import MLSignalStrategy
        model_name = "rf" if args.strategy == "ml_rf" else "gb"
        strategy = MLSignalStrategy(
            f"ml_{model_name}", [args.symbol],
            model_name=model_name,
        )

    # ── Ejecutar backtest ─────────────────────────────────────────────────────
    from backtesting.engine import BacktestConfig, BacktestEngine
    config = BacktestConfig(
        initial_capital=args.capital,
        commission_pct=args.commission,
    )
    engine = BacktestEngine(strategy=strategy, config=config)

    logger.info("Ejecutando backtest…")
    result = engine.run(df, args.symbol)

    # ── Mostrar resumen ───────────────────────────────────────────────────────
    from backtesting.reporter import print_summary
    print_summary(result)

    # ── Exportar resultados ───────────────────────────────────────────────────
    safe_sym = args.symbol.replace("/", "-")
    json_path = reports_dir / f"backtest_{safe_sym}_{args.strategy}.json"
    from backtesting.reporter import export_json
    export_json(result, json_path)
    logger.info("JSON guardado: %s", json_path)

    if not args.no_report:
        html_path = reports_dir / f"backtest_{safe_sym}_{args.strategy}.html"
        try:
            from backtesting.reporter import generate_html_report
            generate_html_report(result, html_path)
            logger.info("Informe HTML: %s", html_path)
        except ImportError:
            logger.warning("plotly no instalado — omitiendo HTML report")


if __name__ == "__main__":
    main()

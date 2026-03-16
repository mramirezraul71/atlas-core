"""Atlas Code-Quant — Módulo de backtesting."""
from backtesting.engine import BacktestConfig, BacktestEngine, BacktestResult, Trade
from backtesting.metrics import compute_metrics
from backtesting.reporter import export_json, generate_html_report, print_summary

__all__ = [
    "BacktestConfig",
    "BacktestEngine",
    "BacktestResult",
    "Trade",
    "compute_metrics",
    "export_json",
    "generate_html_report",
    "print_summary",
]

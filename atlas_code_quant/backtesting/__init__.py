"""Atlas Code-Quant — Módulo de backtesting."""
from importlib import import_module

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

_EXPORTS = {
    "BacktestConfig": ("atlas_code_quant.backtesting.engine", "BacktestConfig"),
    "BacktestEngine": ("atlas_code_quant.backtesting.engine", "BacktestEngine"),
    "BacktestResult": ("atlas_code_quant.backtesting.engine", "BacktestResult"),
    "Trade": ("atlas_code_quant.backtesting.engine", "Trade"),
    "compute_metrics": ("atlas_code_quant.backtesting.metrics", "compute_metrics"),
    "export_json": ("atlas_code_quant.backtesting.reporter", "export_json"),
    "generate_html_report": ("atlas_code_quant.backtesting.reporter", "generate_html_report"),
    "print_summary": ("atlas_code_quant.backtesting.reporter", "print_summary"),
}


def __getattr__(name: str):
    target = _EXPORTS.get(name)
    if target is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module_name, attr_name = target
    module = import_module(module_name)
    return getattr(module, attr_name)

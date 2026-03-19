"""Atlas Code-Quant — Punto de entrada del motor de trading.

Uso:
    python main.py

Arranca la API REST en el puerto 8792.
"""
from __future__ import annotations

import logging
import sys
from pathlib import Path

# Asegura que el directorio raíz está en el path
sys.path.insert(0, str(Path(__file__).resolve().parent))

import uvicorn

from api.main import app, _strategies, _portfolio
from config.settings import settings
from execution.portfolio import Portfolio
from strategies.ma_cross import MACrossStrategy

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s — %(message)s",
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler(settings.logs_dir / "quant.log"),
    ],
)
logger = logging.getLogger("quant.main")


def bootstrap():
    """Inicializa el portfolio y registra las estrategias disponibles."""
    import api.main as api_mod

    # Portfolio con capital inicial
    portfolio = Portfolio(
        initial_capital=10_000.0,
        max_position_pct=settings.max_position_pct,
        max_drawdown_pct=settings.max_drawdown_pct,
    )
    api_mod._portfolio = portfolio

    # Registrar estrategias
    ma_cross = MACrossStrategy(
        name="ma_cross",
        symbols=settings.assets,
        timeframe=settings.primary_timeframe,
        fast_period=10,
        slow_period=50,
    )
    api_mod._strategies["ma_cross"] = ma_cross

    logger.info("Bootstrap completo — %d estrategias registradas", len(api_mod._strategies))
    logger.info("Portfolio: $%.2f capital inicial", portfolio.initial_capital)
    logger.info("Paper trading: %s", settings.paper_trading)


if __name__ == "__main__":
    bootstrap()
    logger.info("Iniciando Atlas Code-Quant API en puerto %d", settings.api_port)
    uvicorn.run(
        app,
        host=settings.api_host,
        port=settings.api_port,
        log_level="info",
    )

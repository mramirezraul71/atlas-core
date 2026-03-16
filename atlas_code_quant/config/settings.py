"""Atlas Code-Quant — Configuración central."""
from __future__ import annotations
import os
from dataclasses import dataclass, field
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent.parent

@dataclass
class TradingConfig:
    # Activos soportados
    assets: list[str] = field(default_factory=lambda: ["BTC/USDT", "ETH/USDT"])
    asset_type: str = "crypto"          # crypto | stocks | multi

    # Exchange / broker
    exchange: str = "binance"           # ccxt exchange id
    paper_trading: bool = True          # True = simulación, nunca real sin confirmar

    # Gestión de riesgo
    max_position_pct: float = 0.05      # 5% del capital por posición
    max_drawdown_pct: float = 0.15      # 15% drawdown máximo
    default_stop_loss_pct: float = 0.02 # 2% stop loss
    default_take_profit_pct: float = 0.04

    # Timeframes
    primary_timeframe: str = "1h"
    secondary_timeframe: str = "4h"

    # API interna (Atlas/ROS2)
    api_host: str = "0.0.0.0"
    api_port: int = 8792
    api_key: str = os.getenv("QUANT_API_KEY", "atlas-quant-local")

    # Paths
    data_dir: Path = BASE_DIR / "data" / "cache"
    models_dir: Path = BASE_DIR / "models" / "saved"
    logs_dir: Path = BASE_DIR / "logs"

    def __post_init__(self):
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self.models_dir.mkdir(parents=True, exist_ok=True)
        self.logs_dir.mkdir(parents=True, exist_ok=True)


# Singleton global
settings = TradingConfig()

"""Atlas Code-Quant central configuration."""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path


BASE_DIR = Path(__file__).resolve().parent.parent


@dataclass
class TradingConfig:
    # Supported assets
    assets: list[str] = field(default_factory=lambda: ["BTC/USDT", "ETH/USDT"])
    asset_type: str = "crypto"  # crypto | stocks | multi | options

    # Exchange / broker
    exchange: str = "binance"
    paper_trading: bool = True
    tradier_live_token: str = os.getenv("TRADIER_LIVE_TOKEN", "").strip()
    tradier_paper_token: str = os.getenv("TRADIER_PAPER_TOKEN", "").strip()
    tradier_live_account_id: str = os.getenv("TRADIER_LIVE_ACCOUNT_ID", "").strip()
    tradier_paper_account_id: str = os.getenv("TRADIER_PAPER_ACCOUNT_ID", "").strip()
    tradier_live_base_url: str = os.getenv("TRADIER_LIVE_BASE_URL", "https://api.tradier.com/v1").rstrip("/")
    tradier_paper_base_url: str = os.getenv("TRADIER_PAPER_BASE_URL", "https://sandbox.tradier.com/v1").rstrip("/")
    tradier_default_scope: str = os.getenv("TRADIER_DEFAULT_SCOPE", "").strip().lower()
    tradier_timeout_sec: int = int(os.getenv("TRADIER_TIMEOUT_SEC", "15"))

    # PDT / monitoring controls
    tradier_monitor_cache_ttl_sec: int = int(os.getenv("TRADIER_MONITOR_CACHE_TTL_SEC", "300"))
    tradier_probability_refresh_sec: int = int(os.getenv("TRADIER_PROBABILITY_REFRESH_SEC", "300"))
    tradier_pdt_min_equity: float = float(os.getenv("TRADIER_PDT_MIN_EQUITY", "25000"))
    tradier_pdt_max_day_trades: int = int(os.getenv("TRADIER_PDT_MAX_DAY_TRADES", "3"))
    tradier_pdt_window_days: int = int(os.getenv("TRADIER_PDT_WINDOW_DAYS", "5"))
    tradier_pdt_fail_closed: bool = os.getenv("TRADIER_PDT_FAIL_CLOSED", "true").strip().lower() not in {"0", "false", "no"}

    # Risk management
    max_position_pct: float = 0.05
    max_drawdown_pct: float = 0.15
    default_stop_loss_pct: float = 0.02
    default_take_profit_pct: float = 0.04

    # Timeframes
    primary_timeframe: str = "1h"
    secondary_timeframe: str = "4h"

    # Internal API (Atlas/ROS2)
    api_host: str = "0.0.0.0"
    api_port: int = 8792
    api_key: str = os.getenv("QUANT_API_KEY", "atlas-quant-local")

    # Paths
    data_dir: Path = BASE_DIR / "data" / "cache"
    models_dir: Path = BASE_DIR / "models" / "saved"
    logs_dir: Path = BASE_DIR / "logs"

    def __post_init__(self) -> None:
        if self.tradier_default_scope not in {"live", "paper"}:
            self.tradier_default_scope = "paper" if self.paper_trading else "live"
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self.models_dir.mkdir(parents=True, exist_ok=True)
        self.logs_dir.mkdir(parents=True, exist_ok=True)


settings = TradingConfig()

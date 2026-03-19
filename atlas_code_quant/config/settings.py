"""Atlas Code-Quant central configuration."""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path


BASE_DIR = Path(__file__).resolve().parent.parent
DEFAULT_TRADIER_CREDENTIALS_FILE = Path(r"C:\dev\credenciales.txt")


def _clean_setting(value: str | None, default: str = "") -> str:
    return (value or default).strip()


def _default_tradier_credentials_file() -> str:
    explicit = _clean_setting(os.getenv("TRADIER_CREDENTIALS_FILE"))
    if explicit:
        return explicit
    if DEFAULT_TRADIER_CREDENTIALS_FILE.exists():
        return str(DEFAULT_TRADIER_CREDENTIALS_FILE)
    return ""


def _load_tradier_file_credentials(path_value: str | None) -> dict[str, dict[str, str]]:
    path = Path(_clean_setting(path_value))
    if not path or not path.exists() or not path.is_file():
        return {"live": {}, "paper": {}}

    section_map = {
        "PRODUCTION_ACCOUNT": "live",
        "PAPER_ACCOUNT": "paper",
    }
    credentials: dict[str, dict[str, str]] = {"live": {}, "paper": {}}
    current_scope: str | None = None

    for raw_line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("[") and line.endswith("]"):
            current_scope = section_map.get(line[1:-1].strip().upper())
            continue
        if current_scope is None or "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip().upper()
        value = value.strip()
        if key in {"ACCOUNT_ID", "ACCESS_TOKEN", "BASE_URL"} and value:
            credentials[current_scope][key] = value.rstrip("/") if key == "BASE_URL" else value

    return credentials


_TRADIER_CREDENTIALS_FILE = _default_tradier_credentials_file()
_TRADIER_FILE_CREDENTIALS = _load_tradier_file_credentials(_TRADIER_CREDENTIALS_FILE)


@dataclass
class TradingConfig:
    # Supported assets
    assets: list[str] = field(default_factory=lambda: ["BTC/USDT", "ETH/USDT"])
    asset_type: str = "crypto"  # crypto | stocks | multi | options

    # Exchange / broker
    exchange: str = "binance"
    paper_trading: bool = True
    tradier_credentials_file: str = _TRADIER_CREDENTIALS_FILE
    tradier_live_token: str = _clean_setting(os.getenv("TRADIER_LIVE_TOKEN"), _TRADIER_FILE_CREDENTIALS["live"].get("ACCESS_TOKEN", ""))
    tradier_paper_token: str = _clean_setting(os.getenv("TRADIER_PAPER_TOKEN"), _TRADIER_FILE_CREDENTIALS["paper"].get("ACCESS_TOKEN", ""))
    tradier_live_account_id: str = _clean_setting(os.getenv("TRADIER_LIVE_ACCOUNT_ID"), _TRADIER_FILE_CREDENTIALS["live"].get("ACCOUNT_ID", ""))
    tradier_paper_account_id: str = _clean_setting(os.getenv("TRADIER_PAPER_ACCOUNT_ID"), _TRADIER_FILE_CREDENTIALS["paper"].get("ACCOUNT_ID", ""))
    tradier_live_base_url: str = _clean_setting(os.getenv("TRADIER_LIVE_BASE_URL"), _TRADIER_FILE_CREDENTIALS["live"].get("BASE_URL", "https://api.tradier.com/v1")).rstrip("/")
    tradier_paper_base_url: str = _clean_setting(os.getenv("TRADIER_PAPER_BASE_URL"), _TRADIER_FILE_CREDENTIALS["paper"].get("BASE_URL", "https://sandbox.tradier.com/v1")).rstrip("/")
    tradier_default_scope: str = _clean_setting(os.getenv("TRADIER_DEFAULT_SCOPE")).lower()
    tradier_timeout_sec: int = int(os.getenv("TRADIER_TIMEOUT_SEC", "15"))

    # PDT / monitoring controls
    tradier_monitor_cache_ttl_sec: int = int(os.getenv("TRADIER_MONITOR_CACHE_TTL_SEC", "300"))
    tradier_probability_refresh_sec: int = int(os.getenv("TRADIER_PROBABILITY_REFRESH_SEC", "300"))
    tradier_live_update_interval_sec: int = int(os.getenv("TRADIER_LIVE_UPDATE_INTERVAL_SEC", "5"))
    tradier_journal_sync_interval_sec: int = int(os.getenv("TRADIER_JOURNAL_SYNC_INTERVAL_SEC", "5"))
    tradier_journal_history_days: int = int(os.getenv("TRADIER_JOURNAL_HISTORY_DAYS", "30"))
    tradier_pdt_min_equity: float = float(os.getenv("TRADIER_PDT_MIN_EQUITY", "25000"))
    tradier_pdt_max_day_trades: int = int(os.getenv("TRADIER_PDT_MAX_DAY_TRADES", "3"))
    tradier_pdt_window_days: int = int(os.getenv("TRADIER_PDT_WINDOW_DAYS", "5"))
    tradier_pdt_history_fill_limit: int = int(os.getenv("TRADIER_PDT_HISTORY_FILL_LIMIT", "200"))
    tradier_pdt_fail_closed: bool = os.getenv("TRADIER_PDT_FAIL_CLOSED", "true").strip().lower() not in {"0", "false", "no"}

    # Opportunity scanner
    scanner_auto_start: bool = os.getenv("QUANT_SCANNER_AUTO_START", "true").strip().lower() not in {"0", "false", "no"}
    scanner_enabled: bool = os.getenv("QUANT_SCANNER_ENABLED", "true").strip().lower() not in {"0", "false", "no"}
    scanner_source: str = _clean_setting(os.getenv("QUANT_SCANNER_SOURCE"), "yfinance").lower()
    scanner_scan_interval_sec: int = int(os.getenv("QUANT_SCANNER_INTERVAL_SEC", "180"))
    scanner_min_signal_strength: float = float(os.getenv("QUANT_SCANNER_MIN_SIGNAL_STRENGTH", "0.55"))
    scanner_min_local_win_rate_pct: float = float(os.getenv("QUANT_SCANNER_MIN_LOCAL_WIN_RATE_PCT", "53"))
    scanner_min_selection_score: float = float(os.getenv("QUANT_SCANNER_MIN_SELECTION_SCORE", "62"))
    scanner_max_candidates: int = int(os.getenv("QUANT_SCANNER_MAX_CANDIDATES", "8"))
    scanner_activity_limit: int = int(os.getenv("QUANT_SCANNER_ACTIVITY_LIMIT", "160"))
    scanner_require_higher_tf_confirmation: bool = os.getenv("QUANT_SCANNER_REQUIRE_HIGHER_TF", "true").strip().lower() not in {"0", "false", "no"}
    scanner_universe_raw: str = _clean_setting(
        os.getenv("QUANT_SCANNER_UNIVERSE"),
        "SPY,QQQ,IWM,AAPL,MSFT,NVDA,AMZN,META,AMD,TSLA",
    )
    scanner_timeframes_raw: str = _clean_setting(
        os.getenv("QUANT_SCANNER_TIMEFRAMES"),
        "5m,15m,1h,4h,1d",
    )

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
    journal_db_path: Path = BASE_DIR / "data" / "journal" / "trading_journal.sqlite3"
    journal_db_url: str = os.getenv("TRADIER_JOURNAL_DB_URL", "").strip()

    def __post_init__(self) -> None:
        if self.tradier_default_scope not in {"live", "paper"}:
            self.tradier_default_scope = "paper" if self.paper_trading else "live"
        self.scanner_source = self.scanner_source if self.scanner_source in {"yfinance", "ccxt"} else "yfinance"
        self.scanner_scan_interval_sec = max(15, min(self.scanner_scan_interval_sec, 3600))
        self.scanner_min_signal_strength = max(0.0, min(self.scanner_min_signal_strength, 1.0))
        self.scanner_min_local_win_rate_pct = max(0.0, min(self.scanner_min_local_win_rate_pct, 100.0))
        self.scanner_min_selection_score = max(0.0, min(self.scanner_min_selection_score, 100.0))
        self.scanner_max_candidates = max(1, min(self.scanner_max_candidates, 50))
        self.scanner_activity_limit = max(20, min(self.scanner_activity_limit, 1000))
        self.scanner_universe = [item.strip().upper() for item in self.scanner_universe_raw.split(",") if item.strip()]
        self.scanner_timeframes = [item.strip() for item in self.scanner_timeframes_raw.split(",") if item.strip()]
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self.models_dir.mkdir(parents=True, exist_ok=True)
        self.logs_dir.mkdir(parents=True, exist_ok=True)
        self.journal_db_path.parent.mkdir(parents=True, exist_ok=True)
        if not self.journal_db_url:
            self.journal_db_url = f"sqlite:///{self.journal_db_path.as_posix()}"


settings = TradingConfig()

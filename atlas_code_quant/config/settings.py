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


def _load_credentials_file_to_env() -> None:
    """Carga todos los pares KEY=VALUE de credenciales.txt en os.environ (sin sobreescribir).

    Permite que AlertDispatcher, TelegramBridge y otros módulos lean las credenciales
    aunque Code-Quant arranque como proceso independiente (sin Atlas Core).
    """
    path = DEFAULT_TRADIER_CREDENTIALS_FILE
    if not path.exists():
        return
    # Estas claves las maneja _load_tradier_file_credentials con contexto de sección
    _tradier_only = {"ACCOUNT_ID", "ACCESS_TOKEN", "BASE_URL"}
    try:
        for raw_line in path.read_text(encoding="utf-8-sig", errors="ignore").splitlines():
            line = raw_line.strip()
            if not line or line.startswith("#") or line.startswith("["):
                continue
            if "=" not in line:
                continue
            key, _, value = line.partition("=")
            key = key.strip()
            value = value.strip()
            if not key or key in _tradier_only:
                continue
            if key not in os.environ:  # no sobreescribir vars ya definidas
                os.environ[key] = value
    except Exception:
        pass


_load_credentials_file_to_env()


def _ienv(key: str, default: int) -> int:
    """int(os.getenv) con fallback seguro ante valores malformados."""
    try:
        return int(os.getenv(key, str(default)))
    except (ValueError, TypeError):
        return default


def _fenv(key: str, default: float) -> float:
    """float(os.getenv) con fallback seguro ante valores malformados."""
    try:
        return float(os.getenv(key, str(default)))
    except (ValueError, TypeError):
        return default


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
    tradier_timeout_sec: int = _ienv("TRADIER_TIMEOUT_SEC", 15)

    # PDT / monitoring controls
    tradier_monitor_cache_ttl_sec: int = _ienv("TRADIER_MONITOR_CACHE_TTL_SEC", 300)
    tradier_probability_refresh_sec: int = _ienv("TRADIER_PROBABILITY_REFRESH_SEC", 300)
    tradier_live_update_interval_sec: int = _ienv("TRADIER_LIVE_UPDATE_INTERVAL_SEC", 2)
    tradier_journal_sync_interval_sec: int = _ienv("TRADIER_JOURNAL_SYNC_INTERVAL_SEC", 5)
    tradier_journal_history_days: int = _ienv("TRADIER_JOURNAL_HISTORY_DAYS", 30)
    tradier_pdt_min_equity: float = _fenv("TRADIER_PDT_MIN_EQUITY", 25000.0)
    tradier_pdt_max_day_trades: int = _ienv("TRADIER_PDT_MAX_DAY_TRADES", 3)
    tradier_pdt_window_days: int = _ienv("TRADIER_PDT_WINDOW_DAYS", 5)
    tradier_pdt_history_fill_limit: int = _ienv("TRADIER_PDT_HISTORY_FILL_LIMIT", 200)
    tradier_pdt_fail_closed: bool = os.getenv("TRADIER_PDT_FAIL_CLOSED", "true").strip().lower() not in {"0", "false", "no"}

    # ── Multi-asset: opciones ─────────────────────────────────────────────────
    options_enabled:          bool  = os.getenv("ATLAS_OPTIONS_ENABLED", "false").strip().lower() not in {"0", "false", "no"}
    options_min_iv_rank:      float = _fenv("ATLAS_OPTIONS_MIN_IV_RANK", 25.0)
    options_max_spread_pct:   float = _fenv("ATLAS_OPTIONS_MAX_SPREAD_PCT", 0.20)
    options_min_dte:          int   = _ienv("ATLAS_OPTIONS_MIN_DTE", 14)
    options_max_dte:          int   = _ienv("ATLAS_OPTIONS_MAX_DTE", 45)
    options_target_dte:       int   = _ienv("ATLAS_OPTIONS_TARGET_DTE", 30)
    options_min_oi:           int   = _ienv("ATLAS_OPTIONS_MIN_OI", 100)
    options_max_bpr_pct:      float = _fenv("ATLAS_OPTIONS_MAX_BPR_PCT", 0.02)
    options_width_pct:        float = _fenv("ATLAS_OPTIONS_WIDTH_PCT", 0.03)
    # ── Multi-asset: ETF ──────────────────────────────────────────────────────
    etf_options_enabled:      bool  = os.getenv("ATLAS_ETF_OPTIONS_ENABLED", "false").strip().lower() not in {"0", "false", "no"}
    etf_universe_raw:         str   = _clean_setting(os.getenv("ATLAS_ETF_UNIVERSE"), "SPY,QQQ,IWM,GLD,TLT,XLF,XLE,XLK")
    # ── Multi-asset: índices ──────────────────────────────────────────────────
    index_options_enabled:    bool  = os.getenv("ATLAS_INDEX_OPTIONS_ENABLED", "false").strip().lower() not in {"0", "false", "no"}
    index_universe_raw:       str   = _clean_setting(os.getenv("ATLAS_INDEX_UNIVERSE"), "SPX")
    spx_max_contracts:        int   = _ienv("ATLAS_SPX_MAX_CONTRACTS", 1)
    index_options_min_dte:    int   = _ienv("ATLAS_INDEX_OPTIONS_MIN_DTE", 7)
    index_options_max_dte:    int   = _ienv("ATLAS_INDEX_OPTIONS_MAX_DTE", 45)
    # ── Multi-asset: crypto ───────────────────────────────────────────────────
    crypto_enabled:           bool  = os.getenv("ATLAS_CRYPTO_ENABLED", "false").strip().lower() not in {"0", "false", "no"}
    crypto_universe_raw:      str   = _clean_setting(os.getenv("ATLAS_CRYPTO_UNIVERSE"), "BTC/USDT,ETH/USDT")
    crypto_exchange:          str   = _clean_setting(os.getenv("ATLAS_CRYPTO_EXCHANGE"), "binance")
    # ── Multi-asset: futuros (Fase 5) ─────────────────────────────────────────
    futures_enabled:          bool  = os.getenv("ATLAS_FUTURES_ENABLED", "false").strip().lower() not in {"0", "false", "no"}
    futures_universe_raw:     str   = _clean_setting(os.getenv("ATLAS_FUTURES_UNIVERSE"), "MES,MNQ")

    # Alpaca — fallback paper cuando Tradier sandbox no responde
    # Cuenta gratuita en: https://app.alpaca.markets → Paper Trading
    alpaca_api_key:    str = _clean_setting(os.getenv("ALPACA_API_KEY"))
    alpaca_secret_key: str = _clean_setting(os.getenv("ALPACA_SECRET_KEY"))
    alpaca_paper_fallback: bool = os.getenv("ALPACA_PAPER_FALLBACK", "true").strip().lower() not in {"0", "false", "no"}

    # ATLAS brain bridge
    atlas_brain_enabled: bool = os.getenv("ATLAS_BRAIN_BRIDGE_ENABLED", "true").strip().lower() not in {"0", "false", "no"}
    atlas_brain_base_url: str = _clean_setting(os.getenv("ATLAS_BRAIN_BASE_URL"), "http://127.0.0.1:8791").rstrip("/")
    atlas_brain_api_key: str = _clean_setting(os.getenv("ATLAS_BRAIN_API_KEY"))
    atlas_brain_timeout_sec: int = _ienv("ATLAS_BRAIN_TIMEOUT_SEC", 5)
    atlas_brain_memory_enabled: bool = os.getenv("ATLAS_BRAIN_MEMORY_ENABLED", "true").strip().lower() not in {"0", "false", "no"}
    atlas_brain_bitacora_enabled: bool = os.getenv("ATLAS_BRAIN_BITACORA_ENABLED", "true").strip().lower() not in {"0", "false", "no"}
    atlas_brain_source: str = _clean_setting(os.getenv("ATLAS_BRAIN_SOURCE"), "quant_brain")

    # Opportunity scanner
    scanner_auto_start: bool = os.getenv("QUANT_SCANNER_AUTO_START", "true").strip().lower() not in {"0", "false", "no"}
    scanner_enabled: bool = os.getenv("QUANT_SCANNER_ENABLED", "true").strip().lower() not in {"0", "false", "no"}
    scanner_source: str = _clean_setting(os.getenv("QUANT_SCANNER_SOURCE"), "yfinance").lower()
    scanner_scan_interval_sec: int = _ienv("QUANT_SCANNER_INTERVAL_SEC", 180)
    scanner_min_signal_strength: float = _fenv("QUANT_SCANNER_MIN_SIGNAL_STRENGTH", 0.55)
    scanner_min_local_win_rate_pct: float = _fenv("QUANT_SCANNER_MIN_LOCAL_WIN_RATE_PCT", 53.0)
    scanner_min_selection_score: float = _fenv("QUANT_SCANNER_MIN_SELECTION_SCORE", 75.0)
    scanner_max_candidates: int = _ienv("QUANT_SCANNER_MAX_CANDIDATES", 8)
    scanner_activity_limit: int = _ienv("QUANT_SCANNER_ACTIVITY_LIMIT", 160)
    scanner_require_higher_tf_confirmation: bool = os.getenv("QUANT_SCANNER_REQUIRE_HIGHER_TF", "true").strip().lower() not in {"0", "false", "no"}
    scanner_universe_mode: str = _clean_setting(os.getenv("QUANT_SCANNER_UNIVERSE_MODE"), "us_equities_rotating").lower()
    scanner_universe_batch_size: int = _ienv("QUANT_SCANNER_UNIVERSE_BATCH_SIZE", 80)
    scanner_prefilter_count: int = _ienv("QUANT_SCANNER_PREFILTER_COUNT", 20)
    scanner_prefilter_min_price: float = _fenv("QUANT_SCANNER_PREFILTER_MIN_PRICE", 5.0)
    scanner_prefilter_min_dollar_volume_millions: float = _fenv("QUANT_SCANNER_PREFILTER_MIN_DOLLAR_VOLUME_M", 10.0)
    scanner_universe_cache_ttl_sec: int = _ienv("QUANT_SCANNER_UNIVERSE_CACHE_TTL_SEC", 86400)
    scanner_universe_raw: str = _clean_setting(
        os.getenv("QUANT_SCANNER_UNIVERSE"),
        "SPY,QQQ,IWM,AAPL,MSFT,NVDA,AMZN,META,AMD,TSLA",
    )
    scanner_timeframes_raw: str = _clean_setting(
        os.getenv("QUANT_SCANNER_TIMEFRAMES"),
        "5m,15m,1h,4h,1d",
    )
    # Multi-asset universe expansion
    scanner_include_etfs: bool    = os.getenv("ATLAS_SCANNER_INCLUDE_ETFS",    "true").strip().lower()  not in {"0","false","no"}
    scanner_include_indices: bool = os.getenv("ATLAS_SCANNER_INCLUDE_INDICES", "true").strip().lower()  not in {"0","false","no"}
    scanner_include_crypto: bool  = os.getenv("ATLAS_SCANNER_INCLUDE_CRYPTO",  "false").strip().lower() not in {"0","false","no"}

    # AtlasLearningBrain — criterios de readiness y scoring híbrido
    learning_brain_enabled: bool = os.getenv("ATLAS_LEARNING_BRAIN_ENABLED", "true").strip().lower() not in {"0", "false", "no"}
    learning_brain_ml_weight: float = _fenv("ATLAS_LEARNING_ML_WEIGHT", 0.4)
    learning_brain_stats_weight: float = _fenv("ATLAS_LEARNING_STATS_WEIGHT", 0.6)
    learning_brain_retrain_every_n: int = _ienv("ATLAS_LEARNING_RETRAIN_N", 50)
    # Umbrales de readiness para live
    readiness_min_n_trades: int = _ienv("ATLAS_READINESS_MIN_TRADES", 300)
    readiness_min_months: float = _fenv("ATLAS_READINESS_MIN_MONTHS", 3.0)
    readiness_min_profit_factor: float = _fenv("ATLAS_READINESS_MIN_PF", 1.5)
    readiness_min_calmar: float = _fenv("ATLAS_READINESS_MIN_CALMAR", 1.5)
    readiness_max_dd_pct: float = _fenv("ATLAS_READINESS_MAX_DD_PCT", 15.0)
    readiness_min_expectancy_r: float = _fenv("ATLAS_READINESS_MIN_EXPECTANCY_R", 0.20)
    readiness_min_stability: float = _fenv("ATLAS_READINESS_MIN_STABILITY", 0.70)

    # Adaptive learning
    adaptive_learning_enabled: bool = os.getenv("QUANT_ADAPTIVE_LEARNING_ENABLED", "true").strip().lower() not in {"0", "false", "no"}
    adaptive_learning_refresh_sec: int = _ienv("QUANT_ADAPTIVE_LEARNING_REFRESH_SEC", 300)
    adaptive_learning_window_days: int = _ienv("QUANT_ADAPTIVE_LEARNING_WINDOW_DAYS", 180)
    adaptive_learning_min_strategy_samples: int = _ienv("QUANT_ADAPTIVE_LEARNING_MIN_STRATEGY_SAMPLES", 4)
    adaptive_learning_min_symbol_samples: int = _ienv("QUANT_ADAPTIVE_LEARNING_MIN_SYMBOL_SAMPLES", 3)

    # Risk management — v2 logarítmico (Grok/xAI criterio)
    max_position_pct: float = 0.05             # Fallback si no hay Kelly
    max_drawdown_pct: float = 0.20             # 20% recomendado Grok (antes 15%)
    default_stop_loss_pct: float = 0.02
    default_take_profit_pct: float = 0.04
    # Kelly Criterion
    kelly_enabled: bool = True
    kelly_fraction: float = _fenv("QUANT_KELLY_FRACTION", 0.25)        # Quarter-Kelly
    kelly_max_position_pct: float = _fenv("QUANT_KELLY_MAX_PCT", 0.20)
    kelly_min_samples: int = _ienv("QUANT_KELLY_MIN_SAMPLES", 6)
    # ATR stops dinámicos
    atr_stops_enabled: bool = os.getenv("QUANT_ATR_STOPS_ENABLED", "true").strip().lower() not in {"0", "false", "no"}
    atr_sl_multiplier: float = _fenv("QUANT_ATR_SL_MULT", 1.5)        # SL = entry - ATR*1.5
    atr_tp_multiplier: float = _fenv("QUANT_ATR_TP_MULT", 3.0)        # TP = entry + ATR*3.0
    atr_period: int = _ienv("QUANT_ATR_PERIOD", 14)
    # Circuit breaker
    circuit_breaker_consecutive_losses: int = _ienv("QUANT_CB_CONSEC_LOSSES", 3)
    # Walk-forward validation
    walk_forward_enabled: bool = os.getenv("QUANT_WALK_FORWARD_ENABLED", "true").strip().lower() not in {"0", "false", "no"}
    walk_forward_folds: int = _ienv("QUANT_WALK_FORWARD_FOLDS", 5)
    walk_forward_train_pct: float = _fenv("QUANT_WF_TRAIN_PCT", 0.70)
    walk_forward_val_pct: float = _fenv("QUANT_WF_VAL_PCT", 0.15)

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
    atlas_brain_state_path: Path = BASE_DIR / "data" / "operation" / "quant_brain_bridge_state.json"
    atlas_brain_events_path: Path = BASE_DIR / "logs" / "quant_brain_bridge.jsonl"
    adaptive_learning_snapshot_path: Path = BASE_DIR / "data" / "learning" / "adaptive_policy_snapshot.json"
    scanner_universe_cache_path: Path = BASE_DIR / "data" / "scanner" / "us_equities_universe.json"

    def __post_init__(self) -> None:
        if self.tradier_default_scope not in {"live", "paper"}:
            self.tradier_default_scope = "paper" if self.paper_trading else "live"
        self.scanner_source = self.scanner_source if self.scanner_source in {"yfinance", "ccxt"} else "yfinance"
        self.scanner_scan_interval_sec = max(15, min(self.scanner_scan_interval_sec, 3600))
        self.scanner_min_signal_strength = max(0.0, min(self.scanner_min_signal_strength, 1.0))
        self.scanner_min_local_win_rate_pct = max(0.0, min(self.scanner_min_local_win_rate_pct, 100.0))
        self.scanner_min_selection_score = max(75.0, min(self.scanner_min_selection_score, 100.0))
        self.scanner_max_candidates = max(1, min(self.scanner_max_candidates, 50))
        self.scanner_activity_limit = max(20, min(self.scanner_activity_limit, 1000))
        self.scanner_universe_mode = self.scanner_universe_mode if self.scanner_universe_mode in {"manual", "us_equities_rotating"} else "us_equities_rotating"
        self.scanner_universe_batch_size = max(20, min(self.scanner_universe_batch_size, 500))
        self.scanner_prefilter_count = max(8, min(self.scanner_prefilter_count, 80))
        self.scanner_prefilter_count = min(self.scanner_prefilter_count, self.scanner_universe_batch_size)
        self.scanner_prefilter_min_price = max(0.5, min(self.scanner_prefilter_min_price, 1000.0))
        self.scanner_prefilter_min_dollar_volume_millions = max(0.1, min(self.scanner_prefilter_min_dollar_volume_millions, 5000.0))
        self.scanner_universe_cache_ttl_sec = max(3600, min(self.scanner_universe_cache_ttl_sec, 604800))
        self.adaptive_learning_refresh_sec = max(30, min(self.adaptive_learning_refresh_sec, 3600))
        self.adaptive_learning_window_days = max(30, min(self.adaptive_learning_window_days, 730))
        self.adaptive_learning_min_strategy_samples = max(2, min(self.adaptive_learning_min_strategy_samples, 50))
        self.adaptive_learning_min_symbol_samples = max(2, min(self.adaptive_learning_min_symbol_samples, 50))
        self.scanner_universe = [item.strip().upper() for item in self.scanner_universe_raw.split(",") if item.strip()]
        self.scanner_timeframes = [item.strip() for item in self.scanner_timeframes_raw.split(",") if item.strip()]
        # Inject multi-asset symbols into universe (deduplicated, order preserved)
        _extra: list[str] = []
        if self.scanner_include_etfs:
            try:
                from scanner.etf_universe import ETF_OPTIONS_UNIVERSE
                _extra.extend(s for s in ETF_OPTIONS_UNIVERSE if s not in self.scanner_universe and s not in _extra)
            except Exception:
                pass
        if self.scanner_include_indices:
            try:
                from scanner.index_universe import INDEX_PROFILES
                _extra.extend(s for s in INDEX_PROFILES if s not in self.scanner_universe and s not in _extra)
            except Exception:
                pass
        if self.scanner_include_crypto:
            try:
                from scanner.crypto_universe import CRYPTO_SPOT_UNIVERSE
                _extra.extend(s for s in CRYPTO_SPOT_UNIVERSE if s not in self.scanner_universe and s not in _extra)
            except Exception:
                pass
        if _extra:
            self.scanner_universe = self.scanner_universe + _extra
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self.models_dir.mkdir(parents=True, exist_ok=True)
        self.logs_dir.mkdir(parents=True, exist_ok=True)
        self.journal_db_path.parent.mkdir(parents=True, exist_ok=True)
        self.atlas_brain_state_path.parent.mkdir(parents=True, exist_ok=True)
        self.atlas_brain_events_path.parent.mkdir(parents=True, exist_ok=True)
        self.adaptive_learning_snapshot_path.parent.mkdir(parents=True, exist_ok=True)
        self.scanner_universe_cache_path.parent.mkdir(parents=True, exist_ok=True)
        if not self.journal_db_url:
            self.journal_db_url = f"sqlite:///{self.journal_db_path.as_posix()}"


settings = TradingConfig()

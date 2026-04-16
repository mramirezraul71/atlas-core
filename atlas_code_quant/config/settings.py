"""Atlas Code-Quant central configuration."""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path


BASE_DIR = Path(__file__).resolve().parent.parent
DEFAULT_TRADIER_CREDENTIALS_FILE = Path(r"C:\dev\credenciales.txt")
DEFAULT_ATLAS_ENV_FILE = BASE_DIR.parent / "config" / "atlas.env"


def _clean_setting(value: str | None, default: str = "") -> str:
    return (value or default).strip()


def _load_env_file(path: Path, *, override: bool = False) -> None:
    """Carga pares KEY=VALUE básicos sin depender de bootstrap de Atlas.

    Quant puede arrancar como proceso independiente; por eso necesita ver atlas.env
    aunque no se inicie a través del adaptador principal.
    """
    try:
        if not path.exists() or not path.is_file():
            return
        for raw_line in path.read_text(encoding="utf-8-sig", errors="ignore").splitlines():
            line = raw_line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, value = line.split("=", 1)
            key = key.strip()
            value = value.strip().strip('"').strip("'")
            if not key:
                continue
            if override or key not in os.environ:
                os.environ[key] = value
    except Exception:
        pass


_load_env_file(DEFAULT_ATLAS_ENV_FILE, override=False)


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
    scanner_min_local_profit_factor: float = _fenv("QUANT_SCANNER_MIN_LOCAL_PROFIT_FACTOR", 1.05)
    scanner_min_backtest_sample: int = _ienv("QUANT_SCANNER_MIN_BACKTEST_SAMPLE", 12)
    scanner_min_selection_score: float = _fenv("QUANT_SCANNER_MIN_SELECTION_SCORE", 65.0)
    scanner_max_candidates: int = _ienv("QUANT_SCANNER_MAX_CANDIDATES", 8)
    scanner_activity_limit: int = _ienv("QUANT_SCANNER_ACTIVITY_LIMIT", 160)
    scanner_require_higher_tf_confirmation: bool = os.getenv("QUANT_SCANNER_REQUIRE_HIGHER_TF", "true").strip().lower() not in {"0", "false", "no"}
    scanner_universe_mode: str = _clean_setting(os.getenv("QUANT_SCANNER_UNIVERSE_MODE"), "us_equities_rotating").lower()
    scanner_universe_batch_size: int = _ienv("QUANT_SCANNER_UNIVERSE_BATCH_SIZE", 120)
    # Pesos del selection_score (suma ≈ 1.0; normalización en __post_init__ si hace falta)
    scanner_weight_lq: float = _fenv("QUANT_SCANNER_WEIGHT_LQ", 0.25)
    scanner_weight_rs: float = _fenv("QUANT_SCANNER_WEIGHT_RS", 0.12)
    scanner_weight_strength: float = _fenv("QUANT_SCANNER_WEIGHT_STRENGTH", 0.12)
    scanner_weight_alignment: float = _fenv("QUANT_SCANNER_WEIGHT_ALIGNMENT", 0.10)
    scanner_weight_evidence: float = _fenv("QUANT_SCANNER_WEIGHT_EVIDENCE", 0.06)
    scanner_weight_order_flow: float = _fenv("QUANT_SCANNER_WEIGHT_ORDER_FLOW", 0.15)
    scanner_weight_ofa: float = _fenv("QUANT_SCANNER_WEIGHT_OFA", 0.07)
    scanner_weight_xgboost_conf: float = _fenv("QUANT_SCANNER_WEIGHT_XGBOOST_CONF", 0.05)
    scanner_weight_ichimoku_conf: float = _fenv("QUANT_SCANNER_WEIGHT_ICHIMOKU_CONF", 0.05)
    scanner_weight_adx_adjust: float = _fenv("QUANT_SCANNER_WEIGHT_ADX_ADJUST", 0.04)
    # Fase 2 — contexto de mercado
    scanner_vix_enabled: bool = os.getenv("QUANT_SCANNER_VIX_ENABLED", "true").strip().lower() not in {"0", "false", "no"}
    scanner_vix_panic_skip_cycle: bool = os.getenv("QUANT_SCANNER_VIX_PANIC_SKIP", "true").strip().lower() not in {
        "0",
        "false",
        "no",
    }
    scanner_regime_adapt_enabled: bool = os.getenv("QUANT_SCANNER_REGIME_ADAPT", "true").strip().lower() not in {
        "0",
        "false",
        "no",
    }
    # MACD divergencia contraria al setup — dos modos (elegir uno vía env):
    # Opción 1 (recomendada para validar Fase 2 / paper 7d): mode=points, penalty en PUNTOS de selection_score.
    # Opción 2 (tras paper OK, optimizar): mode=percent, penalty = fracción (0.10 = restar 10% multiplicando score).
    # Env: QUANT_SCANNER_MACD_DIV_MODE=points|percent  · QUANT_SCANNER_MACD_DIV_PENALTY=<float>
    scanner_macd_divergence_mode: str = _clean_setting(os.getenv("QUANT_SCANNER_MACD_DIV_MODE"), "points").lower()
    scanner_macd_divergence_penalty: float = _fenv("QUANT_SCANNER_MACD_DIV_PENALTY", 10.0)
    scanner_stochastic_enabled: bool = os.getenv("QUANT_SCANNER_STOCHASTIC_ENABLED", "true").strip().lower() not in {
        "0",
        "false",
        "no",
    }
    # Fase 3 — CNN + Prophet activos por defecto; desactivar con QUANT_SCANNER_*_ENABLED=false si faltan deps.
    # On-chain / fundamental siguen en false por defecto (API o red).
    scanner_cnn_lstm_enabled: bool = os.getenv("QUANT_SCANNER_CNN_LSTM_ENABLED", "true").strip().lower() not in {
        "0",
        "false",
        "no",
    }
    scanner_cnn_lstm_pattern_threshold: float = _fenv("QUANT_SCANNER_CNN_PATTERN_THRESHOLD", 0.65)
    scanner_cnn_lstm_model_path: str = _clean_setting(
        os.getenv("QUANT_CNN_LSTM_MODEL"),
        str(BASE_DIR / "models" / "saved" / "cnn_lstm_trained.h5"),
    )
    scanner_prophet_enabled: bool = os.getenv("QUANT_SCANNER_PROPHET_ENABLED", "true").strip().lower() not in {
        "0",
        "false",
        "no",
    }
    scanner_prophet_min_bars: int = _ienv("QUANT_SCANNER_PROPHET_MIN_BARS", 80)
    scanner_onchain_enabled: bool = os.getenv("QUANT_SCANNER_ONCHAIN_ENABLED", "false").strip().lower() not in {
        "0",
        "false",
        "no",
    }
    scanner_fundamental_enabled: bool = os.getenv("QUANT_SCANNER_FUNDAMENTAL_ENABLED", "false").strip().lower() not in {
        "0",
        "false",
        "no",
    }
    scanner_fundamental_min_accept: float = _fenv("QUANT_SCANNER_FUNDAMENTAL_MIN_ACCEPT", 25.0)
    scanner_fundamental_reject_below: float = _fenv("QUANT_SCANNER_FUNDAMENTAL_REJECT_BELOW", 15.0)
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
    selector_session_mode: str = _clean_setting(os.getenv("QUANT_SELECTOR_SESSION_MODE"), "balanced").lower()
    selector_options_require_available: bool = os.getenv("QUANT_SELECTOR_OPTIONS_REQUIRE_AVAILABLE", "true").strip().lower() not in {"0", "false", "no"}
    startup_preload_sessions_in_background: bool = os.getenv(
        "QUANT_STARTUP_PRELOAD_SESSIONS_IN_BACKGROUND",
        "true",
    ).strip().lower() not in {"0", "false", "no"}
    startup_alert_dispatcher_enabled: bool = os.getenv(
        "QUANT_STARTUP_ALERT_DISPATCHER_ENABLED",
        "true",
    ).strip().lower() not in {"0", "false", "no"}
    startup_notifications_enabled: bool = os.getenv(
        "QUANT_STARTUP_NOTIFICATIONS_ENABLED",
        "true",
    ).strip().lower() not in {"0", "false", "no"}
    startup_journal_sync_enabled: bool = os.getenv(
        "QUANT_STARTUP_JOURNAL_SYNC_ENABLED",
        "true",
    ).strip().lower() not in {"0", "false", "no"}
    startup_scanner_enabled: bool = os.getenv(
        "QUANT_STARTUP_SCANNER_ENABLED",
        "true",
    ).strip().lower() not in {"0", "false", "no"}
    startup_learning_enabled: bool = os.getenv(
        "QUANT_STARTUP_LEARNING_ENABLED",
        "true",
    ).strip().lower() not in {"0", "false", "no"}
    startup_snapshot_prewarm_enabled: bool = os.getenv(
        "QUANT_STARTUP_SNAPSHOT_PREWARM_ENABLED",
        "true",
    ).strip().lower() not in {"0", "false", "no"}
    startup_visual_connect_enabled: bool = os.getenv(
        "QUANT_STARTUP_VISUAL_CONNECT_ENABLED",
        "true",
    ).strip().lower() not in {"0", "false", "no"}
    startup_alert_dispatcher_delay_sec: int = _ienv("QUANT_STARTUP_ALERT_DISPATCHER_DELAY_SEC", 1)
    startup_journal_sync_delay_sec: int = _ienv("QUANT_STARTUP_JOURNAL_SYNC_DELAY_SEC", 8)
    startup_scanner_delay_sec: int = _ienv("QUANT_STARTUP_SCANNER_DELAY_SEC", 10)
    startup_learning_delay_sec: int = _ienv("QUANT_STARTUP_LEARNING_DELAY_SEC", 10)
    lightweight_startup: bool = os.getenv("QUANT_LIGHTWEIGHT_STARTUP", "false").strip().lower() in {"1", "true", "yes"}
    autocycle_auto_start: bool = os.getenv("QUANT_AUTOCYCLE_AUTO_START", "true").strip().lower() not in {"0", "false", "no"}

    # Entry validation
    entry_validation_enabled: bool = os.getenv("QUANT_ENTRY_VALIDATION_ENABLED", "true").strip().lower() not in {"0", "false", "no"}
    entry_max_equity_spread_pct: float = _fenv("QUANT_ENTRY_MAX_EQUITY_SPREAD_PCT", 1.50)
    entry_max_adverse_drift_pct: float = _fenv("QUANT_ENTRY_MAX_ADVERSE_DRIFT_PCT", 1.00)
    entry_warn_drift_vs_expected_move_pct: float = _fenv("QUANT_ENTRY_WARN_DRIFT_SHARE_EXPECTED_MOVE_PCT", 25.0)
    chart_auto_open_enabled: bool = os.getenv("QUANT_CHART_AUTO_OPEN_ENABLED", "false").strip().lower() not in {"0", "false", "no"}
    chart_open_cooldown_sec: int = _ienv("QUANT_CHART_OPEN_COOLDOWN_SEC", 90)
    # Tras Popen del navegador: comprobar que exista proceso Chrome/Chromium (Windows: tasklist).
    chart_verify_after_open: bool = os.getenv("QUANT_CHART_VERIFY_AFTER_OPEN", "true").strip().lower() not in {"0", "false", "no"}
    chart_verify_delay_sec: float = _fenv("QUANT_CHART_VERIFY_DELAY_SEC", 1.25)
    # Conexión arranque API: visión persistida + URLs TradingView iguales que StrategySelector
    default_vision_provider: str = _clean_setting(os.getenv("QUANT_DEFAULT_VISION_PROVIDER"), "").strip().lower()
    startup_chart_warmup_enabled: bool = os.getenv("QUANT_STARTUP_CHART_WARMUP", "false").strip().lower() not in {"0", "false", "no"}
    startup_chart_warmup_symbols_raw: str = _clean_setting(os.getenv("QUANT_STARTUP_CHART_SYMBOLS"), "SPY")
    startup_chart_warmup_timeframe: str = _clean_setting(os.getenv("QUANT_STARTUP_CHART_TIMEFRAME"), "1h")
    chart_provider_default: str = _clean_setting(os.getenv("QUANT_CHART_PROVIDER"), "tradingview")
    startup_chart_warmup_symbols: list[str] = field(default_factory=list)
    # Histéresis régimen ML: delta mínimo en prob. para cambiar de clase (evita flip-flop).
    regime_hysteresis_enabled: bool = os.getenv("QUANT_REGIME_HYSTERESIS_ENABLED", "true").strip().lower() not in {"0", "false", "no"}
    regime_hysteresis_margin: float = _fenv("QUANT_REGIME_HYSTERESIS_MARGIN", 0.08)
    # Recorte central del frame antes de OCR (0 = desactivado; 0.12 = quitar 12% por borde).
    vision_ocr_crop_margin: float = max(0.0, min(0.45, _fenv("QUANT_VISION_OCR_CROP_MARGIN", 0.0)))
    # Histéresis del régimen *contextual* (heurístico): puntos de score para cambiar primary_regime.
    context_regime_score_hysteresis: float = max(0.0, min(40.0, _fenv("QUANT_CONTEXT_REGIME_SCORE_HYSTERESIS", 10.0)))
    context_regime_strong_score: float = max(50.0, min(100.0, _fenv("QUANT_CONTEXT_REGIME_STRONG_SCORE", 72.0)))
    context_price_cycle_enabled: bool = os.getenv("QUANT_CONTEXT_PRICE_CYCLE_ENABLED", "true").strip().lower() not in {"0", "false", "no"}
    # Degradar gate (no bloquear) si ciclos ~ruido y confianza contextual baja; desactivado por defecto.
    context_cycle_soft_gate: bool = os.getenv("QUANT_CONTEXT_CYCLE_SOFT_GATE", "false").strip().lower() in {"1", "true", "yes"}
    context_cycle_soft_gate_max_regime_confidence: float = max(
        0.0, min(95.0, _fenv("QUANT_CONTEXT_CYCLE_SOFT_GATE_MAX_REGIME_CONF", 50.0))
    )
    visual_gate_min_readiness_pct: float = _fenv("QUANT_VISUAL_GATE_MIN_READINESS_PCT", 75.0)
    visual_gate_fail_closed: bool = os.getenv("QUANT_VISUAL_GATE_FAIL_CLOSED", "true").strip().lower() not in {"0", "false", "no"}
    visual_gate_supervised_manual_chart_preview: bool = os.getenv(
        "QUANT_VISUAL_GATE_SUPERVISED_MANUAL_CHART_PREVIEW",
        "true",
    ).strip().lower() not in {"0", "false", "no"}

    # Position management
    position_management_enabled: bool = os.getenv("QUANT_POSITION_MANAGEMENT_ENABLED", "true").strip().lower() not in {"0", "false", "no"}
    position_management_max_symbol_heat_pct: float = _fenv("QUANT_POSITION_MGMT_MAX_SYMBOL_HEAT_PCT", 12.0)
    position_management_max_unrealized_loss_r: float = _fenv("QUANT_POSITION_MGMT_MAX_UNREALIZED_LOSS_R", 0.35)
    position_management_max_thesis_drift_pct: float = _fenv("QUANT_POSITION_MGMT_MAX_THESIS_DRIFT_PCT", 15.0)
    position_management_max_stale_hours: int = _ienv("QUANT_POSITION_MGMT_MAX_STALE_HOURS", 72)
    position_management_var_mc_scenarios: int = _ienv("QUANT_POSITION_MGMT_VAR_MC_SCENARIOS", 750)
    position_management_var_confidence_pct: float = _fenv("QUANT_POSITION_MGMT_VAR_CONFIDENCE_PCT", 95.0)
    position_management_var_horizon_days: int = _ienv("QUANT_POSITION_MGMT_VAR_HORIZON_DAYS", 1)

    # Exit governance
    exit_governance_enabled: bool = os.getenv("QUANT_EXIT_GOVERNANCE_ENABLED", "true").strip().lower() not in {"0", "false", "no"}
    exit_governance_hard_exit_loss_r: float = _fenv("QUANT_EXIT_GOVERNANCE_HARD_EXIT_LOSS_R", 0.50)
    exit_governance_take_profit_r: float = _fenv("QUANT_EXIT_GOVERNANCE_TAKE_PROFIT_R", 0.75)
    exit_governance_time_stop_hours: int = _ienv("QUANT_EXIT_GOVERNANCE_TIME_STOP_HOURS", 96)
    exit_governance_trailing_profit_floor_r: float = _fenv("QUANT_EXIT_GOVERNANCE_TRAILING_PROFIT_FLOOR_R", 0.35)
    exit_governance_hard_exit_loss_usd: float = _fenv("QUANT_EXIT_GOVERNANCE_HARD_EXIT_LOSS_USD", 200.0)

    # Briefing operativo / notificaciones inteligentes (Telegram + WhatsApp vía AlertDispatcher)
    notify_enabled: bool = os.getenv("QUANT_NOTIFY_ENABLED", "false").strip().lower() in {"1", "true", "yes"}
    notify_premarket: bool = os.getenv("QUANT_NOTIFY_PREMARKET", "true").strip().lower() not in {"0", "false", "no"}
    notify_eod: bool = os.getenv("QUANT_NOTIFY_EOD", "true").strip().lower() not in {"0", "false", "no"}
    notify_intraday: bool = os.getenv("QUANT_NOTIFY_INTRADAY", "true").strip().lower() not in {"0", "false", "no"}
    notify_exit_intelligence: bool = os.getenv("QUANT_NOTIFY_EXIT_INTEL", "true").strip().lower() not in {"0", "false", "no"}
    notify_channels_raw: str = _clean_setting(os.getenv("QUANT_NOTIFY_CHANNELS"), "telegram,whatsapp")
    notify_min_severity: str = _clean_setting(os.getenv("QUANT_NOTIFY_MIN_SEVERITY"), "INFO").upper()
    notify_cooldown_sec: float = _fenv("QUANT_NOTIFY_COOLDOWN_SEC", 180.0)
    notify_dedup_ttl_sec: float = _fenv("QUANT_NOTIFY_DEDUP_TTL_SEC", 600.0)
    notify_max_opportunities: int = _ienv("QUANT_NOTIFY_MAX_OPPS", 5)
    notify_max_positions: int = _ienv("QUANT_NOTIFY_MAX_POSITIONS", 8)
    notify_render_mode: str = _clean_setting(os.getenv("QUANT_NOTIFY_RENDER"), "telegram_html")
    notify_paper_only: bool = os.getenv("QUANT_NOTIFY_PAPER_ONLY", "false").strip().lower() in {"1", "true", "yes"}
    notify_tz: str = _clean_setting(os.getenv("QUANT_NOTIFY_TZ"), "America/New_York")
    notify_premarket_hhmm: str = _clean_setting(os.getenv("QUANT_NOTIFY_PREMARKET_HHMM"), "08:00")
    notify_eod_hhmm: str = _clean_setting(os.getenv("QUANT_NOTIFY_EOD_HHMM"), "16:10")
    notify_scheduler_poll_sec: int = _ienv("QUANT_NOTIFY_SCHEDULER_POLL_SEC", 45)
    notify_intraday_min_interval_sec: float = _fenv("QUANT_NOTIFY_INTRADAY_MIN_SEC", 300.0)

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
    adaptive_learning_epoch_start: str = _clean_setting(os.getenv("QUANT_ADAPTIVE_LEARNING_EPOCH_START"), "2026-03-30")
    adaptive_learning_exclude_untracked: bool = os.getenv("QUANT_ADAPTIVE_LEARNING_EXCLUDE_UNTRACKED", "true").strip().lower() not in {"0", "false", "no"}
    adaptive_learning_require_trade_context: bool = os.getenv("QUANT_ADAPTIVE_LEARNING_REQUIRE_TRADE_CONTEXT", "true").strip().lower() not in {"0", "false", "no"}
    journal_quality_min_score_pct: float = _fenv("QUANT_JOURNAL_QUALITY_MIN_SCORE_PCT", 85.0)
    journal_quality_min_duration_sec: float = _fenv("QUANT_JOURNAL_QUALITY_MIN_DURATION_SEC", 5.0)
    journal_quality_outlier_day_share_pct: float = _fenv("QUANT_JOURNAL_QUALITY_OUTLIER_DAY_SHARE_PCT", 20.0)
    journal_quality_max_negative_price_ratio_pct: float = _fenv("QUANT_JOURNAL_QUALITY_MAX_NEGATIVE_PRICE_RATIO_PCT", 0.5)
    journal_quality_strategy_anomaly_min_samples: int = _ienv("QUANT_JOURNAL_QUALITY_STRATEGY_ANOMALY_MIN_SAMPLES", 250)
    journal_sync_partial_snapshot_guard_enabled: bool = os.getenv("QUANT_JOURNAL_SYNC_PARTIAL_GUARD", "true").strip().lower() not in {"0", "false", "no"}
    journal_sync_mass_close_ratio_pct: float = _fenv("QUANT_JOURNAL_SYNC_MASS_CLOSE_RATIO_PCT", 25.0)
    journal_sync_mass_close_min_positions: int = _ienv("QUANT_JOURNAL_SYNC_MASS_CLOSE_MIN_POSITIONS", 25)
    journal_sync_close_debounce_sec: int = _ienv("QUANT_JOURNAL_SYNC_CLOSE_DEBOUNCE_SEC", 60)

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
    api_port: int = 8795
    api_key: str = os.getenv("QUANT_API_KEY", "atlas-quant-local")
    # WebSocket dashboard: conexiones simultáneas por IP+scope (varias pestañas / reconexión)
    quant_dashboard_ws_limit: int = _ienv("QUANT_DASHBOARD_WS_LIMIT", 8)

    # XGBoost Signal Module (opt-in; por defecto desactivado)
    xgboost_enabled: bool = os.getenv("QUANT_XGBOOST_ENABLED", "false").strip().lower() not in {"0", "false", "no"}
    xgboost_min_trades_to_train: int = _ienv("QUANT_XGBOOST_MIN_TRADES", 30)
    xgboost_block_threshold: float = _fenv("QUANT_XGBOOST_BLOCK_THRESHOLD", 0.35)
    xgboost_warn_threshold: float = _fenv("QUANT_XGBOOST_WARN_THRESHOLD", 0.50)
    xgboost_exit_alert_threshold: float = _fenv("QUANT_XGBOOST_EXIT_ALERT_THRESHOLD", 0.55)

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
    notify_channels: list[str] = field(default_factory=list)
    notifications_data_dir: Path = field(default_factory=lambda: BASE_DIR / "data" / "notifications")
    xgboost_model_dir: Path = field(init=False)
    xgboost_audit_path: Path = field(init=False)

    def __post_init__(self) -> None:
        if self.tradier_default_scope not in {"live", "paper"}:
            self.tradier_default_scope = "paper" if self.paper_trading else "live"
        self.scanner_source = self.scanner_source if self.scanner_source in {"yfinance", "ccxt"} else "yfinance"
        self.scanner_scan_interval_sec = max(15, min(self.scanner_scan_interval_sec, 3600))
        self.scanner_min_signal_strength = max(0.0, min(self.scanner_min_signal_strength, 1.0))
        self.scanner_min_local_win_rate_pct = max(0.0, min(self.scanner_min_local_win_rate_pct, 100.0))
        self.scanner_min_local_profit_factor = max(0.5, min(self.scanner_min_local_profit_factor, 10.0))
        self.scanner_min_backtest_sample = max(3, min(self.scanner_min_backtest_sample, 200))
        self.scanner_min_selection_score = max(50.0, min(self.scanner_min_selection_score, 100.0))
        self.scanner_max_candidates = max(1, min(self.scanner_max_candidates, 50))
        self.scanner_activity_limit = max(20, min(self.scanner_activity_limit, 1000))
        self.scanner_universe_mode = self.scanner_universe_mode if self.scanner_universe_mode in {"manual", "us_equities_rotating"} else "us_equities_rotating"
        self.scanner_universe_batch_size = max(20, min(self.scanner_universe_batch_size, 500))
        _w_keys = (
            "scanner_weight_lq",
            "scanner_weight_rs",
            "scanner_weight_strength",
            "scanner_weight_alignment",
            "scanner_weight_evidence",
            "scanner_weight_order_flow",
            "scanner_weight_ofa",
            "scanner_weight_xgboost_conf",
            "scanner_weight_ichimoku_conf",
            "scanner_weight_adx_adjust",
        )
        for _k in _w_keys:
            _v = float(getattr(self, _k))
            setattr(self, _k, max(0.0, min(_v, 1.0)))
        _w_sum = sum(float(getattr(self, k)) for k in _w_keys)
        if _w_sum <= 0:
            raise ValueError("Los pesos del escáner deben sumar un valor positivo.")
        if not (0.99 <= _w_sum <= 1.01):
            _scale = 1.0 / _w_sum
            for _k in _w_keys:
                setattr(self, _k, round(float(getattr(self, _k)) * _scale, 6))
        _mdiv_mode = self.scanner_macd_divergence_mode if self.scanner_macd_divergence_mode in {"points", "percent"} else "points"
        self.scanner_macd_divergence_mode = _mdiv_mode
        if _mdiv_mode == "percent":
            self.scanner_macd_divergence_penalty = max(0.0, min(self.scanner_macd_divergence_penalty, 0.5))
        else:
            self.scanner_macd_divergence_penalty = max(0.0, min(self.scanner_macd_divergence_penalty, 15.0))
        self.scanner_cnn_lstm_pattern_threshold = max(0.5, min(self.scanner_cnn_lstm_pattern_threshold, 0.95))
        self.scanner_prophet_min_bars = max(40, min(self.scanner_prophet_min_bars, 500))
        self.scanner_fundamental_min_accept = max(0.0, min(self.scanner_fundamental_min_accept, 50.0))
        self.scanner_fundamental_reject_below = max(0.0, min(self.scanner_fundamental_reject_below, self.scanner_fundamental_min_accept))
        self.scanner_prefilter_count = max(8, min(self.scanner_prefilter_count, 80))
        self.scanner_prefilter_count = min(self.scanner_prefilter_count, self.scanner_universe_batch_size)
        self.scanner_prefilter_min_price = max(0.5, min(self.scanner_prefilter_min_price, 1000.0))
        self.scanner_prefilter_min_dollar_volume_millions = max(0.1, min(self.scanner_prefilter_min_dollar_volume_millions, 5000.0))
        self.scanner_universe_cache_ttl_sec = max(3600, min(self.scanner_universe_cache_ttl_sec, 604800))
        self.entry_max_equity_spread_pct = max(0.01, min(self.entry_max_equity_spread_pct, 5.0))
        self.entry_max_adverse_drift_pct = max(0.01, min(self.entry_max_adverse_drift_pct, 10.0))
        self.entry_warn_drift_vs_expected_move_pct = max(1.0, min(self.entry_warn_drift_vs_expected_move_pct, 100.0))
        self.chart_open_cooldown_sec = max(10, min(self.chart_open_cooldown_sec, 3600))
        self.startup_chart_warmup_symbols = [
            s.strip().upper() for s in self.startup_chart_warmup_symbols_raw.split(",") if s.strip()
        ][:8]
        if self.chart_provider_default not in {"tradingview", "yahoo"}:
            self.chart_provider_default = "tradingview"
        self.visual_gate_min_readiness_pct = max(0.0, min(self.visual_gate_min_readiness_pct, 100.0))
        self.position_management_max_symbol_heat_pct = max(1.0, min(self.position_management_max_symbol_heat_pct, 100.0))
        self.position_management_max_unrealized_loss_r = max(0.05, min(self.position_management_max_unrealized_loss_r, 10.0))
        self.position_management_max_thesis_drift_pct = max(1.0, min(self.position_management_max_thesis_drift_pct, 100.0))
        self.position_management_max_stale_hours = max(1, min(self.position_management_max_stale_hours, 24 * 30))
        self.position_management_var_mc_scenarios = max(250, min(self.position_management_var_mc_scenarios, 10000))
        self.position_management_var_confidence_pct = max(80.0, min(self.position_management_var_confidence_pct, 99.5))
        self.position_management_var_horizon_days = max(1, min(self.position_management_var_horizon_days, 10))
        self.exit_governance_hard_exit_loss_r = max(0.05, min(self.exit_governance_hard_exit_loss_r, 10.0))
        self.exit_governance_take_profit_r = max(0.05, min(self.exit_governance_take_profit_r, 20.0))
        self.exit_governance_time_stop_hours = max(1, min(self.exit_governance_time_stop_hours, 24 * 60))
        self.exit_governance_trailing_profit_floor_r = max(0.05, min(self.exit_governance_trailing_profit_floor_r, 20.0))
        self.exit_governance_hard_exit_loss_usd = max(10.0, min(self.exit_governance_hard_exit_loss_usd, 100000.0))
        self.adaptive_learning_refresh_sec = max(30, min(self.adaptive_learning_refresh_sec, 3600))
        self.adaptive_learning_window_days = max(30, min(self.adaptive_learning_window_days, 730))
        self.adaptive_learning_min_strategy_samples = max(2, min(self.adaptive_learning_min_strategy_samples, 50))
        self.adaptive_learning_min_symbol_samples = max(2, min(self.adaptive_learning_min_symbol_samples, 50))
        self.journal_quality_min_score_pct = max(0.0, min(self.journal_quality_min_score_pct, 100.0))
        self.journal_quality_min_duration_sec = max(0.0, min(self.journal_quality_min_duration_sec, 3600.0))
        self.journal_quality_outlier_day_share_pct = max(1.0, min(self.journal_quality_outlier_day_share_pct, 95.0))
        self.journal_quality_max_negative_price_ratio_pct = max(0.0, min(self.journal_quality_max_negative_price_ratio_pct, 100.0))
        self.journal_quality_strategy_anomaly_min_samples = max(25, min(self.journal_quality_strategy_anomaly_min_samples, 100000))
        self.journal_sync_mass_close_ratio_pct = max(1.0, min(self.journal_sync_mass_close_ratio_pct, 100.0))
        self.journal_sync_mass_close_min_positions = max(1, min(self.journal_sync_mass_close_min_positions, 100000))
        self.journal_sync_close_debounce_sec = max(0, min(self.journal_sync_close_debounce_sec, 86400))
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
        self.notifications_data_dir.mkdir(parents=True, exist_ok=True)
        chans = [c.strip().lower() for c in self.notify_channels_raw.split(",") if c.strip()]
        self.notify_channels = [c for c in chans if c in {"telegram", "whatsapp"}] or ["telegram"]
        self.notify_cooldown_sec = max(30.0, min(self.notify_cooldown_sec, 3600.0))
        self.notify_dedup_ttl_sec = max(60.0, min(self.notify_dedup_ttl_sec, 86400.0))
        self.notify_max_opportunities = max(1, min(self.notify_max_opportunities, 25))
        self.notify_max_positions = max(1, min(self.notify_max_positions, 40))
        self.notify_scheduler_poll_sec = max(15, min(self.notify_scheduler_poll_sec, 600))
        if self.notify_render_mode not in {"telegram_html", "plain"}:
            self.notify_render_mode = "telegram_html"
        if not self.journal_db_url:
            self.journal_db_url = f"sqlite:///{self.journal_db_path.as_posix()}"
        self.quant_dashboard_ws_limit = max(2, min(self.quant_dashboard_ws_limit, 64))

        _xgb_dir = _clean_setting(os.getenv("QUANT_XGBOOST_MODEL_PATH"))
        if _xgb_dir:
            p = Path(_xgb_dir)
            self.xgboost_model_dir = p if p.is_absolute() else (BASE_DIR / p).resolve()
        else:
            self.xgboost_model_dir = BASE_DIR / "learning" / "xgboost_signal" / "models"
        self.xgboost_model_dir.mkdir(parents=True, exist_ok=True)
        _aud = _clean_setting(os.getenv("QUANT_XGBOOST_AUDIT_PATH"))
        if _aud:
            ap = Path(_aud)
            self.xgboost_audit_path = ap if ap.is_absolute() else (BASE_DIR / ap).resolve()
        else:
            self.xgboost_audit_path = self.xgboost_model_dir / "audit_report.json"


settings = TradingConfig()

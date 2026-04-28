"""
lotto_quant.config
==================

Central configuration for the Atlas Lotto-Quant module.
All thresholds, URLs, tax parameters, and local-AI endpoints live here.
Override at runtime via environment variables (see `_env`).
"""

from __future__ import annotations

import os
from typing import Any


def _env(key: str, default: Any, cast: type = str) -> Any:
    """Read an environment variable with type casting."""
    raw = os.getenv(key)
    if raw is None or raw == "":
        return default
    if cast is bool:
        return raw.lower() in ("1", "true", "yes", "on")
    try:
        return cast(raw)
    except (TypeError, ValueError):
        return default


# ─────────────────────────────────────────────────────────────────────
# Tax parameters (NC-specific)
# ─────────────────────────────────────────────────────────────────────
NC_STATE_TAX_RATE: float = _env("ATLAS_NC_TAX", 0.0525, float)        # 5.25%
FEDERAL_TAX_RATE: float = _env("ATLAS_FED_TAX", 0.24, float)          # 24% on prizes >= $5,000
FEDERAL_SMALL_TAX_RATE: float = 0.0                                   # < $600 → no withholding

FEDERAL_WITHHOLDING_THRESHOLD: float = 5_000.0
SMALL_PRIZE_THRESHOLD: float = 600.0

# ─────────────────────────────────────────────────────────────────────
# EV thresholds
# ─────────────────────────────────────────────────────────────────────
MIN_EV_NET_POSITIVE: float = 0.00      # Minimum net EV (post-tax) to alert
MIN_EV_NET_STRONG: float = 0.05        # Strong signal: EV > 5%
MIN_PRIZE_DEPLETION_RATIO: float = 0.75   # Alert when >75% tickets sold
MIN_MAJOR_PRIZE_REMAINING_RATIO: float = 0.80  # AND >80% major prizes still available
ANOMALY_SCORE_STRONG: float = 3.75     # depletion / (1 - retention) above this = STRONG

# ─────────────────────────────────────────────────────────────────────
# Kelly parameters
# ─────────────────────────────────────────────────────────────────────
KELLY_FRACTION: float = 0.25           # Quarter-Kelly for model uncertainty
MAX_POSITION_PCT: float = 0.02         # Max 2% of bankroll per opportunity
MAX_TOTAL_EXPOSURE_PCT: float = 0.10   # Max 10% bankroll in lottery (HIGH VARIANCE asset)

# ─────────────────────────────────────────────────────────────────────
# Scraper parameters
# ─────────────────────────────────────────────────────────────────────
NCEL_BASE_URL: str = "https://nclottery.com/scratch-off-prizes-remaining"
SCRATCHODDS_URL: str = "https://scratchodds.com/north-carolina/scratch-off-remaining-prizes"
SCRAPE_INTERVAL_SECONDS: int = _env("ATLAS_SCRAPE_INTERVAL", 3600, int)  # 1h
REQUEST_TIMEOUT_S: int = 30
MAX_RETRIES: int = 3
RATE_LIMIT_DELAY_S: float = 5.0
USER_AGENT: str = (
    "Mozilla/5.0 (compatible; AtlasLottoQuant/1.0; "
    "+https://github.com/mramirezraul71/atlas-core)"
)

# ─────────────────────────────────────────────────────────────────────
# Jackpot thresholds (draw games)
# ─────────────────────────────────────────────────────────────────────
POWERBALL_EV_POSITIVE_JACKPOT: float = 600_000_000
MEGA_MILLIONS_EV_POSITIVE_JACKPOT: float = 550_000_000
POWERBALL_TICKET_PRICE: float = 2.0
MEGA_MILLIONS_TICKET_PRICE: float = 2.0
POWERBALL_ODDS_JACKPOT: float = 1 / 292_201_338
MEGA_MILLIONS_ODDS_JACKPOT: float = 1 / 302_575_350

# ─────────────────────────────────────────────────────────────────────
# OCR parameters
# ─────────────────────────────────────────────────────────────────────
OCR_CONFIDENCE_MIN: float = 0.85
INSTA360_STREAM_URL: str = _env(
    "ATLAS_INSTA360_URL", "rtsp://192.168.1.100:8080/live", str
)
TESSERACT_LANG: str = "eng"

# ─────────────────────────────────────────────────────────────────────
# Local AI (Ollama / DeepSeek / Qwen3) — replaces cloud Claude
# ─────────────────────────────────────────────────────────────────────
LOCAL_AI_BACKEND: str = _env("ATLAS_LLM_BACKEND", "ollama", str)  # 'ollama' | 'openai_compat'
LOCAL_AI_BASE_URL: str = _env("ATLAS_LLM_URL", "http://localhost:11434", str)
LOCAL_AI_MODEL: str = _env("ATLAS_LLM_MODEL", "qwen3:14b", str)
LOCAL_AI_FALLBACK_MODELS: tuple = (
    "deepseek-r1:14b",
    "qwen3:8b",
    "llama3.1:8b",
)
LOCAL_AI_TIMEOUT_S: int = _env("ATLAS_LLM_TIMEOUT", 120, int)
LOCAL_AI_TEMPERATURE: float = _env("ATLAS_LLM_TEMP", 0.2, float)
LOCAL_AI_MAX_TOKENS: int = _env("ATLAS_LLM_MAX_TOKENS", 2048, int)
LOCAL_AI_ENABLED: bool = _env("ATLAS_LLM_ENABLED", True, bool)

# ─────────────────────────────────────────────────────────────────────
# Database
# ─────────────────────────────────────────────────────────────────────
DB_PATH: str = _env("ATLAS_LOTTO_DB", "data/lotto_quant.duckdb", str)
DB_FALLBACK_SQLITE: str = _env("ATLAS_LOTTO_DB_FALLBACK", "data/lotto_quant.sqlite", str)

# ─────────────────────────────────────────────────────────────────────
# Alerts
# ─────────────────────────────────────────────────────────────────────
TELEGRAM_BOT_TOKEN: str = _env("ATLAS_TELEGRAM_TOKEN", "", str)
TELEGRAM_CHAT_ID: str = _env("ATLAS_TELEGRAM_CHAT_ID", "", str)
ENABLE_TELEGRAM: bool = bool(TELEGRAM_BOT_TOKEN and TELEGRAM_CHAT_ID)
ENABLE_CONSOLE_ALERTS: bool = True

# ─────────────────────────────────────────────────────────────────────
# Watchdog / Circuit breaker
# ─────────────────────────────────────────────────────────────────────
WATCHDOG_HEARTBEAT_S: int = 60
WATCHDOG_MAX_CONSECUTIVE_FAILURES: int = 5
CIRCUIT_BREAKER_COOLDOWN_S: int = 600  # 10 min

# ─────────────────────────────────────────────────────────────────────
# Markov simulation
# ─────────────────────────────────────────────────────────────────────
MARKOV_DEFAULT_SIMULATIONS: int = 10_000
MARKOV_PROJECTION_HORIZON: int = 100_000  # tickets ahead

# ─────────────────────────────────────────────────────────────────────
# Logging
# ─────────────────────────────────────────────────────────────────────
LOG_LEVEL: str = _env("ATLAS_LOG_LEVEL", "INFO", str)
LOG_FORMAT: str = "%(asctime)s | %(levelname)-7s | %(name)s | %(message)s"

"""
config.py — Carga de configuración del Radar Kalshi.

Responsabilidades
-----------------
1. Leer un archivo ``.env`` (ubicado en el mismo módulo o en
   ``C:\\ATLAS\\config\\.env``, alineado con el resto de Atlas).
2. Cargar la **API Key ID** de Kalshi y la **llave privada RSA** en
   formato PEM (firmas RSA-PSS según docs oficiales de Kalshi v2).
3. Exponer endpoints (REST/WS demo o producción) y la URL del
   servidor local de Ollama.
4. Centralizar parámetros del módulo (umbrales de edge, tamaño
   máximo de posición, fracción de Kelly, etc.) tipados con Pydantic.

Por qué Pydantic
----------------
- Validación estricta de tipos en runtime.
- Permite ``Settings`` cacheado (singleton vía ``functools.lru_cache``)
  y serializable a JSON para el dashboard.
- Compatible con futuras mejoras (Pydantic v2 + ``BaseSettings``).
"""
from __future__ import annotations

import os
from functools import lru_cache
from pathlib import Path
from typing import Optional

from dotenv import load_dotenv
from pydantic import BaseModel, Field, field_validator

# ---------------------------------------------------------------------------
# Localización del .env
# ---------------------------------------------------------------------------
# Convención del proyecto Atlas:
#   1) Si existe ``C:\ATLAS\config\.env`` (Windows / entorno productivo
#      del usuario), se usa.
#   2) En su defecto, ``modules/atlas_radar_kalshi/.env`` (desarrollo
#      local cross-platform).
_ATLAS_GLOBAL_ENV = Path(r"C:\ATLAS\config\.env")
_LOCAL_ENV = Path(__file__).resolve().parent / ".env"


def _load_env() -> None:
    """Carga variables de entorno desde el primer .env disponible."""
    if _ATLAS_GLOBAL_ENV.exists():
        load_dotenv(_ATLAS_GLOBAL_ENV, override=False)
    if _LOCAL_ENV.exists():
        load_dotenv(_LOCAL_ENV, override=False)


# ---------------------------------------------------------------------------
# Modelo tipado
# ---------------------------------------------------------------------------
class RadarSettings(BaseModel):
    """Configuración tipada del módulo Radar Kalshi."""

    # --- Kalshi -----------------------------------------------------------
    kalshi_api_key_id: str = Field(
        ...,
        description="UUID de la API Key de Kalshi (header KALSHI-ACCESS-KEY).",
    )
    kalshi_private_key_path: Path = Field(
        ...,
        description="Ruta al archivo .pem (RSA privada) usado para firmar "
        "requests con RSA-PSS + SHA-256.",
    )
    kalshi_environment: str = Field(
        default="demo",
        description="'demo' o 'prod'. Selecciona los endpoints REST/WS.",
    )
    kalshi_base_url: Optional[str] = Field(
        default=None,
        description="Override manual del base URL REST.",
    )
    kalshi_ws_url: Optional[str] = Field(
        default=None,
        description="Override manual del base URL WebSocket.",
    )

    # --- Ollama (cerebro local) -------------------------------------------
    ollama_endpoint: str = Field(
        default="http://127.0.0.1:11434",
        description="Endpoint HTTP del servidor local de Ollama.",
    )
    ollama_model: str = Field(
        default="qwen2.5-coder:7b",
        description="Modelo por defecto para sentimiento/razonamiento.",
    )
    ollama_timeout_seconds: float = Field(
        default=20.0,
        ge=0.5,
        le=120.0,
        description="Timeout de petición a Ollama por evaluación.",
    )
    ollama_backoff_seconds: int = Field(
        default=120,
        ge=5,
        le=3600,
        description="Backoff tras error de Ollama para evitar tormenta de reintentos.",
    )

    # --- Dashboard / Atlas Core -------------------------------------------
    atlas_dashboard_url: str = Field(
        default="http://127.0.0.1:8791",
        description="URL del dashboard central de atlas-core.",
    )
    radar_dashboard_path: str = Field(
        default="/ui/radar",
        description="Path montado por el módulo en el adapter HTTP de Atlas.",
    )

    # --- Risk & Edge ------------------------------------------------------
    edge_threshold: float = Field(
        default=0.05,
        ge=0.0,
        le=1.0,
        description="Edge mínimo (p_modelo - p_mercado) para disparar una "
        "orden. 0.05 = 5 puntos de probabilidad.",
    )
    kelly_fraction: float = Field(
        default=0.25,
        ge=0.0,
        le=1.0,
        description="Fracción de Kelly aplicada (0.25 = quarter-Kelly, "
        "estándar profesional para evitar varianza extrema).",
    )
    max_position_pct: float = Field(
        default=0.05,
        ge=0.0,
        le=1.0,
        description="Cap absoluto sobre el balance por posición.",
    )
    monte_carlo_iters: int = Field(
        default=10_000,
        ge=1_000,
        description="Iteraciones de Monte Carlo para validar p(éxito).",
    )

    # --- Universo de mercados --------------------------------------------
    market_status_filter: str = Field(
        default="open",
        description="Filtro REST aplicado a /markets. Puede ser una lista "
        "separada por comas: 'open,active'.",
    )
    poll_interval_seconds: int = Field(
        default=30,
        ge=5,
        description="Cadencia del sondeo REST para detectar nuevos mercados.",
    )
    market_discovery_limit: int = Field(
        default=120,
        ge=20,
        le=500,
        description="Límite de mercados por polling REST en /markets.",
    )
    new_market_events_max: int = Field(
        default=400,
        ge=0,
        le=10_000,
        description="Máximo total de eventos 'new_market' emitidos al bus interno.",
    )
    ws_max_tickers: int = Field(
        default=80,
        ge=10,
        le=500,
        description="Máximo de tickers a suscribir por conexión WebSocket.",
    )
    ticker_emit_min_ms: int = Field(
        default=250,
        ge=0,
        le=60_000,
        description="Intervalo mínimo entre eventos ticker emitidos por ticker.",
    )
    book_emit_min_ms: int = Field(
        default=500,
        ge=0,
        le=60_000,
        description="Intervalo mínimo entre eventos orderbook emitidos por ticker.",
    )
    event_queue_maxsize: int = Field(
        default=5_000,
        ge=100,
        le=50_000,
        description="Máximo de eventos en cola interna del scanner.",
    )
    decision_workers: int = Field(
        default=4,
        ge=1,
        le=16,
        description="Workers concurrentes para procesar eventos en orquestador.",
    )

    # --- Logging ----------------------------------------------------------
    log_dir: Path = Field(
        default=Path(r"C:\ATLAS\logs"),
        description="Directorio de logs compartido con el resto de Atlas.",
    )
    log_level: str = Field(default="INFO")

    # ------------------------------------------------------------------
    # Validators
    # ------------------------------------------------------------------
    @field_validator("kalshi_environment")
    @classmethod
    def _check_env(cls, v: str) -> str:
        v = v.lower().strip()
        if v not in {"demo", "prod"}:
            raise ValueError("kalshi_environment debe ser 'demo' o 'prod'.")
        return v

    @field_validator("kalshi_private_key_path")
    @classmethod
    def _check_pem(cls, v: Path) -> Path:
        if not v.exists():
            # No fallamos en import-time: el scanner/executor verificarán
            # antes de firmar. Esto facilita el dev sin .pem real.
            return v
        if v.suffix.lower() not in {".pem", ".key"}:
            raise ValueError(
                "Se esperaba un archivo .pem/.key con la llave privada RSA."
            )
        return v

    # ------------------------------------------------------------------
    # Endpoints derivados
    # ------------------------------------------------------------------
    @property
    def base_url(self) -> str:
        """Base URL REST efectivo según entorno."""
        if self.kalshi_base_url:
            return self.kalshi_base_url
        if self.kalshi_environment == "prod":
            return "https://api.elections.kalshi.com/trade-api/v2"
        return "https://demo-api.kalshi.co/trade-api/v2"

    @property
    def ws_url(self) -> str:
        """Base URL WebSocket efectivo según entorno."""
        if self.kalshi_ws_url:
            return self.kalshi_ws_url
        if self.kalshi_environment == "prod":
            return "wss://api.elections.kalshi.com/trade-api/ws/v2"
        return "wss://demo-api.kalshi.co/trade-api/ws/v2"


# ---------------------------------------------------------------------------
# Singleton accessor
# ---------------------------------------------------------------------------
@lru_cache(maxsize=1)
def get_settings() -> RadarSettings:
    """
    Retorna una instancia única de :class:`RadarSettings` con los valores
    cargados desde ``.env`` y los defaults razonables del módulo.
    """
    _load_env()
    return RadarSettings(
        kalshi_api_key_id=os.getenv("KALSHI_API_KEY", "") or os.getenv("KALSHI_API_KEY_ID", ""),
        kalshi_private_key_path=Path(
            os.getenv("KALSHI_PRIVATE_KEY_PATH", "./kalshi_private.pem")
        ),
        kalshi_environment=os.getenv("KALSHI_ENV", "demo"),
        kalshi_base_url=os.getenv("KALSHI_BASE_URL") or None,
        kalshi_ws_url=os.getenv("KALSHI_WS_URL") or None,
        ollama_endpoint=os.getenv("OLLAMA_ENDPOINT", "http://127.0.0.1:11434"),
        ollama_model=os.getenv("OLLAMA_MODEL", "qwen2.5-coder:7b"),
        ollama_timeout_seconds=float(os.getenv("RADAR_OLLAMA_TIMEOUT_S", "20")),
        ollama_backoff_seconds=int(os.getenv("RADAR_OLLAMA_BACKOFF_S", "120")),
        atlas_dashboard_url=os.getenv("ATLAS_DASHBOARD_URL", "http://127.0.0.1:8791"),
        edge_threshold=float(os.getenv("RADAR_EDGE_THRESHOLD", "0.05")),
        kelly_fraction=float(os.getenv("RADAR_KELLY_FRACTION", "0.25")),
        max_position_pct=float(os.getenv("RADAR_MAX_POSITION_PCT", "0.05")),
        monte_carlo_iters=int(os.getenv("RADAR_MC_ITERS", "2000")),
        market_status_filter=os.getenv("RADAR_MARKET_STATUS", "open"),
        poll_interval_seconds=int(os.getenv("RADAR_POLL_SECONDS", "30")),
        market_discovery_limit=int(os.getenv("RADAR_MARKET_DISCOVERY_LIMIT", "120")),
        new_market_events_max=int(os.getenv("RADAR_NEW_MARKET_EVENTS_MAX", "400")),
        ws_max_tickers=int(os.getenv("RADAR_WS_MAX_TICKERS", "80")),
        ticker_emit_min_ms=int(os.getenv("RADAR_TICKER_EMIT_MIN_MS", "250")),
        book_emit_min_ms=int(os.getenv("RADAR_BOOK_EMIT_MIN_MS", "500")),
        event_queue_maxsize=int(os.getenv("RADAR_EVENT_QUEUE_MAXSIZE", "5000")),
        decision_workers=int(os.getenv("RADAR_DECISION_WORKERS", "4")),
        log_dir=Path(os.getenv("ATLAS_LOG_DIR", r"C:\ATLAS\logs")),
        log_level=os.getenv("RADAR_LOG_LEVEL", "INFO"),
    )

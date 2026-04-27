"""Atlas Code Quant — Tradier institutional adapter (F15, paper/dry-run).

Capa **nueva** y aislada para enviar órdenes a Tradier en modo
paper / dry-run. F15 introduce la implementación documentada en el
estudio interno: idempotency por `trace_id`, retries con backoff
exponencial + jitter sólo en 5xx, rate limit lógico por minuto, y
reconciliación de posiciones cada 60 s (placeholder, paper-only).

Reglas duras (F15):

    * **NO live trading.** ``ATLAS_TRADIER_DRYRUN=True`` por defecto.
      Si está en True el adapter no envía ninguna orden real, sólo
      simula la respuesta.
    * **NO** importar ni delegar en
      ``atlas_code_quant.execution.tradier_execution`` (stack F4
      canónico) — F15 es **paralelo** y aislado para no romper la
      ruta productiva.
    * **NO** importar ``execution.broker_router``, ``signal_executor``,
      ``operation_center``, ``auton_executor``, ``live_activation``,
      ``live_loop`` ni ``start_paper_trading``.
    * No toca locks (``paper_only``, ``full_live_globally_locked``).
      El default de dry-run es **independiente** y sólo cubre F15.
    * Defensivo: nunca lanza hacia fuera; los fallos se devuelven
      como ``OrderResult`` con ``ok=False`` y código.

Ver:
    * docs/ATLAS_CODE_QUANT_F15_TRADIER_ADAPTER_INSTITUTIONAL.md
"""

from __future__ import annotations

import hashlib
import logging
import os
import random
import time
from dataclasses import dataclass, field
from typing import Any, Iterable, Optional

import httpx


logger = logging.getLogger("atlas.code_quant.execution.tradier_adapter")


__all__ = [
    "TradierAdapterConfig",
    "OrderIntent",
    "OrderResult",
    "TradierAdapter",
    "load_config_from_env",
    "ERROR_DRY_RUN_OK",
    "ERROR_RATE_LIMITED",
    "ERROR_BAD_REQUEST",
    "ERROR_HTTP_5XX",
    "ERROR_TRANSPORT",
    "ERROR_INVALID_INTENT",
]


# ---------------------------------------------------------------------------
# Constantes de error
# ---------------------------------------------------------------------------


ERROR_DRY_RUN_OK = "TRADIER_DRY_RUN_OK"
ERROR_RATE_LIMITED = "TRADIER_RATE_LIMITED"
ERROR_BAD_REQUEST = "TRADIER_BAD_REQUEST"
ERROR_HTTP_5XX = "TRADIER_HTTP_5XX"
ERROR_TRANSPORT = "TRADIER_TRANSPORT_ERROR"
ERROR_INVALID_INTENT = "TRADIER_INVALID_INTENT"


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------


def _env_bool(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.strip().lower() in ("1", "true", "yes", "on")


@dataclass(frozen=True)
class TradierAdapterConfig:
    """Configuración inmutable del adapter."""

    base_url: str = "https://sandbox.tradier.com"
    token: str = ""
    account_id: str = ""
    dry_run: bool = True
    max_orders_per_minute: int = 30
    max_retries_5xx: int = 3
    retry_base_sleep_sec: float = 0.05
    retry_max_sleep_sec: float = 1.0
    request_timeout_sec: float = 5.0

    @property
    def is_live(self) -> bool:
        return not self.dry_run


def load_config_from_env() -> TradierAdapterConfig:
    """Carga config desde variables de entorno con defaults seguros."""
    return TradierAdapterConfig(
        base_url=os.environ.get("ATLAS_TRADIER_BASE_URL")
        or "https://sandbox.tradier.com",
        token=os.environ.get("ATLAS_TRADIER_TOKEN", ""),
        account_id=os.environ.get("ATLAS_TRADIER_ACCOUNT_ID", ""),
        # Default DRY-RUN. Para salir de dry-run hace falta setear
        # ATLAS_TRADIER_DRYRUN=false EXPLICITAMENTE — F15 sólo
        # contempla paper, esto se mantiene como recordatorio.
        dry_run=_env_bool("ATLAS_TRADIER_DRYRUN", True),
        max_orders_per_minute=int(
            os.environ.get("ATLAS_TRADIER_MAX_OPM", "30") or "30"
        ),
    )


# ---------------------------------------------------------------------------
# OrderIntent / OrderResult
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class OrderIntent:
    """Intent de orden mínimo y suficiente para construir el body Tradier.

    F15 acepta single-leg simple. Para multileg / combos F15 emite
    payload genérico (``class="multileg"``) que LEAN/Tradier saben
    expandir. La construcción detallada del body multileg estará en
    una fase posterior cuando se conecte con ``StrategyIntent`` real.
    """

    symbol: str
    side: str  # "buy", "sell", "buy_to_open", "sell_to_close", ...
    quantity: int
    order_type: str = "market"
    duration: str = "day"
    trace_id: str = ""
    asset_class: str = "equity"  # "equity" | "option" | "multileg"
    legs: tuple[dict[str, Any], ...] = ()
    metadata: dict[str, Any] = field(default_factory=dict)

    def is_valid(self) -> bool:
        if not self.symbol:
            return False
        if int(self.quantity) <= 0:
            return False
        if self.side not in (
            "buy",
            "sell",
            "buy_to_open",
            "sell_to_open",
            "buy_to_close",
            "sell_to_close",
        ):
            return False
        return True

    def idempotency_key(self) -> str:
        """Key estable derivada de ``trace_id`` + campos clave del intent.

        Permite a Tradier deduplicar reintentos del mismo intent.
        """
        seed = (
            f"{self.trace_id}|{self.symbol}|{self.side}|"
            f"{int(self.quantity)}|{self.order_type}|{self.asset_class}"
        )
        return hashlib.sha256(seed.encode("utf-8")).hexdigest()[:32]


@dataclass(frozen=True)
class OrderResult:
    """Resultado canónico de un envío a Tradier."""

    ok: bool
    dry_run: bool
    http_status: int | None = None
    order_id: str | None = None
    error_code: str | None = None
    error_message: str | None = None
    attempts: int = 1
    idempotency_key: str | None = None
    raw_response: dict[str, Any] = field(default_factory=dict, repr=False, compare=False)

    def to_dict(self) -> dict[str, Any]:
        return {
            "ok": self.ok,
            "dry_run": self.dry_run,
            "http_status": self.http_status,
            "order_id": self.order_id,
            "error_code": self.error_code,
            "error_message": self.error_message,
            "attempts": self.attempts,
            "idempotency_key": self.idempotency_key,
        }


# ---------------------------------------------------------------------------
# Rate limiter lógico
# ---------------------------------------------------------------------------


class _SimpleMinuteRateLimiter:
    """Limita peticiones a N por ventana móvil de 60 s.

    Defensivo: si la ventana está saturada, ``acquire`` devuelve
    ``False`` sin dormir. Quien llama decide qué hacer (en F15:
    devolvemos ``OrderResult`` con ``TRADIER_RATE_LIMITED``).
    """

    def __init__(self, max_per_minute: int):
        self._max = max(int(max_per_minute), 1)
        self._timestamps: list[float] = []

    def _purge(self, now: float) -> None:
        cutoff = now - 60.0
        self._timestamps = [t for t in self._timestamps if t >= cutoff]

    def acquire(self, *, now: float | None = None) -> bool:
        t = now if now is not None else time.monotonic()
        self._purge(t)
        if len(self._timestamps) >= self._max:
            return False
        self._timestamps.append(t)
        return True


# ---------------------------------------------------------------------------
# Adapter
# ---------------------------------------------------------------------------


class TradierAdapter:
    """Adapter institucional Tradier (paper / dry-run, F15)."""

    def __init__(
        self,
        config: TradierAdapterConfig | None = None,
        *,
        client: Optional[httpx.Client] = None,
        sleep_fn=None,
    ):
        self._cfg = config or load_config_from_env()
        self._client = client
        self._owns_client = client is None
        # Permite a los tests reemplazar el sleep por un no-op.
        self._sleep = sleep_fn if sleep_fn is not None else time.sleep
        self._rate_limiter = _SimpleMinuteRateLimiter(
            self._cfg.max_orders_per_minute
        )
        self._reconcile_state: dict[str, Any] = {
            "last_run_ts": 0.0,
            "positions": [],
        }

    # ------------------------------------------------------------------
    # Helpers internos
    # ------------------------------------------------------------------

    @property
    def config(self) -> TradierAdapterConfig:
        return self._cfg

    def _ensure_client(self) -> httpx.Client:
        if self._client is None:
            self._client = httpx.Client(
                base_url=self._cfg.base_url,
                timeout=self._cfg.request_timeout_sec,
            )
        return self._client

    def close(self) -> None:
        if self._owns_client and self._client is not None:
            try:
                self._client.close()
            except Exception:  # noqa: BLE001
                pass

    def _backoff_sleep(self, attempt: int) -> None:
        base = self._cfg.retry_base_sleep_sec * (2 ** max(attempt - 1, 0))
        capped = min(base, self._cfg.retry_max_sleep_sec)
        jitter = random.uniform(0.0, capped * 0.1)
        self._sleep(capped + jitter)

    def _build_body(self, intent: OrderIntent) -> dict[str, Any]:
        """Construye el body para POST /accounts/{id}/orders.

        Para single-leg equity: campos clásicos.
        Para single-leg option / multileg: deja que Tradier reciba la
        forma genérica + ``legs``.
        """
        body: dict[str, Any] = {
            "class": intent.asset_class,
            "symbol": intent.symbol,
            "side": intent.side,
            "quantity": int(intent.quantity),
            "type": intent.order_type,
            "duration": intent.duration,
        }
        if intent.legs:
            body["legs"] = list(intent.legs)
        return body

    def _build_headers(self, *, idempotency_key: str) -> dict[str, str]:
        return {
            "Authorization": f"Bearer {self._cfg.token}",
            "Accept": "application/json",
            "X-Idempotency-Key": idempotency_key,
        }

    # ------------------------------------------------------------------
    # API pública
    # ------------------------------------------------------------------

    def submit(self, intent: OrderIntent) -> OrderResult:
        """Envía o simula un intent.

        * dry-run → no hay HTTP; respuesta ``ok=True, error_code=
          TRADIER_DRY_RUN_OK``.
        * live (no implementado en F15 en runtime) → respeta rate
          limit + idempotency + retries 5xx.
        """
        if not isinstance(intent, OrderIntent) or not intent.is_valid():
            return OrderResult(
                ok=False,
                dry_run=self._cfg.dry_run,
                error_code=ERROR_INVALID_INTENT,
                error_message="OrderIntent inválido",
                attempts=0,
            )

        idem = intent.idempotency_key()

        if self._cfg.dry_run:
            return OrderResult(
                ok=True,
                dry_run=True,
                http_status=None,
                order_id=f"DRY-{idem[:12]}",
                error_code=ERROR_DRY_RUN_OK,
                error_message=None,
                attempts=0,
                idempotency_key=idem,
                raw_response={"dry_run": True, "intent": intent.symbol},
            )

        # En F15 el envío live se implementa **defensivamente** pero
        # NO se cruza nunca en tests sin mock: ver doc F15. Si alguien
        # ejecutara este path sin Tradier real, el cliente HTTP daría
        # error de transport y se devuelve OrderResult honesto.
        if not self._rate_limiter.acquire():
            return OrderResult(
                ok=False,
                dry_run=False,
                error_code=ERROR_RATE_LIMITED,
                error_message="rate limit (max_orders_per_minute) exceeded",
                attempts=0,
                idempotency_key=idem,
            )

        body = self._build_body(intent)
        headers = self._build_headers(idempotency_key=idem)
        url = f"/v1/accounts/{self._cfg.account_id}/orders"

        client = self._ensure_client()
        attempts = 0
        last_status: int | None = None
        last_error: str | None = None
        for attempt in range(1, self._cfg.max_retries_5xx + 2):
            attempts = attempt
            try:
                resp = client.post(url, data=body, headers=headers)
            except httpx.HTTPError as exc:
                last_error = f"transport error: {exc}"
                # transport errors NO se reintenta agresivamente en F15:
                # devolvemos resultado honesto.
                return OrderResult(
                    ok=False,
                    dry_run=False,
                    error_code=ERROR_TRANSPORT,
                    error_message=last_error,
                    attempts=attempts,
                    idempotency_key=idem,
                )
            last_status = resp.status_code
            if 200 <= resp.status_code < 300:
                payload: dict[str, Any] = {}
                try:
                    payload = resp.json() if resp.content else {}
                except Exception:  # noqa: BLE001
                    payload = {}
                order_id = None
                # Tradier devuelve {"order": {"id": ...}}
                if isinstance(payload, dict):
                    order = payload.get("order")
                    if isinstance(order, dict):
                        order_id = str(order.get("id")) if order.get("id") else None
                return OrderResult(
                    ok=True,
                    dry_run=False,
                    http_status=resp.status_code,
                    order_id=order_id,
                    attempts=attempts,
                    idempotency_key=idem,
                    raw_response=payload if isinstance(payload, dict) else {},
                )
            if 400 <= resp.status_code < 500:
                # 4xx no se reintenta (bad request, auth, etc.).
                return OrderResult(
                    ok=False,
                    dry_run=False,
                    http_status=resp.status_code,
                    error_code=ERROR_BAD_REQUEST,
                    error_message=resp.text[:300],
                    attempts=attempts,
                    idempotency_key=idem,
                )
            # 5xx → backoff + retry.
            if attempt <= self._cfg.max_retries_5xx:
                self._backoff_sleep(attempt)
                continue
            return OrderResult(
                ok=False,
                dry_run=False,
                http_status=resp.status_code,
                error_code=ERROR_HTTP_5XX,
                error_message=resp.text[:300],
                attempts=attempts,
                idempotency_key=idem,
            )

        # No deberíamos llegar aquí.
        return OrderResult(
            ok=False,
            dry_run=False,
            http_status=last_status,
            error_code=ERROR_HTTP_5XX,
            error_message=last_error or "exhausted retries",
            attempts=attempts,
            idempotency_key=idem,
        )

    # ------------------------------------------------------------------
    # Reconcile (placeholder paper-safe)
    # ------------------------------------------------------------------

    def reconcile_positions(self) -> dict[str, Any]:
        """Reconcilia posiciones simuladas (paper).

        F15 mantiene un journal interno mínimo. Esta función está
        diseñada para ser llamada cada 60 s desde un scheduler
        externo (no implementado en F15) y NO toca Tradier real
        cuando ``dry_run=True``.
        """
        now = time.time()
        self._reconcile_state["last_run_ts"] = now
        if self._cfg.dry_run:
            return {
                "ok": True,
                "dry_run": True,
                "positions": list(self._reconcile_state.get("positions", [])),
                "last_run_ts": now,
            }
        # En live (F15 no llega aquí en runtime): consulta /positions.
        try:
            client = self._ensure_client()
            resp = client.get(
                f"/v1/accounts/{self._cfg.account_id}/positions",
                headers={
                    "Authorization": f"Bearer {self._cfg.token}",
                    "Accept": "application/json",
                },
            )
            data = resp.json() if resp.content else {}
            self._reconcile_state["positions"] = (
                data.get("positions") if isinstance(data, dict) else []
            )
            return {
                "ok": 200 <= resp.status_code < 300,
                "dry_run": False,
                "http_status": resp.status_code,
                "positions": self._reconcile_state["positions"],
                "last_run_ts": now,
            }
        except httpx.HTTPError as exc:
            return {
                "ok": False,
                "dry_run": False,
                "error_code": ERROR_TRANSPORT,
                "error_message": str(exc),
                "last_run_ts": now,
            }

"""Atlas Code Quant — Cliente HTTP/SSE del Radar (F6, shadow only).

Cliente *intake* aislado para consumir los endpoints multi-símbolo del Radar
expuestos por ``atlas_adapter`` (introducidos en F5):

    GET /api/radar/opportunities
    GET /api/radar/opportunities/{symbol}
    GET /api/radar/stream/opportunities      (SSE — opcional)

Todo el módulo es **shadow only** en F6:

    * No se invoca desde ``atlas_code_quant/api/main.py``.
    * No se conecta a operations / execution / autonomy / risk / live.
    * No ejecuta órdenes ni toca Tradier ni paper loop.
    * El flag :data:`atlas_code_quant.config.legacy_flags.ATLAS_RADAR_INTAKE_ENABLED`
      es doc-only (default ``False``); este módulo no lo consulta en runtime.

Manejo de errores (filosofía "no matar el proceso"):

    * Fallo de red, timeout, 5xx, JSON inválido o esquema inesperado se
      traducen a un :class:`RadarOpportunityBatchInternal.empty_with_degradation`
      con el código adecuado:

          * ``RADAR_UNREACHABLE``   — ConnectError / red caída.
          * ``RADAR_TIMEOUT``       — TimeoutException.
          * ``RADAR_HTTP_ERROR``    — status >= 500 (o 4xx no-404).
          * ``RADAR_INVALID_PAYLOAD`` — JSON no decodificable / no dict.

    * En :func:`RadarClient.fetch_opportunity` un 404 explícito devuelve
      ``None`` (símbolo no encontrado o universe filter), distinto de error.

Configuración por entorno:

    * ``ATLAS_RADAR_BASE_URL`` (default ``http://localhost:8791``)
    * ``ATLAS_RADAR_TIMEOUT_MS`` (default ``5000``)

Tests fuerzan ``transport=httpx.MockTransport`` para no abrir red.

Ver también:
    * docs/ATLAS_CODE_QUANT_F6_RADAR_INTAKE_SHADOW.md
    * atlas_code_quant/intake/opportunity.py
    * atlas_adapter/routes/radar_schemas.py
"""

from __future__ import annotations

import json
import logging
import os
from typing import Any, AsyncIterator, Mapping

import httpx

from atlas_code_quant.intake.opportunity import (
    RadarIntakeDegradation,
    RadarOpportunityBatchInternal,
    RadarOpportunityInternal,
    batch_from_radar_payload,
    from_radar_payload,
)

logger = logging.getLogger("atlas.code_quant.intake.radar")


__all__ = [
    "RadarClient",
    "RadarClientConfig",
    "fetch_opportunities_snapshot",
    "DEFAULT_BASE_URL",
    "DEFAULT_TIMEOUT_MS",
    "INTAKE_CODE_UNREACHABLE",
    "INTAKE_CODE_TIMEOUT",
    "INTAKE_CODE_HTTP_ERROR",
    "INTAKE_CODE_INVALID_PAYLOAD",
]


DEFAULT_BASE_URL: str = "http://localhost:8791"
DEFAULT_TIMEOUT_MS: int = 5000

INTAKE_CODE_UNREACHABLE: str = "RADAR_UNREACHABLE"
INTAKE_CODE_TIMEOUT: str = "RADAR_TIMEOUT"
INTAKE_CODE_HTTP_ERROR: str = "RADAR_HTTP_ERROR"
INTAKE_CODE_INVALID_PAYLOAD: str = "RADAR_INVALID_PAYLOAD"


# ---------------------------------------------------------------------------
# Configuración
# ---------------------------------------------------------------------------


class RadarClientConfig:
    """Configuración resuelta desde entorno (o explícita en tests).

    No es un Pydantic model a propósito: queremos cero dependencia extra y
    fallback honesto a defaults si las variables de entorno faltan o son
    malformadas.
    """

    __slots__ = ("base_url", "timeout_seconds")

    def __init__(
        self,
        base_url: str | None = None,
        timeout_ms: int | float | None = None,
    ) -> None:
        resolved_base = base_url if base_url is not None else os.getenv(
            "ATLAS_RADAR_BASE_URL", DEFAULT_BASE_URL
        )
        # Normaliza: quita trailing slash para evitar dobles barras al unir.
        self.base_url: str = (resolved_base or DEFAULT_BASE_URL).rstrip("/")

        if timeout_ms is None:
            env_val = os.getenv("ATLAS_RADAR_TIMEOUT_MS")
            try:
                ms = float(env_val) if env_val is not None else float(DEFAULT_TIMEOUT_MS)
            except (TypeError, ValueError):
                ms = float(DEFAULT_TIMEOUT_MS)
        else:
            try:
                ms = float(timeout_ms)
            except (TypeError, ValueError):
                ms = float(DEFAULT_TIMEOUT_MS)
        if ms <= 0:
            ms = float(DEFAULT_TIMEOUT_MS)
        self.timeout_seconds: float = ms / 1000.0

    def __repr__(self) -> str:  # pragma: no cover - debug only
        return (
            f"RadarClientConfig(base_url={self.base_url!r}, "
            f"timeout_seconds={self.timeout_seconds:.3f})"
        )


# ---------------------------------------------------------------------------
# Helpers privados
# ---------------------------------------------------------------------------


def _coerce_filters(filters: Mapping[str, Any] | None) -> dict[str, Any]:
    if not filters:
        return {}
    out: dict[str, Any] = {}
    for k, v in filters.items():
        if v is None:
            continue
        if isinstance(v, (list, tuple, set)):
            # httpx admite list values; los convertimos a list para estabilidad.
            out[str(k)] = [str(x) for x in v if x is not None]
        elif isinstance(v, bool):
            out[str(k)] = "true" if v else "false"
        else:
            out[str(k)] = str(v)
    return out


def _safe_json(resp: httpx.Response) -> Any:
    """Decodifica JSON tolerantemente.

    Devuelve ``None`` si no se puede decodificar (en ese caso el caller
    convierte la respuesta en ``RADAR_INVALID_PAYLOAD``).
    """
    try:
        return resp.json()
    except (ValueError, json.JSONDecodeError):
        return None


def _degradation_batch(
    code: str, label: str, *, severity: str = "warning"
) -> RadarOpportunityBatchInternal:
    return RadarOpportunityBatchInternal.empty_with_degradation(
        code=code,
        label=label,
        severity=severity,
        source="atlas.code_quant.intake.radar",
        trace_id="",
    )


# ---------------------------------------------------------------------------
# Cliente principal
# ---------------------------------------------------------------------------


class RadarClient:
    """Cliente HTTP del Radar para uso interno de Code Quant (F6 shadow).

    Uso típico (sincrónico)::

        client = RadarClient()
        batch = client.fetch_opportunities({"min_score": 60, "asset_class": "equity"})
        for opp in batch.items:
            ...  # consumo shadow (logging, métricas, comparación)

    Uso en tests::

        transport = httpx.MockTransport(handler)
        client = RadarClient(transport=transport)

    Parámetros
    ----------
    config:
        Override explícito (tests / wiring futuro). Si ``None`` se construye
        desde entorno con :class:`RadarClientConfig`.
    transport:
        ``httpx.BaseTransport`` opcional. Permite que los tests inyecten
        ``MockTransport`` sin tocar la red. Si se omite, ``httpx`` usa el
        transport por defecto.
    """

    def __init__(
        self,
        config: RadarClientConfig | None = None,
        *,
        transport: httpx.BaseTransport | None = None,
    ) -> None:
        self._config = config or RadarClientConfig()
        self._transport = transport

    # ------------------------------------------------------------------
    # API pública (sincrónica)
    # ------------------------------------------------------------------

    @property
    def config(self) -> RadarClientConfig:
        return self._config

    def fetch_opportunities(
        self, filters: Mapping[str, Any] | None = None
    ) -> RadarOpportunityBatchInternal:
        """Snapshot de oportunidades multi-símbolo desde el Radar.

        Devuelve siempre un :class:`RadarOpportunityBatchInternal`. En caso
        de error de intake (red, timeout, 5xx, JSON inválido) devuelve un
        batch vacío con la degradación correspondiente — **nunca** lanza.
        """
        params = _coerce_filters(filters)
        url = f"{self._config.base_url}/api/radar/opportunities"
        try:
            with self._build_client() as client:
                resp = client.get(url, params=params)
        except httpx.TimeoutException as exc:
            logger.warning("intake.radar timeout en %s: %s", url, exc)
            return _degradation_batch(
                INTAKE_CODE_TIMEOUT, "Radar fetch timeout", severity="warning"
            )
        except httpx.ConnectError as exc:
            logger.warning("intake.radar inalcanzable %s: %s", url, exc)
            return _degradation_batch(
                INTAKE_CODE_UNREACHABLE,
                "Radar host inalcanzable",
                severity="warning",
            )
        except httpx.HTTPError as exc:
            logger.warning("intake.radar httpx error %s: %s", url, exc)
            return _degradation_batch(
                INTAKE_CODE_UNREACHABLE,
                f"Radar transport error: {type(exc).__name__}",
                severity="warning",
            )

        if resp.status_code >= 500:
            logger.warning(
                "intake.radar 5xx en %s status=%s", url, resp.status_code
            )
            return _degradation_batch(
                INTAKE_CODE_HTTP_ERROR,
                f"Radar HTTP {resp.status_code}",
                severity="critical",
            )
        if resp.status_code >= 400:
            logger.warning(
                "intake.radar 4xx en %s status=%s", url, resp.status_code
            )
            return _degradation_batch(
                INTAKE_CODE_HTTP_ERROR,
                f"Radar HTTP {resp.status_code}",
                severity="warning",
            )

        data = _safe_json(resp)
        if not isinstance(data, dict):
            logger.warning("intake.radar payload no-dict en %s", url)
            return _degradation_batch(
                INTAKE_CODE_INVALID_PAYLOAD,
                "Radar respondió payload no JSON-objeto",
                severity="warning",
            )

        try:
            return batch_from_radar_payload(data)
        except Exception:  # noqa: BLE001 — última red de seguridad
            logger.exception("intake.radar fallo mapeando payload de %s", url)
            return _degradation_batch(
                INTAKE_CODE_INVALID_PAYLOAD,
                "Radar payload no mapeable a modelo interno",
                severity="warning",
            )

    def fetch_opportunity(self, symbol: str) -> RadarOpportunityInternal | None:
        """Snapshot de una sola oportunidad por símbolo.

        Devuelve ``None`` si:
            * el símbolo es vacío,
            * el Radar responde 404,
            * hay error de intake (red, timeout, 5xx, JSON inválido) — el
              error se loguea como warning y se traga.
        """
        sym = (symbol or "").strip().upper()
        if not sym:
            return None
        url = f"{self._config.base_url}/api/radar/opportunities/{sym}"
        try:
            with self._build_client() as client:
                resp = client.get(url)
        except httpx.TimeoutException as exc:
            logger.warning("intake.radar timeout en %s: %s", url, exc)
            return None
        except httpx.HTTPError as exc:
            logger.warning("intake.radar transport error en %s: %s", url, exc)
            return None

        if resp.status_code == 404:
            logger.info("intake.radar 404 (no opportunity) symbol=%s", sym)
            return None
        if resp.status_code >= 400:
            logger.warning(
                "intake.radar HTTP %s en %s", resp.status_code, url
            )
            return None

        data = _safe_json(resp)
        if not isinstance(data, dict):
            logger.warning("intake.radar payload no-dict en %s", url)
            return None
        try:
            return from_radar_payload(data)
        except Exception:  # noqa: BLE001
            logger.exception(
                "intake.radar fallo mapeando opportunity symbol=%s", sym
            )
            return None

    # ------------------------------------------------------------------
    # API pública (asíncrona — SSE opcional)
    # ------------------------------------------------------------------

    async def stream_opportunities(
        self,
        filters: Mapping[str, Any] | None = None,
        *,
        max_events: int | None = None,
    ) -> AsyncIterator[dict[str, Any]]:
        """Generador asíncrono de eventos SSE desde el Radar.

        Yield de ``dict`` por cada evento parseado (no se construye el
        :class:`RadarOpportunityStreamEvent` completo aquí; el caller decide
        qué mapear). El generador termina:

            * cuando el servidor cierra la conexión,
            * cuando se alcanzan ``max_events`` (defensivo en tests),
            * en cualquier excepción transport: se loguea y se sale en silencio.

        SHADOW: NO se conecta automáticamente a operations en F6.
        """
        params = _coerce_filters(filters)
        url = f"{self._config.base_url}/api/radar/stream/opportunities"
        # Streaming: timeout sólo de connect; el read timeout debería ser
        # tolerante porque SSE puede tardar entre eventos.
        timeout = httpx.Timeout(
            connect=self._config.timeout_seconds,
            read=None,
            write=self._config.timeout_seconds,
            pool=self._config.timeout_seconds,
        )
        try:
            async with httpx.AsyncClient(
                timeout=timeout, transport=self._async_transport()
            ) as client:
                async with client.stream("GET", url, params=params) as resp:
                    if resp.status_code >= 400:
                        logger.warning(
                            "intake.radar stream HTTP %s en %s",
                            resp.status_code,
                            url,
                        )
                        return
                    emitted = 0
                    data_buf: list[str] = []
                    async for line in resp.aiter_lines():
                        if line == "":
                            # Fin de evento SSE.
                            if data_buf:
                                payload_text = "\n".join(data_buf)
                                data_buf = []
                                try:
                                    payload = json.loads(payload_text)
                                except json.JSONDecodeError:
                                    logger.warning(
                                        "intake.radar stream evento JSON inválido"
                                    )
                                    continue
                                if isinstance(payload, dict):
                                    yield payload
                                    emitted += 1
                                    if max_events is not None and emitted >= max_events:
                                        return
                            continue
                        if line.startswith(":"):
                            # Comentario / heartbeat SSE.
                            continue
                        if line.startswith("data:"):
                            data_buf.append(line[5:].lstrip())
        except httpx.HTTPError as exc:
            logger.warning(
                "intake.radar stream transport error en %s: %s", url, exc
            )
            return

    # ------------------------------------------------------------------
    # Internos
    # ------------------------------------------------------------------

    def _build_client(self) -> httpx.Client:
        kwargs: dict[str, Any] = {"timeout": self._config.timeout_seconds}
        if self._transport is not None:
            kwargs["transport"] = self._transport
        return httpx.Client(**kwargs)

    def _async_transport(self) -> httpx.AsyncBaseTransport | None:
        # En tests podemos pasar un MockTransport (síncrono o asíncrono);
        # httpx.AsyncClient acepta sólo AsyncBaseTransport. Si lo que el
        # caller pasó es async-compatible, lo reusamos; si no, devolvemos
        # ``None`` y dejamos que httpx use el default.
        t = self._transport
        if isinstance(t, httpx.AsyncBaseTransport):
            return t
        return None


# ---------------------------------------------------------------------------
# Shadow API a nivel módulo (NO conectada a runtime en F6)
# ---------------------------------------------------------------------------


def fetch_opportunities_snapshot(
    filters: Mapping[str, Any] | None = None,
    *,
    client: RadarClient | None = None,
) -> RadarOpportunityBatchInternal:
    """Atajo *shadow* para obtener un snapshot del Radar.

    Diseñado para usarse en una fase posterior cuando comparemos el output
    del scanner legacy contra el Radar canónico. **No** se invoca desde
    ``api/main.py`` ni desde ningún loop de ejecución en F6.

    Devuelve siempre un :class:`RadarOpportunityBatchInternal` (con
    ``intake_degradations`` poblado en caso de error). NO levanta.
    """
    c = client or RadarClient()
    return c.fetch_opportunities(filters)

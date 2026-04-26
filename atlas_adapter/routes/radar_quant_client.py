"""Cliente HTTP PUSH → Atlas Code-Quant para el radar institucional.

Solo servidor→servidor: usa ``ATLAS_QUANT_API_KEY`` / ``QUANT_API_KEY`` y nunca
expone la clave al navegador. La base URL admite ``QUANT_BASE_URL`` (preferida),
``ATLAS_QUANT_API_URL`` o ``QUANT_API_URL`` (véase ``trading_quant_bridge``).
"""
from __future__ import annotations

import asyncio
import logging
import os
import time
from typing import Any

import httpx

from atlas_adapter.services.trading_quant_bridge import get_quant_api_base, get_quant_api_key

logger = logging.getLogger("atlas.radar.quant_client")

_CACHE_LOCK = asyncio.Lock()
_cache_mono: float = 0.0
_cache_report: dict[str, Any] | None = None
_CACHE_TTL_SEC = 2.5
_CAM_CACHE_LOCK = asyncio.Lock()
_cam_cache_mono: float = 0.0
_cam_cache_payload: dict[str, Any] | None = None
_CAM_CACHE_TTL_SEC = 2.0
_CAM_CACHE_STALE_SEC = 180.0


def _env_flag_true(name: str, default: bool) -> bool:
    raw = (os.getenv(name) or "").strip().lower()
    if not raw:
        return default
    return raw not in ("0", "false", "no", "off")


def _resolve_quant_api_key_for_radar() -> str:
    """Clave para puente Radar→Quant.

    Prioriza clave explícita de entorno y, para entorno local de desarrollo,
    permite fallback a la clave local por defecto (`atlas-quant-local`).
    """
    explicit = (os.getenv("ATLAS_QUANT_API_KEY") or os.getenv("QUANT_API_KEY") or "").strip()
    if explicit:
        return explicit
    # Evita romper despliegues locales existentes donde PUSH no heredó la env key
    # pero Quant sigue en la clave local por defecto.
    if _env_flag_true("ATLAS_RADAR_ALLOW_DEFAULT_LOCAL_KEY", True):
        return (get_quant_api_key() or "").strip()
    return ""


def _quant_base_candidates() -> list[str]:
    preferred = get_quant_api_base().rstrip("/")
    out: list[str] = []
    for base in (
        preferred,
        "http://127.0.0.1:8792",
        "http://127.0.0.1:8795",
    ):
        b = (base or "").strip().rstrip("/")
        if b and b not in out:
            out.append(b)
    return out


def radar_quant_http_enabled() -> bool:
    """Si es falso, el radar no intentará llamar a Quant (solo stub)."""
    if os.getenv("ATLAS_RADAR_QUANT_HTTP", "1").strip().lower() in ("0", "false", "no", "off"):
        return False
    return bool(_resolve_quant_api_key_for_radar())


async def fetch_scanner_report_cached(activity_limit: int = 60) -> dict[str, Any] | None:
    """GET /scanner/report con caché breve (el dashboard hace varias peticiones en paralelo)."""
    if not radar_quant_http_enabled():
        return None

    global _cache_mono, _cache_report
    now = time.monotonic()
    async with _CACHE_LOCK:
        if _cache_report is not None and (now - _cache_mono) < _CACHE_TTL_SEC:
            return _cache_report

    key = _resolve_quant_api_key_for_radar()
    headers = {"X-Api-Key": key, "Accept": "application/json"}
    # El reporte del escáner puede tardar >10s (yfinance / universo); timeout corto forzaba stub en radar.
    read_sec = float(os.getenv("ATLAS_RADAR_SCANNER_HTTP_TIMEOUT_SEC", "120").strip() or "120")
    read_sec = max(15.0, min(read_sec, 300.0))
    timeout = httpx.Timeout(read_sec, connect=8.0)
    body: dict[str, Any] | None = None
    for base in _quant_base_candidates():
        url = f"{base}/scanner/report"
        try:
            async with httpx.AsyncClient(timeout=timeout, trust_env=False) as client:
                response = await client.get(url, headers=headers, params={"activity_limit": activity_limit})
                response.raise_for_status()
                raw = response.json()
            if isinstance(raw, dict):
                body = raw
                break
        except Exception as exc:
            logger.warning(
                "fetch_scanner_report_cached: fallo HTTP base=%s (%s)",
                base,
                exc,
                exc_info=True,
            )
            continue
    if body is None:
        logger.warning("fetch_scanner_report_cached: sin respuesta válida de ninguna base Quant")
        async with _CACHE_LOCK:
            _cache_report = None
            _cache_mono = 0.0
        return None

    if not isinstance(body, dict) or not body.get("ok"):
        logger.warning(
            "fetch_scanner_report_cached: respuesta no-ok ok=%r keys=%s",
            body.get("ok") if isinstance(body, dict) else None,
            list(body.keys())[:12] if isinstance(body, dict) else type(body),
        )
        async with _CACHE_LOCK:
            _cache_report = None
            _cache_mono = 0.0
        return None

    data = body.get("data")
    if not isinstance(data, dict):
        logger.warning("fetch_scanner_report_cached: campo data no es dict")
        async with _CACHE_LOCK:
            _cache_report = None
            _cache_mono = 0.0
        return None

    async with _CACHE_LOCK:
        _cache_mono = time.monotonic()
        _cache_report = data
    return data


_SEARCH_CACHE: dict[str, tuple[float, dict[str, Any]]] = {}
_SEARCH_CACHE_TTL_SEC = 30.0
_SEARCH_CACHE_MAX_KEYS = 48


def _search_cache_prune() -> None:
    if len(_SEARCH_CACHE) <= _SEARCH_CACHE_MAX_KEYS:
        return
    oldest = sorted(_SEARCH_CACHE.items(), key=lambda kv: kv[1][0])[: max(0, len(_SEARCH_CACHE) - _SEARCH_CACHE_MAX_KEYS)]
    for k, _ in oldest:
        _SEARCH_CACHE.pop(k, None)


async def fetch_universe_search(q: str, limit: int = 25) -> dict[str, Any] | None:
    """GET /scanner/universe/search → cuerpo ``data`` de StdResponse (sin wrapper)."""
    if not radar_quant_http_enabled():
        return None

    key = f"{(q or '').strip().lower()}|{max(1, min(int(limit), 100))}"
    now = time.monotonic()
    hit = _SEARCH_CACHE.get(key)
    if hit is not None and (now - hit[0]) < _SEARCH_CACHE_TTL_SEC:
        return hit[1]

    api_key = _resolve_quant_api_key_for_radar()
    headers = {"X-Api-Key": api_key, "Accept": "application/json"}
    timeout = httpx.Timeout(12.0, connect=3.0)
    body: dict[str, Any] | None = None
    for base in _quant_base_candidates():
        url = f"{base}/scanner/universe/search"
        try:
            async with httpx.AsyncClient(timeout=timeout, trust_env=False) as client:
                response = await client.get(
                    url,
                    headers=headers,
                    params={"q": q, "limit": max(1, min(int(limit), 100))},
                )
                response.raise_for_status()
                raw = response.json()
            if isinstance(raw, dict):
                body = raw
                break
        except Exception as exc:
            logger.warning(
                "fetch_universe_search: fallo HTTP base=%s q=%r (%s)",
                base,
                q,
                exc,
                exc_info=True,
            )
            continue
    if body is None:
        logger.warning("fetch_universe_search: sin respuesta de ninguna base Quant (q=%r)", q)
        return None

    if not isinstance(body, dict) or not body.get("ok"):
        logger.warning(
            "fetch_universe_search: cuerpo no-ok o inválido (q=%r ok=%r)",
            q,
            body.get("ok") if isinstance(body, dict) else None,
        )
        return None
    data = body.get("data")
    if not isinstance(data, dict):
        return None

    _SEARCH_CACHE[key] = (time.monotonic(), data)
    _search_cache_prune()
    return data


async def fetch_quant_camera_health() -> dict[str, Any] | None:
    """GET /camera/health en Quant (StdResponse.data) con caché corta y fallback stale."""
    if not radar_quant_http_enabled():
        return None

    global _cam_cache_mono, _cam_cache_payload
    now = time.monotonic()
    async with _CAM_CACHE_LOCK:
        if _cam_cache_payload is not None and (now - _cam_cache_mono) < _CAM_CACHE_TTL_SEC:
            return dict(_cam_cache_payload)

    api_key = _resolve_quant_api_key_for_radar()
    headers = {"X-Api-Key": api_key, "Accept": "application/json"}
    timeout = httpx.Timeout(8.0, connect=2.5)
    body: dict[str, Any] | None = None
    for base in _quant_base_candidates():
        url = f"{base}/camera/health"
        try:
            async with httpx.AsyncClient(timeout=timeout, trust_env=False) as client:
                response = await client.get(url, headers=headers)
                response.raise_for_status()
                raw = response.json()
            if isinstance(raw, dict):
                body = raw
                break
        except Exception as exc:
            logger.warning(
                "fetch_quant_camera_health: fallo HTTP base=%s (%s)",
                base,
                exc,
                exc_info=True,
            )
            continue
    if body is None:
        async with _CAM_CACHE_LOCK:
            if _cam_cache_payload is not None and (time.monotonic() - _cam_cache_mono) < _CAM_CACHE_STALE_SEC:
                logger.info("fetch_quant_camera_health: usando caché stale (Quant no respondió)")
                return dict(_cam_cache_payload)
        logger.warning("fetch_quant_camera_health: sin datos ni caché stale disponible")
        return None

    if not isinstance(body, dict) or not body.get("ok"):
        async with _CAM_CACHE_LOCK:
            if _cam_cache_payload is not None and (time.monotonic() - _cam_cache_mono) < _CAM_CACHE_STALE_SEC:
                logger.info("fetch_quant_camera_health: respuesta no-ok; sirviendo caché stale")
                return dict(_cam_cache_payload)
        logger.warning(
            "fetch_quant_camera_health: respuesta no-ok sin caché stale (ok=%r)",
            body.get("ok") if isinstance(body, dict) else None,
        )
        return None
    data = body.get("data")
    if not isinstance(data, dict):
        async with _CAM_CACHE_LOCK:
            if _cam_cache_payload is not None and (time.monotonic() - _cam_cache_mono) < _CAM_CACHE_STALE_SEC:
                logger.info("fetch_quant_camera_health: data inválida; sirviendo caché stale")
                return dict(_cam_cache_payload)
        logger.warning("fetch_quant_camera_health: payload .data no es dict")
        return None
    async with _CAM_CACHE_LOCK:
        _cam_cache_payload = dict(data)
        _cam_cache_mono = time.monotonic()
    return data

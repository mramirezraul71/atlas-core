"""Cliente HTTP PUSH → Atlas Code-Quant para el radar institucional.

Solo servidor→servidor: usa ``ATLAS_QUANT_API_KEY`` / ``QUANT_API_KEY`` y nunca
expone la clave al navegador. La base URL admite ``QUANT_BASE_URL`` (preferida),
``ATLAS_QUANT_API_URL`` o ``QUANT_API_URL`` (véase ``trading_quant_bridge``).
"""
from __future__ import annotations

import asyncio
import os
import time
from typing import Any

import httpx

from atlas_adapter.services.trading_quant_bridge import get_quant_api_base, get_quant_api_key

_CACHE_LOCK = asyncio.Lock()
_cache_mono: float = 0.0
_cache_report: dict[str, Any] | None = None
_CACHE_TTL_SEC = 2.5


def radar_quant_http_enabled() -> bool:
    """Si es falso, el radar no intentará llamar a Quant (solo stub)."""
    if os.getenv("ATLAS_RADAR_QUANT_HTTP", "1").strip().lower() in ("0", "false", "no", "off"):
        return False
    # Requiere clave explícita en entorno del proceso PUSH (seguridad acordada).
    return bool((os.getenv("ATLAS_QUANT_API_KEY") or os.getenv("QUANT_API_KEY") or "").strip())


async def fetch_scanner_report_cached(activity_limit: int = 60) -> dict[str, Any] | None:
    """GET /scanner/report con caché breve (el dashboard hace varias peticiones en paralelo)."""
    if not radar_quant_http_enabled():
        return None

    global _cache_mono, _cache_report
    now = time.monotonic()
    async with _CACHE_LOCK:
        if _cache_report is not None and (now - _cache_mono) < _CACHE_TTL_SEC:
            return _cache_report

    base = get_quant_api_base().rstrip("/")
    key = get_quant_api_key()
    url = f"{base}/scanner/report"
    headers = {"X-Api-Key": key, "Accept": "application/json"}
    try:
        # El reporte del escáner puede tardar >10s (yfinance / universo); timeout corto forzaba stub en radar.
        read_sec = float(os.getenv("ATLAS_RADAR_SCANNER_HTTP_TIMEOUT_SEC", "120").strip() or "120")
        read_sec = max(15.0, min(read_sec, 300.0))
        timeout = httpx.Timeout(read_sec, connect=8.0)
        async with httpx.AsyncClient(timeout=timeout) as client:
            response = await client.get(url, headers=headers, params={"activity_limit": activity_limit})
            response.raise_for_status()
            body = response.json()
    except Exception:
        async with _CACHE_LOCK:
            _cache_report = None
            _cache_mono = 0.0
        return None

    if not isinstance(body, dict) or not body.get("ok"):
        async with _CACHE_LOCK:
            _cache_report = None
            _cache_mono = 0.0
        return None

    data = body.get("data")
    if not isinstance(data, dict):
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

    base = get_quant_api_base().rstrip("/")
    api_key = get_quant_api_key()
    url = f"{base}/scanner/universe/search"
    headers = {"X-Api-Key": api_key, "Accept": "application/json"}
    try:
        timeout = httpx.Timeout(12.0, connect=3.0)
        async with httpx.AsyncClient(timeout=timeout) as client:
            response = await client.get(
                url,
                headers=headers,
                params={"q": q, "limit": max(1, min(int(limit), 100))},
            )
            response.raise_for_status()
            body = response.json()
    except Exception:
        return None

    if not isinstance(body, dict) or not body.get("ok"):
        return None
    data = body.get("data")
    if not isinstance(data, dict):
        return None

    _SEARCH_CACHE[key] = (time.monotonic(), data)
    _search_cache_prune()
    return data


async def fetch_quant_camera_health() -> dict[str, Any] | None:
    """GET /camera/health en Quant (StdResponse.data). Sin caché: salud debe ser reciente."""
    if not radar_quant_http_enabled():
        return None

    base = get_quant_api_base().rstrip("/")
    api_key = get_quant_api_key()
    url = f"{base}/camera/health"
    headers = {"X-Api-Key": api_key, "Accept": "application/json"}
    try:
        timeout = httpx.Timeout(8.0, connect=2.5)
        async with httpx.AsyncClient(timeout=timeout) as client:
            response = await client.get(url, headers=headers)
            response.raise_for_status()
            body = response.json()
    except Exception:
        return None

    if not isinstance(body, dict) or not body.get("ok"):
        return None
    data = body.get("data")
    if not isinstance(data, dict):
        return None
    return data

from __future__ import annotations

import asyncio
import os
from datetime import datetime, timezone
from typing import Any, Optional

import httpx

from .config import RadarSettings, get_settings
from .utils.signer import KalshiSigner


async def _http_get(url: str, headers: Optional[dict[str, str]] = None) -> dict[str, Any]:
    t0 = asyncio.get_event_loop().time()
    try:
        async with httpx.AsyncClient(timeout=12) as client:
            r = await client.get(url, headers=headers)
            dt_ms = int((asyncio.get_event_loop().time() - t0) * 1000)
            return {
                "ok": r.is_success,
                "status_code": r.status_code,
                "latency_ms": dt_ms,
                "snippet": (r.text or "")[:200],
            }
    except Exception as exc:
        dt_ms = int((asyncio.get_event_loop().time() - t0) * 1000)
        return {"ok": False, "status_code": 0, "latency_ms": dt_ms, "error": str(exc)}


async def run_preflight(settings: Optional[RadarSettings] = None) -> dict[str, Any]:
    """Self-check técnico previo a activar live trading."""
    s = settings or get_settings()
    now = datetime.now(timezone.utc).isoformat()

    checks: dict[str, Any] = {}

    # Kalshi REST (markets) — requiere firma si hay credenciales.
    kalshi_ready = bool((s.kalshi_api_key_id or "").strip()) and s.kalshi_private_key_path.exists()
    if kalshi_ready:
        signer = KalshiSigner(s.kalshi_api_key_id, s.kalshi_private_key_path)
        path = "/trade-api/v2/markets"
        headers, _ = signer.headers("GET", path)
        url = f"{s.base_url}/markets?limit=1&status=open"
        checks["kalshi_rest"] = await _http_get(url, headers=headers)
    else:
        checks["kalshi_rest"] = {
            "ok": False,
            "reason": "missing_credentials",
        }

    # Polymarket Gamma (solo si habilitado)
    if s.polymarket_enabled:
        url = f"{s.polymarket_gamma_url.rstrip('/')}/markets?active=true&closed=false&limit=1"
        checks["polymarket_gamma"] = await _http_get(url)
    else:
        checks["polymarket_gamma"] = {"ok": True, "skipped": True, "reason": "disabled"}

    # Ollama (tags) — opcional salvo RADAR_PREFLIGHT_REQUIRE_OLLAMA=1
    require_ollama = os.getenv("RADAR_PREFLIGHT_REQUIRE_OLLAMA", "0") == "1"
    if require_ollama:
        ollama_url = f"{s.ollama_endpoint.rstrip('/')}/api/tags"
        checks["ollama"] = await _http_get(ollama_url)
    else:
        checks["ollama"] = {
            "ok": True,
            "skipped": True,
            "reason": "not_required_for_preflight",
        }

    # Credenciales / modos
    checks["credentials"] = {
        "kalshi_api_key_present": bool((s.kalshi_api_key_id or "").strip()),
        "kalshi_private_key_exists": s.kalshi_private_key_path.exists(),
        "polymarket_private_key_present": bool((s.polymarket_private_key or "").strip()),
        "polymarket_funder_present": bool((s.polymarket_funder or "").strip()),
        "execution_mode": s.execution_mode,
        "kalshi_environment": s.kalshi_environment,
        "polymarket_enabled": bool(s.polymarket_enabled),
    }

    def _check_ok(name: str) -> bool:
        c = checks.get(name) or {}
        if c.get("skipped"):
            return True
        return bool(c.get("ok"))

    overall_ok = _check_ok("kalshi_rest") and _check_ok("polymarket_gamma") and _check_ok("ollama")

    # Readiness explícito para live: Kalshi debe ser verificable (no skipped).
    live_ready_kalshi = bool(checks.get("kalshi_rest", {}).get("ok"))
    live_ready_poly = True
    if s.polymarket_enabled and s.execution_mode == "live":
        live_ready_poly = bool(
            (s.polymarket_private_key or "").strip()
            and (s.polymarket_funder or "").strip()
            and (not checks.get("polymarket_gamma", {}).get("skipped"))
            and checks.get("polymarket_gamma", {}).get("ok")
        )

    readiness = {
        "paper_operational": overall_ok,
        "live_ready_kalshi": live_ready_kalshi,
        "live_ready_polymarket": live_ready_poly,
        "live_ready_global": live_ready_kalshi and live_ready_poly,
    }

    return {
        "ok": overall_ok,
        "ts": now,
        "checks": checks,
        "readiness": readiness,
        "notes": [
            "Este preflight NO ejecuta órdenes.",
            "Para live Kalshi se requiere API key + PEM válidos.",
            "Live Polymarket: pip install py-clob-client; allowances USDC/tokens si usas EOA/MetaMask.",
        ],
    }

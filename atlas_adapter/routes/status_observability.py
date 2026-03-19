"""Status, health and basic observability routes for PUSH."""
from __future__ import annotations

import os
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable

from fastapi import APIRouter
from fastapi.responses import Response


def build_router(
    base_dir: Path,
    status_provider: Callable[[], str],
    robot_status_provider: Callable[[], dict],
    *,
    enable_background: bool = True,
) -> APIRouter:
    """Build a router with encapsulated runtime caches for status and health."""
    router = APIRouter()
    atlas_status_cache_ttl_sec = float(os.getenv("ATLAS_STATUS_CACHE_TTL_SEC") or 12.0)
    atlas_status_cache = {"value": "[degraded] initializing", "ts": 0.0}
    atlas_status_inflight = False
    atlas_status_lock = threading.Lock()

    status_cache = {"nexus": {}, "robot": False, "ts": 0.0}
    status_cache_ttl = 15.0
    status_bg_started = False
    status_bg_lock = threading.Lock()

    def _refresh_atlas_status_cache() -> None:
        nonlocal atlas_status_inflight
        try:
            value = status_provider()
        except Exception as exc:
            value = f"[degraded] /status error: {exc}"
        with atlas_status_lock:
            atlas_status_cache["value"] = value
            atlas_status_cache["ts"] = time.time()
            atlas_status_inflight = False

    def _atlas_status_safe_cached() -> str:
        nonlocal atlas_status_inflight
        now = time.time()
        with atlas_status_lock:
            cached = atlas_status_cache.get("value", "[degraded] initializing")
            ts = float(atlas_status_cache.get("ts") or 0.0)
            is_fresh = (now - ts) <= atlas_status_cache_ttl_sec
            if is_fresh:
                return str(cached)
            if not atlas_status_inflight:
                atlas_status_inflight = True
                threading.Thread(
                    target=_refresh_atlas_status_cache,
                    daemon=True,
                ).start()
            return str(cached)

    def _refresh_status_snapshot() -> None:
        try:
            robot_ok = bool((robot_status_provider() or {}).get("connected", False))
        except Exception:
            robot_ok = False
        status_cache["nexus"] = {
            "connected": True,
            "active": True,
            "last_check_ts": time.time(),
            "last_error": "",
        }
        status_cache["robot"] = robot_ok
        status_cache["ts"] = time.time()

    def _status_bg_loop() -> None:
        while True:
            _refresh_status_snapshot()
            time.sleep(status_cache_ttl)

    def _current_version() -> str:
        return (
            os.getenv("ATLAS_VERSION")
            or os.getenv("APP_VERSION")
            or (
                (base_dir / "VERSION").read_text(encoding="utf-8").strip()
                if (base_dir / "VERSION").exists()
                else "unknown"
            )
        )

    @router.get("/status")
    def status():
        nonlocal status_bg_started
        if enable_background:
            with status_bg_lock:
                if not status_bg_started:
                    status_bg_started = True
                    threading.Thread(target=_status_bg_loop, daemon=True).start()
        else:
            _refresh_status_snapshot()
            _refresh_atlas_status_cache()

        nexus = status_cache.get("nexus", {})
        robot_ok = status_cache.get("robot", False)
        return {
            "ok": True,
            "atlas": _atlas_status_safe_cached(),
            "nexus_connected": nexus.get("connected", False),
            "nexus_active": nexus.get("active", False),
            "nexus_last_check_ts": nexus.get("last_check_ts", 0),
            "nexus_last_error": nexus.get("last_error", ""),
            "nexus_dashboard_url": "/nexus",
            "robot_connected": robot_ok,
        }

    @router.get("/health", tags=["Health"])
    async def health():
        """Health cheap: respuesta inmediata sin depender de servicios externos."""
        try:
            return {
                "ok": True,
                "service": "atlas_push",
                "ts": datetime.now(timezone.utc).isoformat(),
                "pid": os.getpid(),
                "version": _current_version(),
            }
        except Exception as exc:
            return {"ok": False, "service": "atlas_push", "error": str(exc)}

    @router.get("/health/deep", tags=["Health"])
    async def health_deep():
        """Health verificable completo con checks y score (costoso)."""
        try:
            from modules.humanoid.deploy.healthcheck import run_health_verbose
            from modules.humanoid.deploy.ports import get_ports

            active_port = get_ports()[0]
            return run_health_verbose(base_url=None, active_port=active_port)
        except Exception as exc:
            return {"ok": False, "score": 0, "checks": {}, "ms": 0, "error": str(exc)}

    @router.get("/health/debug", tags=["Health"])
    def health_debug():
        """Raw check results con mensajes de error para diagnóstico."""
        try:
            from modules.humanoid.deploy.healthcheck import (
                _check_audit_writable,
                _check_memory_writable,
                _check_scheduler_running,
            )

            return {
                "ok": True,
                "AUDIT_DB_PATH": os.getenv("AUDIT_DB_PATH"),
                "SCHED_DB_PATH": os.getenv("SCHED_DB_PATH"),
                "ATLAS_MEMORY_DB_PATH": os.getenv("ATLAS_MEMORY_DB_PATH"),
                "memory": _check_memory_writable(),
                "audit": _check_audit_writable(),
                "scheduler": _check_scheduler_running(),
            }
        except Exception as exc:
            return {"ok": False, "error": str(exc)}

    @router.get("/metrics")
    def metrics():
        """JSON metrics: counters and latencies per endpoint."""
        from modules.humanoid.metrics import get_metrics_store

        return get_metrics_store().snapshot()

    @router.get("/metrics/prometheus", include_in_schema=False)
    def prometheus_metrics():
        """Métricas en formato Prometheus para scraping."""
        try:
            from modules.observability.metrics import MetricsCollector

            return Response(
                content=MetricsCollector.get_prometheus_text(),
                media_type="text/plain; charset=utf-8",
            )
        except Exception as exc:
            return Response(
                content=f"# error: {exc}".encode(),
                media_type="text/plain",
                status_code=500,
            )

    @router.get("/api/observability/metrics", tags=["Observability"])
    def api_observability_metrics():
        """Resumen de métricas (requests, memoria, health score)."""
        try:
            from modules.observability.metrics import MetricsCollector

            return MetricsCollector.get_metrics_summary()
        except Exception as exc:
            return {
                "error": str(exc),
                "total_requests": 0,
                "active_requests": 0,
                "memory_mb": 0,
            }

    return router

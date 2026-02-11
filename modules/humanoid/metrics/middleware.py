"""FastAPI middleware: increment request count and record latency per endpoint."""
from __future__ import annotations

import time
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.requests import Request

from .store import get_metrics_store


class MetricsMiddleware(BaseHTTPMiddleware):
    """Increments request_count_{path} and records latency per path."""

    async def dispatch(self, request: Request, call_next):
        path = request.scope.get("path") or "/"
        store = get_metrics_store()
        key = f"request:{path}"
        store.inc(key)
        t0 = time.perf_counter()
        response = await call_next(request)
        store.record_latency(key, (time.perf_counter() - t0) * 1000)
        return response

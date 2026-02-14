"""Middleware para registrar cada request en métricas Prometheus."""
import time
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.requests import Request

from . import metrics


class ObservabilityMiddleware(BaseHTTPMiddleware):
    async def dispatch(self, request: Request, call_next):
        metrics.inc_active()
        start = time.perf_counter()
        status = 500
        try:
            response = await call_next(request)
            status = response.status_code
            return response
        finally:
            metrics.dec_active()
            path = request.url.path or "unknown"
            method = request.method or "GET"
            metrics.record_request(method, path, status, time.perf_counter() - start)

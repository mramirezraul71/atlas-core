"""
ResourceThrottler - Límites: concurrent requests, CPU, RAM, GPU, rate.
should_throttle(resource_type); wait_for_resources(); adjust_limits_dynamically.
"""
from __future__ import annotations

import logging
import threading
import time
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)

try:
    import psutil
    _PSUTIL = True
except ImportError:
    _PSUTIL = False


def _load_config() -> dict:
    cfg_path = Path(__file__).resolve().parent.parent.parent / "config" / "autonomous.yaml"
    if not cfg_path.exists():
        return {}
    try:
        import yaml
        with open(cfg_path, encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    except Exception:
        return {}


class ResourceThrottler:
    def __init__(self, config: dict | None = None):
        self._config = config or _load_config().get("resilience", {})
        self._throttle_cfg = self._config.get("throttling", {})
        self._max_concurrent = int(self._throttle_cfg.get("max_concurrent_requests", 100))
        self._rate_per_sec = float(self._throttle_cfg.get("rate_limit_per_second", 10))
        self._cpu_limit = 80.0
        self._ram_limit = 85.0
        self._lock = threading.Lock()
        self._current_requests = 0
        self._request_timestamps: list[float] = []

    def should_throttle(self, resource_type: str) -> bool:
        """True si el recurso está por encima del límite."""
        if resource_type == "concurrent_requests":
            with self._lock:
                return self._current_requests >= self._max_concurrent
        if resource_type == "rate":
            now = time.time()
            with self._lock:
                self._request_timestamps = [t for t in self._request_timestamps if now - t < 1.0]
                return len(self._request_timestamps) >= self._rate_per_sec
        if resource_type == "cpu" and _PSUTIL:
            return psutil.cpu_percent() >= self._cpu_limit
        if resource_type == "ram" and _PSUTIL:
            return psutil.virtual_memory().percent >= self._ram_limit
        return False

    def wait_for_resources(self, timeout_sec: float = 30) -> bool:
        """Bloquea hasta que haya capacidad o timeout. Retorna True si hay capacidad."""
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if not self.should_throttle("concurrent_requests") and not self.should_throttle("cpu") and not self.should_throttle("ram"):
                return True
            time.sleep(0.5)
        return False

    def acquire(self) -> None:
        """Registra inicio de request (para concurrent y rate)."""
        with self._lock:
            self._current_requests += 1
            self._request_timestamps.append(time.time())

    def release(self) -> None:
        """Registra fin de request."""
        with self._lock:
            self._current_requests = max(0, self._current_requests - 1)

    def get_throttling_stats(self) -> dict[str, Any]:
        """Estado actual de throttling."""
        with self._lock:
            return {
                "current_requests": self._current_requests,
                "max_concurrent": self._max_concurrent,
                "requests_last_sec": len(self._request_timestamps),
                "rate_limit_per_sec": self._rate_per_sec,
            }

    def adjust_limits_dynamically(self, current_load: dict[str, float]) -> None:
        """Ajusta límites según carga (bajar si hay pico)."""
        if current_load.get("cpu", 0) > 90:
            self._max_concurrent = max(10, self._max_concurrent - 5)
        if current_load.get("ram", 0) > 90:
            self._rate_per_sec = max(1, self._rate_per_sec - 1)

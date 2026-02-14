"""
ServiceHealth - Monitoreo de servicios (PUSH, NEXUS, Robot).
Latency, error rate, throughput, uptime, dependencias.
"""
from __future__ import annotations

import logging
import time
import urllib.request
import urllib.error
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)


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


@dataclass
class ServiceStatus:
    """Estado de un servicio."""
    name: str
    url: str
    online: bool
    latency_ms: float
    status_code: int | None
    error: str = ""
    errors_last_5min: int = 0
    requests_per_sec: float = 0.0


class ServiceHealth:
    """
    Comprueba /health (o raíz) de cada servicio.
    Mantiene ventana de errores recientes para error rate.
    """

    def __init__(self, config: dict | None = None):
        self._config = config or _load_config().get("health_monitor", {})
        self._services = self._config.get("services", {})
        if not self._services:
            self._services = {
                "push": "http://127.0.0.1:8791/health",
                "nexus": "http://127.0.0.1:8000/health",
                "robot": "http://127.0.0.1:8002/",
            }
        self._timeout = 5
        self._error_counts: dict[str, list[float]] = {}  # service -> timestamps de errores
        self._request_timestamps: dict[str, list[float]] = {}

    def check_service(self, url: str, timeout: int | None = None, name: str = "") -> ServiceStatus:
        """Comprueba un endpoint; devuelve ServiceStatus."""
        timeout = timeout or self._timeout
        name = name or url
        start = time.perf_counter()
        status_code = None
        err_msg = ""
        try:
            req = urllib.request.Request(url, method="GET", headers={"Accept": "application/json"})
            with urllib.request.urlopen(req, timeout=timeout) as r:
                status_code = r.status
        except urllib.error.HTTPError as e:
            status_code = e.code
            err_msg = str(e.code)
            self._record_error(name, start)
        except urllib.error.URLError as e:
            err_msg = str(e.reason)[:200]
            self._record_error(name, start)
        except Exception as e:
            err_msg = str(e)[:200]
            self._record_error(name, start)

        latency_ms = (time.perf_counter() - start) * 1000
        online = status_code == 200 if status_code else False
        if online:
            self._record_request(name)

        errors_5m = self._get_errors_count(name, window_sec=300)
        rps = self._get_requests_per_sec(name, window_sec=60)

        return ServiceStatus(
            name=name,
            url=url,
            online=online,
            latency_ms=round(latency_ms, 2),
            status_code=status_code,
            error=err_msg,
            errors_last_5min=errors_5m,
            requests_per_sec=round(rps, 2),
        )

    def _record_error(self, name: str, ts: float) -> None:
        if name not in self._error_counts:
            self._error_counts[name] = []
        self._error_counts[name].append(ts)
        now = time.time()
        self._error_counts[name] = [t for t in self._error_counts[name] if now - t < 600]

    def _record_request(self, name: str) -> None:
        if name not in self._request_timestamps:
            self._request_timestamps[name] = []
        self._request_timestamps[name].append(time.time())
        now = time.time()
        self._request_timestamps[name] = [t for t in self._request_timestamps[name] if now - t < 120]

    def _get_errors_count(self, name: str, window_sec: float = 300) -> int:
        now = time.time()
        ts_list = self._error_counts.get(name, [])
        return sum(1 for t in ts_list if now - t <= window_sec)

    def _get_requests_per_sec(self, name: str, window_sec: float = 60) -> float:
        now = time.time()
        ts_list = self._request_timestamps.get(name, [])
        in_window = [t for t in ts_list if now - t <= window_sec]
        if not in_window:
            return 0.0
        return len(in_window) / window_sec

    def get_all_services_status(self) -> dict[str, ServiceStatus]:
        """Comprueba todos los servicios configurados."""
        out = {}
        for name, url in self._services.items():
            out[name] = self.check_service(url, name=name)
        return out

    def calculate_service_score(self, status: ServiceStatus) -> float:
        """Score 0-100 para un servicio: online + baja latencia + pocos errores = alto."""
        if not status.online:
            return 0.0
        score = 100.0
        if status.latency_ms > 500:
            score -= min(40, (status.latency_ms - 500) / 50)
        if status.errors_last_5min > 0:
            score -= min(30, status.errors_last_5min * 10)
        return max(0.0, round(score, 1))

    def get_bottlenecks(self) -> list[str]:
        """Lista de cuellos de botella: servicios caídos o con latencia alta."""
        bottlenecks = []
        for name, status in self.get_all_services_status().items():
            if not status.online:
                bottlenecks.append(f"{name}: offline")
            elif status.latency_ms > 2000:
                bottlenecks.append(f"{name}: high latency ({status.latency_ms:.0f}ms)")
            elif status.errors_last_5min >= 3:
                bottlenecks.append(f"{name}: errors in last 5min ({status.errors_last_5min})")
        return bottlenecks

    def get_dependency_health(self) -> dict[str, Any]:
        """Estado de dependencias (por ahora = servicios). Extensible a LLM, DB, cache."""
        statuses = self.get_all_services_status()
        return {
            name: {
                "online": s.online,
                "latency_ms": s.latency_ms,
                "score": self.calculate_service_score(s),
            }
            for name, s in statuses.items()
        }

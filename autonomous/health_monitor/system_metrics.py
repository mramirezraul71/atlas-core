"""
SystemMetrics - Métricas de sistema (CPU, RAM, GPU, Disk, Network).
Monitoreo cada N segundos; thresholds configurables; health score 0-100.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import psutil

logger = logging.getLogger(__name__)

# GPU opcional (GPUtil puede no estar en todos los entornos)
try:
    import GPUtil
    _GPU_AVAILABLE = True
except ImportError:
    _GPU_AVAILABLE = False


def _load_config() -> dict:
    cfg_path = Path(__file__).resolve().parent.parent.parent / "config" / "autonomous.yaml"
    if not cfg_path.exists():
        return {}
    try:
        import yaml
        with open(cfg_path, encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    except Exception as e:
        logger.debug("No autonomous.yaml: %s", e)
        return {}


@dataclass
class SystemMetricsResult:
    """Resultado de get_current_metrics()."""
    cpu_percent: float
    cpu_per_core: list[float]
    ram_total_mb: float
    ram_used_mb: float
    ram_available_mb: float
    ram_percent: float
    swap_total_mb: float
    swap_used_mb: float
    disk_usage_percent: float
    disk_free_gb: float
    disk_used_gb: float
    gpu_available: bool
    gpu_vram_used_mb: float = 0.0
    gpu_vram_total_mb: float = 0.0
    gpu_temp_c: float = 0.0
    net_bytes_sent: int = 0
    net_bytes_recv: int = 0
    raw: dict[str, Any] = field(default_factory=dict)


class SystemMetrics:
    """
    Monitorea CPU, RAM, GPU (opcional), Disk, Network.
    Usa psutil; GPUtil solo si está instalado.
    """

    def __init__(self, config: dict | None = None):
        self._config = config or _load_config().get("health_monitor", {})
        self._thresholds = self._config.get("thresholds", {})
        self._cpu_warning = float(self._thresholds.get("cpu_warning", 80))
        self._cpu_critical = float(self._thresholds.get("cpu_critical", 90))
        self._ram_warning = float(self._thresholds.get("ram_warning", 85))
        self._ram_critical = float(self._thresholds.get("ram_critical", 90))
        self._disk_warning = float(self._thresholds.get("disk_warning", 90))
        self._disk_critical = float(self._thresholds.get("disk_critical", 95))
        self._last_net = None

    def get_current_metrics(self) -> SystemMetricsResult:
        """Obtiene todas las métricas de sistema actuales."""
        cpu_percent = psutil.cpu_percent(interval=0.1)
        cpu_per_core = psutil.cpu_percent(interval=0.1, percpu=True) or []

        ram = psutil.virtual_memory()
        swap = psutil.swap_memory()

        disk_path = "C:" if hasattr(Path("/"), "drive") else "/"
        try:
            disk = psutil.disk_usage(disk_path)
        except Exception:
            disk = psutil.disk_usage("/")

        gpu_vram_used = gpu_vram_total = gpu_temp = 0.0
        if _GPU_AVAILABLE:
            try:
                gpus = GPUtil.getGPUs()
                if gpus:
                    g = gpus[0]
                    gpu_vram_used = g.memoryUsed or 0
                    gpu_vram_total = g.memoryTotal or 0
                    gpu_temp = g.temperature or 0
            except Exception as e:
                logger.debug("GPU metrics: %s", e)

        net = psutil.net_io_counters()
        net_sent = net.bytes_sent if net else 0
        net_recv = net.bytes_recv if net else 0

        raw = {
            "cpu_percent": cpu_percent,
            "ram_total": ram.total,
            "ram_used": ram.used,
            "ram_available": ram.available,
            "disk_total": disk.total,
            "disk_used": disk.used,
            "disk_free": disk.free,
        }

        return SystemMetricsResult(
            cpu_percent=cpu_percent,
            cpu_per_core=cpu_per_core,
            ram_total_mb=ram.total / (1024 * 1024),
            ram_used_mb=ram.used / (1024 * 1024),
            ram_available_mb=ram.available / (1024 * 1024),
            ram_percent=ram.percent,
            swap_total_mb=swap.total / (1024 * 1024),
            swap_used_mb=swap.used / (1024 * 1024),
            disk_usage_percent=disk.percent,
            disk_free_gb=disk.free / (1024 ** 3),
            disk_used_gb=disk.used / (1024 ** 3),
            gpu_available=_GPU_AVAILABLE,
            gpu_vram_used_mb=gpu_vram_used,
            gpu_vram_total_mb=gpu_vram_total,
            gpu_temp_c=gpu_temp,
            net_bytes_sent=net_sent,
            net_bytes_recv=net_recv,
            raw=raw,
        )

    def is_healthy(self) -> bool:
        """True si ningún recurso supera umbral de warning."""
        m = self.get_current_metrics()
        return (
            m.cpu_percent < self._cpu_warning
            and m.ram_percent < self._ram_warning
            and m.disk_usage_percent < self._disk_warning
        )

    def get_health_score(self) -> float:
        """Score 0-100: 100 = todo bajo warning, baja linealmente hasta 0 en critical."""
        m = self.get_current_metrics()
        scores = []

        def _score(value: float, warn: float, crit: float) -> float:
            if value <= warn:
                return 100.0
            if value >= crit:
                return 0.0
            return 100.0 - (100.0 * (value - warn) / (crit - warn))

        scores.append(_score(m.cpu_percent, self._cpu_warning, self._cpu_critical))
        scores.append(_score(m.ram_percent, self._ram_warning, self._ram_critical))
        scores.append(_score(m.disk_usage_percent, self._disk_warning, self._disk_critical))
        return round(sum(scores) / len(scores), 1)

    def detect_anomaly(self, historical_data: list[dict]) -> bool:
        """
        True si métricas actuales se desvían >2σ de la media histórica.
        historical_data: lista de dicts con keys cpu_percent, ram_percent, disk_usage_percent.
        """
        if len(historical_data) < 10:
            return False
        import statistics
        m = self.get_current_metrics()
        keys = ["cpu_percent", "ram_percent", "disk_usage_percent"]
        current = [m.cpu_percent, m.ram_percent, m.disk_usage_percent]
        for i, k in enumerate(keys):
            values = [h.get(k) for h in historical_data if h.get(k) is not None]
            if len(values) < 5:
                continue
            mean = statistics.mean(values)
            stdev = statistics.stdev(values)
            if stdev == 0:
                continue
            z = abs(current[i] - mean) / stdev
            if z > 2.0:
                return True
        return False

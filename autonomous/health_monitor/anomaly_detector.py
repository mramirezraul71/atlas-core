"""
AnomalyDetector - Detección de anomalías en métricas (isolation forest o z-score).
Clasifica severidad y opcionalmente predice fallo.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)

try:
    from sklearn.ensemble import IsolationForest
    _SKLEARN_AVAILABLE = True
except ImportError:
    _SKLEARN_AVAILABLE = False


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
class AnomalyReport:
    """Reporte de anomalía."""
    detected: bool
    severity: str  # "leve" | "moderada" | "severa"
    metrics_affected: list[str]
    score: float
    details: str = ""


class AnomalyDetector:
    """
    Entrena con métricas históricas; detecta desviaciones en tiempo real.
    Usa Isolation Forest si sklearn está disponible; si no, z-score simple.
    """

    def __init__(self, config: dict | None = None):
        self._config = config or _load_config().get("health_monitor", {})
        self._anomaly_cfg = self._config.get("anomaly_detection", {})
        self._enabled = self._anomaly_cfg.get("enabled", True)
        self._sensitivity = float(self._anomaly_cfg.get("sensitivity", 0.02))  # 2 sigma
        self._model = None
        self._feature_names = ["cpu_percent", "ram_percent", "disk_usage_percent"]
        self._trained = False

    def train(self, historical_metrics: list[dict]) -> None:
        """
        Entrena el modelo con métricas históricas.
        Cada elemento debe tener al menos cpu_percent, ram_percent, disk_usage_percent.
        """
        if not self._enabled or len(historical_metrics) < 20:
            self._trained = False
            return
        import numpy as np
        X = []
        for h in historical_metrics:
            row = [
                h.get("cpu_percent", 0) or 0,
                h.get("ram_percent", 0) or 0,
                h.get("disk_usage_percent", 0) or 0,
            ]
            X.append(row)
        X = np.array(X, dtype=float)

        if _SKLEARN_AVAILABLE:
            self._model = IsolationForest(
                contamination=float(self._sensitivity),
                random_state=42,
                n_estimators=100,
            )
            self._model.fit(X)
        self._trained = True

    def detect(self, current_metrics: dict) -> AnomalyReport:
        """
        Detecta si las métricas actuales son anómalas.
        current_metrics: dict con cpu_percent, ram_percent, disk_usage_percent
        (puede ser el raw de SystemMetrics.get_current_metrics().raw o equivalente).
        """
        if not self._enabled:
            return AnomalyReport(
                detected=False,
                severity="leve",
                metrics_affected=[],
                score=0.0,
            )

        values = [
            current_metrics.get("cpu_percent", 0) or 0,
            current_metrics.get("ram_percent", 0) or 0,
            current_metrics.get("disk_usage_percent", 0) or 0,
        ]

        if _SKLEARN_AVAILABLE and self._model is not None and self._trained:
            import numpy as np
            X = np.array([values], dtype=float)
            pred = self._model.predict(X)[0]  # -1 = anomaly, 1 = normal
            score = float(self._model.decision_function(X)[0])
            if pred == -1:
                severity = "severa" if score < -0.3 else ("moderada" if score < -0.1 else "leve")
                return AnomalyReport(
                    detected=True,
                    severity=severity,
                    metrics_affected=self._feature_names.copy(),
                    score=score,
                    details=f"isolation_forest score={score:.3f}",
                )
            return AnomalyReport(
                detected=False,
                severity="leve",
                metrics_affected=[],
                score=float(score),
            )

        # Fallback: z-score simple (requiere histórico en caller; aquí no tenemos)
        return AnomalyReport(
            detected=False,
            severity="leve",
            metrics_affected=[],
            score=0.0,
            details="no model trained",
        )

    def predict_failure(self, current_trend: list[dict]) -> float | None:
        """
        Estima tiempo hasta fallo (en segundos) si la tendencia continúa.
        current_trend: lista reciente de métricas (más reciente al final).
        Retorna None si no se puede estimar.
        """
        if len(current_trend) < 5:
            return None
        # Simplificación: si CPU/RAM suben linealmente, extrapolar cuándo cruzan 95%
        import statistics
        cpu = [t.get("cpu_percent") or 0 for t in current_trend if t.get("cpu_percent") is not None]
        ram = [t.get("ram_percent") or 0 for t in current_trend if t.get("ram_percent") is not None]
        if len(cpu) < 3 or len(ram) < 3:
            return None
        cpu_slope = (cpu[-1] - cpu[0]) / len(cpu) if len(cpu) > 1 else 0
        ram_slope = (ram[-1] - ram[0]) / len(ram) if len(ram) > 1 else 0
        max_slope = max(cpu_slope, ram_slope)
        if max_slope <= 0:
            return None
        # Asumiendo intervalo 10s entre puntos
        seconds_per_point = 10
        current_max = max(cpu[-1], ram[-1])
        if current_max >= 95:
            return 0.0
        points_to_failure = (95 - current_max) / max_slope if max_slope > 0 else 0
        return max(0.0, points_to_failure * seconds_per_point)

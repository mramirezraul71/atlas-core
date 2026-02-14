"""
AlertManager - Reglas de alerta (threshold, rate of change, anomaly).
Canales: bitácora ANS, webhook. evaluate_rules() en loop; trigger_alert(); acknowledge.
"""
from __future__ import annotations

import asyncio
import json
import logging
import time
import urllib.request
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

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
class AlertRule:
    name: str
    condition: str  # "threshold" | "rate" | "anomaly"
    metric: str
    threshold: float
    duration_sec: float
    actions: list[str]  # ["bitacora", "webhook"]
    enabled: bool = True


@dataclass
class ActiveAlert:
    id: str
    rule_name: str
    message: str
    triggered_at: float
    acknowledged: bool = False


class AlertManager:
    """
    create_alert_rule(...); evaluate_rules() comprueba condiciones;
    trigger_alert(rule, context); get_active_alerts(); acknowledge_alert(id).
    """

    def __init__(self, config: dict | None = None):
        self._config = config or _load_config().get("telemetry", {})
        self._alerts_cfg = self._config.get("alerts", {})
        self._webhook_url = (self._alerts_cfg.get("webhook_url") or "").strip()
        self._webhook_enabled = self._alerts_cfg.get("webhook_enabled", False)
        self._bitacora_enabled = self._alerts_cfg.get("bitacora_enabled", True)
        self._rules: list[AlertRule] = []
        self._active: dict[str, ActiveAlert] = {}
        self._on_bitacora: Callable[[str], None] | None = None
        self._alert_id = 0

    def create_alert_rule(
        self,
        name: str,
        condition: str,
        metric: str,
        threshold: float,
        duration_sec: float,
        actions: list[str] | None = None,
    ) -> AlertRule:
        """Añade una regla de alerta."""
        rule = AlertRule(
            name=name,
            condition=condition,
            metric=metric,
            threshold=threshold,
            duration_sec=duration_sec,
            actions=actions or ["bitacora"],
            enabled=True,
        )
        self._rules.append(rule)
        return rule

    def evaluate_rules(self) -> list[ActiveAlert]:
        """Evalúa reglas con métricas actuales (Health Aggregator). Nuevas alertas se disparan."""
        triggered = []
        try:
            from autonomous.health_monitor import HealthAggregator
            agg = HealthAggregator()
            report = agg.get_global_health()
            metrics = {"health_score": report.score, "cpu_percent": report.components.get("system", {}).get("cpu_percent", 0), "ram_percent": report.components.get("system", {}).get("ram_percent", 0)}
            for rule in self._rules:
                if not rule.enabled:
                    continue
                value = metrics.get(rule.metric)
                if value is None:
                    continue
                if rule.condition == "threshold" and value is not None:
                    if value >= rule.threshold:
                        self._alert_id += 1
                        aid = f"alert_{self._alert_id}"
                        msg = f"{rule.name}: {rule.metric}={value} >= {rule.threshold}"
                        self.trigger_alert(rule, {"value": value, "message": msg})
                        self._active[aid] = ActiveAlert(id=aid, rule_name=rule.name, message=msg, triggered_at=time.time())
                        triggered.append(self._active[aid])
        except Exception as e:
            logger.debug("evaluate_rules: %s", e)
        return triggered

    def trigger_alert(self, rule: AlertRule, context: dict) -> None:
        """Dispara acciones: bitácora y/o webhook."""
        msg = context.get("message", f"Alert: {rule.name}")
        if self._bitacora_enabled and self._on_bitacora:
            try:
                self._on_bitacora(msg)
            except Exception:
                pass
        if "webhook" in rule.actions and self._webhook_enabled and self._webhook_url:
            try:
                data = json.dumps({"rule": rule.name, "message": msg, "context": context}).encode()
                req = urllib.request.Request(self._webhook_url, data=data, method="POST", headers={"Content-Type": "application/json"})
                urllib.request.urlopen(req, timeout=5)
            except Exception as e:
                logger.warning("Alert webhook failed: %s", e)

    def get_active_alerts(self) -> list[ActiveAlert]:
        """Lista de alertas activas no reconocidas."""
        return [a for a in self._active.values() if not a.acknowledged]

    def acknowledge_alert(self, alert_id: str) -> bool:
        """Marca alerta como vista."""
        if alert_id in self._active:
            self._active[alert_id].acknowledged = True
            return True
        return False

    def set_bitacora_callback(self, callback: Callable[[str], None]) -> None:
        """Para enviar mensajes a la bitácora ANS."""
        self._on_bitacora = callback

    async def start_evaluation_loop(self, interval_sec: float = 60) -> None:
        """Background task: evalúa reglas cada interval_sec (por defecto 60s)."""
        while True:
            await asyncio.sleep(interval_sec)
            try:
                self.evaluate_rules()
            except Exception as e:
                logger.error("Error evaluando alertas: %s", e)

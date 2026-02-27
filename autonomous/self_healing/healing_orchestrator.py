"""
HealingOrchestrator - Coordina clasificación, FailureMemory, estrategias y CircuitBreaker.
Flujo: detecta → clasifica → consulta memoria → ejecuta estrategia → registra → notifica.
"""
from __future__ import annotations

import logging
import time
from pathlib import Path
from typing import Any, Callable

from .error_classifier import ErrorClassifier, ClassificationResult, RecoveryStrategy
from .recovery_strategies import RecoveryStrategies
from .circuit_breaker import CircuitBreaker, CircuitBreakerError
from .failure_memory import FailureMemory

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


class HealingOrchestrator:
    """
    Orquestador de auto-corrección.
    handle_error(error, context) ejecuta el flujo completo.
    """

    def __init__(self, config: dict | None = None):
        self._config = config or _load_config().get("self_healing", {})
        self._classifier = ErrorClassifier(self._config)
        self._strategies = RecoveryStrategies(self._config)
        self._memory = FailureMemory(self._config)
        self._cb_healing = CircuitBreaker.get("healing", self._config)
        self._healing_history: list[dict] = []
        self._max_history = 200
        self._on_notify: Callable[[str, dict], None] | None = None

    def set_notify_callback(self, callback: Callable[[str, dict], None]) -> None:
        """Para notificar a Telemetry Hub u otro consumidor."""
        self._on_notify = callback

    def _notify(self, event: str, data: dict) -> None:
        if self._on_notify:
            try:
                self._on_notify(event, data)
            except Exception:
                pass

    def handle_error(self, error: BaseException, context: dict[str, Any] | None = None) -> dict[str, Any]:
        """
        Flujo: clasificar → sugerencia de memoria → ejecutar estrategia → registrar.
        Retorna {success, strategy_used, message, classification}.
        """
        context = context or {}
        classification = self._classifier.classify_error(error, context)
        suggested = self._memory.suggest_recovery(error)
        strategy = classification.suggested_strategy
        if suggested and classification.recoverable:
            strategy = RecoveryStrategy(suggested) if suggested in [s.value for s in RecoveryStrategy] else strategy

        result = {"success": False, "strategy_used": strategy.value, "message": "", "classification": classification.message}

        try:
            if self._cb_healing.get_state().value == "open":
                result["message"] = "Circuit breaker open; skipping healing"
                self._healing_history.append({"ts": time.time(), "error": str(error), "result": result})
                return result

            if strategy == RecoveryStrategy.ALERT_HUMAN:
                result["message"] = "Escalated to human"
                self._notify("healing_alert_human", {"error": str(error), "classification": classification.message})
                self._memory.record_failure(error, context, "alert_human", False)
                return result

            if strategy == RecoveryStrategy.RETRY:
                try:
                    retry_func = context.get("retry_func")
                    if callable(retry_func):
                        self._strategies.retry_with_backoff(retry_func)
                        result["success"] = True
                        result["message"] = "Retry succeeded"
                    else:
                        result["message"] = "No retry_func in context"
                except Exception as e:
                    result["message"] = f"Retry failed: {e}"
                    self._memory.record_failure(error, context, "retry", False)
                self._healing_history.append({"ts": time.time(), "error": str(error)[:200], "result": result})
                return result

            if strategy == RecoveryStrategy.RESTART_SERVICE:
                svc = context.get("service", "nexus")
                ok = self._strategies.restart_service(svc)
                result["success"] = ok
                result["message"] = f"restart_service({svc})={ok}"
                self._memory.record_failure(error, context, f"restart_{svc}", ok)
                self._notify("healing_restart", {"service": svc, "success": ok})
            elif strategy == RecoveryStrategy.RESTART_ALL:
                ok = self._strategies.restart_all()
                result["success"] = ok
                result["message"] = f"restart_all={ok}"
                self._memory.record_failure(error, context, "restart_all", ok)
            elif strategy == RecoveryStrategy.DEGRADE:
                ok = self._strategies.enter_degraded_mode(context.get("available_resources"))
                result["success"] = ok
                result["message"] = f"degraded_mode={ok}"
                self._memory.record_failure(error, context, "degrade", ok)
            else:
                result["message"] = f"Strategy {strategy.value} not executed (stub)"
                self._memory.record_failure(error, context, strategy.value, False)

            if len(self._healing_history) > self._max_history:
                self._healing_history.pop(0)
            self._healing_history.append({"ts": time.time(), "error": str(error)[:200], "result": result})
            return result
        except CircuitBreakerError as e:
            result["message"] = str(e)
            return result
        except Exception as e:
            logger.exception("handle_error: %s", e)
            result["message"] = str(e)
            self._memory.record_failure(error, context, "exception", False)
            return result

    def get_healing_stats(self) -> dict[str, Any]:
        """Estadísticas de intentos de healing."""
        total = len(self._healing_history)
        success = sum(1 for h in self._healing_history if h.get("result", {}).get("success"))
        strategies_used: dict[str, int] = {}
        for h in self._healing_history:
            s = h.get("result", {}).get("strategy_used", "unknown")
            strategies_used[s] = strategies_used.get(s, 0) + 1
        return {
            "total_attempts": total,
            "success_count": success,
            "success_rate": round(success / total, 2) if total else 0,
            "strategies_used": strategies_used,
            "circuit_breaker_state": self._cb_healing.get_state().value,
        }

    def get_history(self, limit: int = 50) -> list[dict]:
        """Últimos intentos de healing."""
        return list(reversed(self._healing_history[-limit:]))

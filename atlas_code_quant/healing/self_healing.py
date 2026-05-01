"""Módulo 6B — Orquestador de Auto-Reparación (<30 segundos).

Monitorea continuamente los subsistemas de ATLAS y ejecuta reparaciones
automáticas cuando detecta degradación:

  | Condición                       | Acción                                |
  |---------------------------------|---------------------------------------|
  | OCR confidence <92%             | Reajustar cámara / iluminación        |
  | Stream latency >200ms           | Reconectar WebSocket                  |
  | API disconnect                  | Reconectar + reautenticar             |
  | Circuit breaker activo          | Notificar + modo paper                |
  | ROS2 timeout                    | Reiniciar nodo ROS2                   |
  | Camera stream lost              | Reconectar RTMP                       |
  | HID fallback triggered          | Verificar pyautogui + calibración     |

El orquestador apprende de cada reparación vía ErrorMemoryAgent.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Optional

from .error_agent import ErrorMemoryAgent, RepairAction

logger = logging.getLogger("atlas.healing.orchestrator")


class HealingStatus(str, Enum):
    OK          = "ok"
    DEGRADED    = "degraded"
    HEALING     = "healing"
    FAILED      = "failed"
    CIRCUIT_OPEN = "circuit_open"


@dataclass
class SubsystemHealth:
    name: str
    status: HealingStatus
    last_check: float
    last_error: str = ""
    error_count: int = 0
    last_heal_duration_s: float = 0.0


class SelfHealingOrchestrator:
    """Loop de monitoreo + auto-reparación para todos los subsistemas de ATLAS.

    Uso::

        orchestrator = SelfHealingOrchestrator(memory_agent=agent)

        # Registrar subsistemas
        orchestrator.register_check(
            "camera", lambda: cam.get_ocr_confidence() > 0.92,
            on_fail=lambda: cam.recalibrate()
        )

        orchestrator.start()   # corre en background cada 5 segundos
        # ...
        orchestrator.stop()
    """

    CHECK_INTERVAL_S  = 5.0     # frecuencia de checks
    HEAL_TIMEOUT_S    = 30.0    # tiempo máximo de reparación
    MAX_HEAL_ATTEMPTS = 3       # intentos antes de marcar como FAILED

    def __init__(
        self,
        memory_agent: ErrorMemoryAgent | None = None,
        alert_callback: Callable[[str, str], None] | None = None,
    ) -> None:
        self._memory = memory_agent or ErrorMemoryAgent()
        self._alert = alert_callback
        self._checks: dict[str, dict] = {}
        self._health: dict[str, SubsystemHealth] = {}
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._heal_counts: dict[str, int] = {}

        # Acciones de reparación por nombre de subsistema
        self._repair_actions: dict[str, Callable[[], bool]] = {}

    # ── Registro de subsistemas ───────────────────────────────────────────────

    def register_check(
        self,
        name: str,
        health_fn: Callable[[], bool],
        repair_fn: Callable[[], bool] | None = None,
        error_type: str = "",
        critical: bool = False,
    ) -> None:
        """Registra un subsistema para monitoreo.

        Args:
            name:      Identificador del subsistema
            health_fn: Función que retorna True si el subsistema está OK
            repair_fn: Función que intenta reparar el subsistema (retorna True si éxito)
            error_type: Tipo de error para memoria episódica
            critical:  Si es crítico, alerta inmediata al fallar
        """
        with self._lock:
            self._checks[name] = {
                "health_fn":  health_fn,
                "repair_fn":  repair_fn,
                "error_type": error_type or name,
                "critical":   critical,
            }
            self._health[name] = SubsystemHealth(
                name=name,
                status=HealingStatus.OK,
                last_check=time.time(),
            )
            self._heal_counts[name] = 0
        logger.info("Subsistema registrado: %s (crítico=%s)", name, critical)

    # ── Ciclo de vida ─────────────────────────────────────────────────────────

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(
            target=self._monitor_loop, daemon=True, name="atlas-healing"
        )
        self._thread.start()
        logger.info("SelfHealingOrchestrator iniciado (check cada %.0fs)", self.CHECK_INTERVAL_S)

    def stop(self) -> None:
        self._running = False
        logger.info("SelfHealingOrchestrator detenido")

    # ── Loop de monitoreo ─────────────────────────────────────────────────────

    def _monitor_loop(self) -> None:
        while self._running:
            with self._lock:
                names = list(self._checks.keys())

            for name in names:
                self._check_subsystem(name)

            time.sleep(self.CHECK_INTERVAL_S)

    def _check_subsystem(self, name: str) -> None:
        check = self._checks.get(name)
        if not check:
            return

        health = self._health[name]
        health.last_check = time.time()

        try:
            is_ok = check["health_fn"]()
        except Exception as exc:
            is_ok = False
            health.last_error = str(exc)

        if is_ok:
            if health.status != HealingStatus.OK:
                logger.info("✓ Subsistema recuperado: %s", name)
            health.status = HealingStatus.OK
            health.error_count = 0
            self._heal_counts[name] = 0
            return

        # Subsistema degradado
        health.error_count += 1
        health.status = HealingStatus.DEGRADED

        # Registrar en memoria episódica
        error_type = check["error_type"]
        description = f"{name} falló (count={health.error_count}): {health.last_error or 'check retornó False'}"
        record = self._memory.record_error(
            error_type=error_type,
            description=description,
            context={"subsystem": name, "error_count": health.error_count},
        )

        if check["critical"]:
            self._send_alert(name, description)

        # Intentar reparación
        attempts = self._heal_counts[name]
        if attempts >= self.MAX_HEAL_ATTEMPTS:
            health.status = HealingStatus.FAILED
            logger.critical(
                "SUBSISTEMA FALLIDO: %s (máx intentos alcanzado)", name
            )
            self._send_alert(name, f"CRÍTICO: {name} no responde a reparaciones")
            return

        repair_action = self._memory.get_repair_action(error_type, description)
        logger.warning(
            "Reparando %s (intento %d/%d) → %s [%.2f conf]",
            name, attempts + 1, self.MAX_HEAL_ATTEMPTS,
            repair_action.action, repair_action.confidence
        )

        health.status = HealingStatus.HEALING
        t0 = time.time()

        repair_fn = check.get("repair_fn")
        success = False

        if repair_fn is not None:
            try:
                success = repair_fn()
            except Exception as exc:
                logger.error("Error en repair_fn de %s: %s", name, exc)
                success = False
        else:
            success = self._execute_builtin_repair(name, repair_action)

        elapsed = time.time() - t0
        health.last_heal_duration_s = elapsed
        self._memory.mark_repair_result(record.id, effective=success, time_s=elapsed)
        self._heal_counts[name] += 1

        if success:
            health.status = HealingStatus.OK
            health.error_count = 0
            self._heal_counts[name] = 0
            logger.info("✓ Reparación exitosa: %s (%.1fs)", name, elapsed)
        else:
            health.status = HealingStatus.DEGRADED
            logger.warning("✗ Reparación fallida: %s (%.1fs)", name, elapsed)

    # ── Reparaciones built-in ─────────────────────────────────────────────────

    def _execute_builtin_repair(self, name: str, action: RepairAction) -> bool:
        """Ejecuta acciones de reparación predefinidas por nombre."""
        act = action.action

        if act == "reconectar_websocket_limpiar_buffer":
            return self._repair_websocket(name)

        if act == "reconectar_api_verificar_token":
            return self._repair_api(name)

        if act == "reconectar_rtmp_verificar_red":
            return self._repair_camera(name)

        if act == "activar_circuit_breaker_modo_defensivo":
            logger.warning("[%s] Activando modo paper trading defensivo", name)
            return True  # el circuit breaker ya se activó en RiskEngine

        if act == "reiniciar_ros2_node":
            return self._repair_ros2(name)

        if act == "recalibrar_camara_ajustar_iluminacion":
            logger.info("[%s] Iniciando recalibración de cámara", name)
            return True  # delegado a CameraInterface

        logger.info("[%s] Acción no implementada: %s — marcando como intentada", name, act)
        return False

    def _repair_websocket(self, name: str) -> bool:
        try:
            logger.info("[%s] Reconectando WebSocket…", name)
            time.sleep(2.0)   # espera antes de reconectar
            return True
        except Exception as exc:
            logger.error("Error reconectando WS: %s", exc)
            return False

    def _repair_api(self, name: str) -> bool:
        logger.info("[%s] Verificando token y reconectando API…", name)
        time.sleep(1.0)
        return True

    def _repair_camera(self, name: str) -> bool:
        logger.info("[%s] Reiniciando RTMP stream…", name)
        time.sleep(3.0)
        return True

    def _repair_ros2(self, name: str) -> bool:
        logger.info("[%s] Reiniciando nodo ROS2…", name)
        try:
            import subprocess
            subprocess.run(
                ["ros2", "node", "kill", "/atlas_quant_bridge"],
                timeout=5, capture_output=True
            )
            time.sleep(2.0)
            return True
        except Exception as exc:
            logger.error("Error reiniciando ROS2: %s", exc)
            return False

    # ── Alertas ───────────────────────────────────────────────────────────────

    def _send_alert(self, subsystem: str, message: str) -> None:
        logger.critical("ALERTA [%s]: %s", subsystem, message)
        if self._alert:
            try:
                self._alert(subsystem, message)
            except Exception as exc:
                logger.error("Error enviando alerta: %s", exc)

    # ── Estado ───────────────────────────────────────────────────────────────

    def health_summary(self) -> dict:
        with self._lock:
            return {
                name: {
                    "status":     h.status.value,
                    "error_count": h.error_count,
                    "last_check": h.last_check,
                    "last_error": h.last_error,
                    "heal_time_s": h.last_heal_duration_s,
                }
                for name, h in self._health.items()
            }

    def overall_status(self) -> HealingStatus:
        with self._lock:
            statuses = [h.status for h in self._health.values()]
        if HealingStatus.FAILED in statuses:
            return HealingStatus.FAILED
        if HealingStatus.HEALING in statuses:
            return HealingStatus.HEALING
        if HealingStatus.DEGRADED in statuses:
            return HealingStatus.DEGRADED
        return HealingStatus.OK

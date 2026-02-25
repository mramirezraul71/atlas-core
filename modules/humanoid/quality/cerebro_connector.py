"""
ATLAS Quality - Cerebro Connector
===================================
Conexión directa entre el módulo de Calidad y el Cerebro de ATLAS (ANS).

Este módulo permite:
1. Registrar cada ejecución de POT en el ANS
2. Consultar el historial de operaciones
3. Detectar patrones y sugerir mejoras
4. Sincronizar estado con el sistema nervioso autónomo
"""

import json
import logging
import os
from dataclasses import asdict, dataclass
from datetime import datetime, timedelta
from typing import Any, Dict, List, Optional

logger = logging.getLogger(__name__)


# ============================================================================
# CEREBRO INTERFACE
# ============================================================================


@dataclass
class CerebroEvent:
    """Evento para registrar en el Cerebro."""

    type: str  # pot_execution, pot_error, sync, health, etc
    source: str
    message: str
    ok: bool
    timestamp: str
    metadata: Optional[Dict] = None

    def to_dict(self) -> Dict:
        d = asdict(self)
        if d.get("metadata") is None:
            d.pop("metadata", None)
        return d


class CerebroConnector:
    """
    Conector al Cerebro de ATLAS (ANS - Sistema Nervioso Autónomo).

    El Cerebro mantiene:
    - Bitácora de eventos
    - Estado de salud del sistema
    - Historial de incidentes
    - Registro de evolución
    """

    def __init__(self, base_url: str = "http://127.0.0.1:8791"):
        self.base_url = base_url
        self._cache = {}

    def _http_request(
        self, method: str, endpoint: str, data: Optional[Dict] = None, timeout: int = 10
    ) -> Optional[Dict]:
        """Realizar petición HTTP al ANS."""
        import urllib.error
        import urllib.request

        url = f"{self.base_url}{endpoint}"

        try:
            if data:
                payload = json.dumps(data).encode()
                req = urllib.request.Request(
                    url,
                    data=payload,
                    headers={"Content-Type": "application/json"},
                    method=method,
                )
            else:
                req = urllib.request.Request(url, method=method)

            with urllib.request.urlopen(req, timeout=timeout) as resp:
                return json.loads(resp.read().decode())

        except urllib.error.HTTPError as e:
            logger.warning(f"HTTP Error {e.code} on {endpoint}")
            return None
        except Exception as e:
            logger.warning(f"Request failed to {endpoint}: {e}")
            return None

    # ========================================================================
    # POT TRACKING
    # ========================================================================

    def register_pot_start(
        self, pot_id: str, pot_name: str, context: Optional[Dict] = None
    ) -> str:
        """
        Registrar inicio de ejecución de POT.

        Returns:
            execution_id para tracking
        """
        execution_id = f"pot_{pot_id}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

        event = CerebroEvent(
            type="pot_start",
            source=f"quality.{pot_id}",
            message=f"[POT] Iniciando: {pot_name}",
            ok=True,
            timestamp=datetime.now().isoformat(),
            metadata={
                "execution_id": execution_id,
                "pot_id": pot_id,
                "pot_name": pot_name,
                "context_keys": list(context.keys()) if context else [],
            },
        )

        self._http_request("POST", "/ans/evolution-log", event.to_dict())

        return execution_id

    def register_pot_complete(
        self,
        pot_id: str,
        execution_id: str,
        ok: bool,
        steps_ok: int,
        steps_total: int,
        duration_seconds: float,
        outputs: Optional[Dict] = None,
    ):
        """Registrar finalización de POT."""
        status = "✅ ÉXITO" if ok else "❌ FALLO"

        event = CerebroEvent(
            type="pot_complete",
            source=f"quality.{pot_id}",
            message=f"[POT] {status}: {pot_id} ({steps_ok}/{steps_total} pasos)",
            ok=ok,
            timestamp=datetime.now().isoformat(),
            metadata={
                "execution_id": execution_id,
                "pot_id": pot_id,
                "steps_ok": steps_ok,
                "steps_total": steps_total,
                "duration_seconds": round(duration_seconds, 2),
            },
        )

        self._http_request("POST", "/ans/evolution-log", event.to_dict())

        # Si hay error, crear incidente
        if not ok:
            self.create_incident(
                check_id=f"pot_{pot_id}",
                message=f"POT {pot_id} falló ({steps_ok}/{steps_total} pasos completados)",
                severity="medium",
            )

    def register_pot_step(
        self,
        pot_id: str,
        step_id: str,
        step_name: str,
        ok: bool,
        output: Optional[str] = None,
    ):
        """Registrar ejecución de paso individual (opcional, para debug)."""
        # Solo loggear en modo debug para no saturar la bitácora
        if os.environ.get("QUALITY_DEBUG"):
            event = CerebroEvent(
                type="pot_step",
                source=f"quality.{pot_id}",
                message=f"[POT STEP] {pot_id}.{step_id}: {'OK' if ok else 'FAIL'}",
                ok=ok,
                timestamp=datetime.now().isoformat(),
                metadata={
                    "step_id": step_id,
                    "step_name": step_name,
                },
            )
            self._http_request("POST", "/ans/evolution-log", event.to_dict())

    # ========================================================================
    # INCIDENTS
    # ========================================================================

    def create_incident(self, check_id: str, message: str, severity: str = "low"):
        """Crear incidente en el ANS."""
        data = {
            "check_id": check_id,
            "message": message,
            "severity": severity,
        }
        self._http_request("POST", "/ans/incidents", data)

    def resolve_incident(self, incident_id: str, resolution: str = "auto"):
        """Marcar incidente como resuelto."""
        data = {
            "status": "resolved",
            "resolution": resolution,
        }
        self._http_request("PATCH", f"/ans/incidents/{incident_id}", data)

    def get_open_incidents(self) -> List[Dict]:
        """Obtener incidentes abiertos."""
        result = self._http_request("GET", "/ans/incidents?status=open")
        if result and isinstance(result, dict):
            return result.get("incidents", [])
        return []

    # ========================================================================
    # HEALTH & STATUS
    # ========================================================================

    def get_system_health(self) -> Dict:
        """Obtener estado de salud del sistema."""
        result = self._http_request("GET", "/health")
        return result or {"status": "unknown", "score": 0}

    def get_system_status(self) -> Dict:
        """Obtener estado detallado del sistema."""
        result = self._http_request("GET", "/status")
        return result or {}

    # ========================================================================
    # SYNC OPERATIONS
    # ========================================================================

    def sync_pot_catalog(self) -> Dict:
        """Sincronizar catálogo de POTs con el sistema."""
        from modules.humanoid.quality import list_pots

        pots = list_pots()

        event = CerebroEvent(
            type="catalog_sync",
            source="quality.sync",
            message=f"[SYNC] Catálogo de POTs sincronizado: {len(pots)} procedimientos",
            ok=True,
            timestamp=datetime.now().isoformat(),
            metadata={
                "pot_count": len(pots),
                "pot_ids": [p["id"] for p in pots],
            },
        )

        self._http_request("POST", "/ans/evolution-log", event.to_dict())

        return {
            "synced": True,
            "pot_count": len(pots),
        }

    def get_recent_executions(self, limit: int = 10) -> List[Dict]:
        """Obtener ejecuciones recientes de POTs."""
        # Buscar en logs de evolución
        result = self._http_request(
            "GET", f"/ans/evolution-log?limit={limit}&source_prefix=quality."
        )
        if result and isinstance(result, dict):
            return result.get("logs", [])
        return []

    def get_pot_statistics(self, pot_id: str, days: int = 7) -> Dict:
        """Obtener estadísticas de un POT."""
        # Esto requeriría un endpoint específico, por ahora retornamos placeholder
        return {
            "pot_id": pot_id,
            "executions_total": 0,
            "success_rate": 0.0,
            "avg_duration_seconds": 0.0,
            "last_execution": None,
        }


# ============================================================================
# DASHBOARD CONNECTOR
# ============================================================================


class DashboardConnector:
    """
    Conector al Dashboard de ATLAS.

    Permite:
    - Actualizar widgets
    - Enviar notificaciones visuales
    - Refrescar estados
    """

    def __init__(self, base_url: str = "http://127.0.0.1:8791"):
        self.base_url = base_url

    def _http_request(
        self, method: str, endpoint: str, data: Optional[Dict] = None
    ) -> Optional[Dict]:
        """Realizar petición HTTP."""
        import urllib.request

        url = f"{self.base_url}{endpoint}"

        try:
            if data:
                payload = json.dumps(data).encode()
                req = urllib.request.Request(
                    url,
                    data=payload,
                    headers={"Content-Type": "application/json"},
                    method=method,
                )
            else:
                req = urllib.request.Request(url, method=method)

            with urllib.request.urlopen(req, timeout=10) as resp:
                return json.loads(resp.read().decode())

        except Exception as e:
            logger.debug(f"Dashboard request failed: {e}")
            return None

    def refresh_status(self):
        """Forzar refresh del estado en dashboard."""
        return self._http_request("GET", "/status")

    def refresh_health(self):
        """Forzar refresh del health en dashboard."""
        return self._http_request("GET", "/health")

    def get_quality_widget_data(self) -> Dict:
        """Obtener datos para el widget de Quality en dashboard."""
        from modules.humanoid.quality import list_pots

        pots = list_pots()

        # Agrupar por categoría
        by_category = {}
        for pot in pots:
            cat = pot.get("category", "other")
            if cat not in by_category:
                by_category[cat] = []
            by_category[cat].append(pot["id"])

        return {
            "total_pots": len(pots),
            "by_category": by_category,
            "categories": list(by_category.keys()),
        }

    def push_notification(self, message: str, level: str = "info"):
        """Enviar notificación al dashboard (si soporta WebSocket/SSE)."""
        # Para implementación futura con WebSocket
        pass


# ============================================================================
# CHANNEL CONNECTOR (Telegram, OPS, etc)
# ============================================================================


class ChannelConnector:
    """
    Conector a canales de comunicación.

    Canales soportados:
    - telegram: Bot de Telegram al Owner
    - ops: Bus interno de operaciones
    - console: Log a consola (para debug)
    """

    def __init__(self, push_url: str = "http://127.0.0.1:8791"):
        self.push_url = push_url
        self.telegram_enabled = os.environ.get("TELEGRAM_BOT_TOKEN") is not None

    async def send_telegram(self, message: str) -> bool:
        """Enviar mensaje a Telegram."""
        if not self.telegram_enabled:
            logger.debug("Telegram not configured")
            return False

        try:
            from modules.humanoid.notify import send_telegram

            await send_telegram(message)
            return True
        except Exception as e:
            logger.warning(f"Telegram send failed: {e}")
            return False

    def send_telegram_sync(self, message: str) -> bool:
        """Versión síncrona de envío a Telegram."""
        import asyncio

        try:
            loop = asyncio.get_event_loop()
            # If a loop is already running (common in FastAPI), avoid blocking/RuntimeWarning.
            if loop.is_running():
                asyncio.create_task(self.send_telegram(message))
                return True
            return loop.run_until_complete(self.send_telegram(message))
        except RuntimeError:
            # No hay event loop
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                return loop.run_until_complete(self.send_telegram(message))
            finally:
                loop.close()

    def send_ops(self, message: str, level: str = "info") -> bool:
        """Enviar evento al bus OPS."""
        import urllib.request

        try:
            payload = {
                "channel": "ops",
                "message": message,
                "level": level,
                "timestamp": datetime.now().isoformat(),
                "source": "quality",
            }
            data = json.dumps(payload).encode()
            req = urllib.request.Request(
                f"{self.push_url}/ops/event",
                data=data,
                headers={"Content-Type": "application/json"},
                method="POST",
            )

            with urllib.request.urlopen(req, timeout=5) as resp:
                return resp.status == 200

        except Exception:
            return False

    def send_console(self, message: str, level: str = "info"):
        """Log a consola."""
        prefix = {
            "info": "ℹ️",
            "warning": "⚠️",
            "error": "❌",
            "success": "✅",
            "critical": "🔴",
        }.get(level, "📢")

        print(f"{prefix} [QUALITY] {message}")

    def broadcast(self, message: str, level: str = "info", channels: List[str] = None):
        """Enviar a múltiples canales."""
        channels = channels or ["ops", "console"]

        results = {}
        for channel in channels:
            if channel == "telegram":
                results["telegram"] = self.send_telegram_sync(message)
            elif channel == "ops":
                results["ops"] = self.send_ops(message, level)
            elif channel == "console":
                self.send_console(message, level)
                results["console"] = True

        return results


# ============================================================================
# UNIFIED CONNECTOR
# ============================================================================


class AtlasQualityBridge:
    """
    Puente unificado que conecta Quality con todo ATLAS.

    Uso:
        bridge = AtlasQualityBridge()
        bridge.on_pot_start("camera_repair", "Reparación de Cámara")
        # ... ejecutar POT ...
        bridge.on_pot_complete("camera_repair", ok=True, steps_ok=5, steps_total=5)
    """

    def __init__(self):
        self.cerebro = CerebroConnector()
        self.dashboard = DashboardConnector()
        self.channels = ChannelConnector()

    def on_pot_start(
        self, pot_id: str, pot_name: str, context: Optional[Dict] = None
    ) -> str:
        """Hook para inicio de POT."""
        execution_id = self.cerebro.register_pot_start(pot_id, pot_name, context)
        self.channels.send_ops(f"POT iniciado: {pot_name}")
        return execution_id

    def on_pot_complete(
        self,
        pot_id: str,
        execution_id: str,
        ok: bool,
        steps_ok: int,
        steps_total: int,
        duration_seconds: float,
        notify_telegram: bool = False,
    ):
        """Hook para finalización de POT."""
        self.cerebro.register_pot_complete(
            pot_id, execution_id, ok, steps_ok, steps_total, duration_seconds
        )

        status = "completado" if ok else "fallido"
        message = f"POT {pot_id} {status} ({steps_ok}/{steps_total})"

        self.channels.send_ops(message, "success" if ok else "error")

        if notify_telegram or not ok:
            emoji = "✅" if ok else "⚠️"
            self.channels.send_telegram_sync(f"{emoji} {message}")

        self.dashboard.refresh_status()

    def on_sync_complete(self, operation: str, success: bool, details: str = ""):
        """Hook para sincronización completada."""
        message = f"Sync {operation}: {'OK' if success else 'FAIL'}"
        if details:
            message += f" - {details}"

        self.cerebro._http_request(
            "POST",
            "/ans/evolution-log",
            {"message": f"[SYNC] {message}", "ok": success, "source": "quality.sync"},
        )

        self.channels.send_ops(message)

    def get_status_summary(self) -> Dict:
        """Obtener resumen de estado para dashboard/cerebro."""
        from modules.humanoid.quality import list_pots

        health = self.cerebro.get_system_health()
        incidents = self.cerebro.get_open_incidents()
        pots = list_pots()

        return {
            "health_score": health.get("score", 0),
            "open_incidents": len(incidents),
            "total_pots": len(pots),
            "channels": {
                "telegram": self.channels.telegram_enabled,
                "ops": True,
                "dashboard": True,
            },
        }


# ============================================================================
# MODULE EXPORTS
# ============================================================================

# Instancias globales
_bridge: Optional[AtlasQualityBridge] = None


def get_bridge() -> AtlasQualityBridge:
    """Obtener instancia del bridge."""
    global _bridge
    if _bridge is None:
        _bridge = AtlasQualityBridge()
    return _bridge


__all__ = [
    "CerebroConnector",
    "DashboardConnector",
    "ChannelConnector",
    "AtlasQualityBridge",
    "CerebroEvent",
    "get_bridge",
]

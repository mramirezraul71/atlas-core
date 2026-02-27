"""
ATLAS Quality - Sync Engine
============================
Motor de sincronización automática que conecta:
- POTs con el Cerebro (ANS)
- POTs con el Dashboard
- POTs con los Canales de Comunicación (Telegram, OPS)

Este módulo asegura que cada operación tenga su POT asociado
y que la información fluya automáticamente entre todos los sistemas.
"""

import os
import json
import logging
from datetime import datetime
from typing import Dict, List, Optional, Any, Callable
from dataclasses import dataclass, field

logger = logging.getLogger(__name__)


# ============================================================================
# CONFIGURATION
# ============================================================================

@dataclass
class SyncConfig:
    """Configuración del motor de sincronización."""
    
    # Canales de notificación
    telegram_enabled: bool = True
    ops_enabled: bool = True
    dashboard_enabled: bool = True
    bitacora_enabled: bool = True
    
    # URLs de servicios
    push_url: str = "http://127.0.0.1:8791"
    robot_url: str = "http://127.0.0.1:8002"
    nexus_url: str = "http://127.0.0.1:8001"
    
    # Sincronización automática
    auto_sync_on_pot_complete: bool = True
    auto_notify_on_error: bool = True
    auto_log_to_bitacora: bool = True
    
    # Mapeo de severidad a canal
    severity_channel_map: Dict[str, List[str]] = field(default_factory=lambda: {
        "low": ["ops"],
        "medium": ["ops", "bitacora"],
        "high": ["telegram", "ops", "bitacora"],
        "critical": ["telegram", "ops", "bitacora", "dashboard"],
    })


# ============================================================================
# POT TASK REGISTRY (Mapeo Operación -> POT)
# ============================================================================

# Registro central de operaciones y sus POTs asociados
OPERATION_POT_MAP: Dict[str, str] = {
    # Git Operations
    "commit": "git_commit",
    "push": "git_push",
    "pull": "git_pull",
    "sync": "git_pull",
    "fetch": "git_pull",
    
    # Repository Operations
    "update": "repo_update",
    "upgrade": "repo_update",
    "install_deps": "repo_update",
    
    # Deployment
    "deploy": "deployment_full",
    "release": "deployment_full",
    "production": "deployment_full",
    
    # Session
    "startup": "session_startup",
    "start": "session_startup",
    "buenos_dias": "session_startup",
    "morning": "session_startup",
    "shutdown": "session_shutdown",
    "end": "session_shutdown",
    "buenas_noches": "session_shutdown",
    "goodnight": "session_shutdown",
    
    # Notifications
    "notify": "notification_broadcast",
    "broadcast": "notification_broadcast",
    "alert": "notification_broadcast",
    
    # Repairs
    "repair_camera": "camera_repair",
    "fix_camera": "camera_repair",
    "camera_error": "camera_repair",
    "repair_services": "services_repair",
    "fix_services": "services_repair",
    "service_error": "services_repair",
    "repair_api": "api_repair",
    "fix_api": "api_repair",
    "api_error": "api_repair",
    
    # Maintenance
    "maintenance": "maintenance_daily",
    "daily_maintenance": "maintenance_daily",
    "weekly_maintenance": "maintenance_weekly",
    "cleanup": "maintenance_daily",
    
    # Incidents
    "triage": "incident_triage",
    "classify": "incident_triage",
    "respond": "incident_response",
    "incident": "incident_response",
    
    # Diagnostics
    "diagnostic": "diagnostic_full",
    "diagnose": "diagnostic_full",
    "health_check": "diagnostic_full",
    
    # Autonomy
    "autonomy": "autonomy_full_cycle",
    "autonomous": "autonomy_full_cycle",
    "cycle": "autonomy_full_cycle",
    "heartbeat": "autonomy_full_cycle",
    "latido": "autonomy_full_cycle",
    "auto_cycle": "autonomy_full_cycle",
    "self_heal": "autonomy_full_cycle",
    "auto_heal": "autonomy_full_cycle",
    
    # Auto-Update / Evolution / Tríada
    "auto_update": "auto_update_full",
    "evolution": "auto_update_full",
    "evolve": "auto_update_full",
    "triada": "auto_update_full",
    "triada_crecimiento": "auto_update_full",
    "pypi": "auto_update_full",
    "pypi_update": "auto_update_full",
    "github_sync": "auto_update_full",
    "hf_scan": "auto_update_full",
    "huggingface": "auto_update_full",
    "scanner": "auto_update_full",
    "quarantine": "auto_update_full",
    "cuarentena": "auto_update_full",
    "status": "diagnostic_full",
}

# Operaciones que requieren aprobación
OPERATIONS_REQUIRING_APPROVAL: List[str] = [
    "deploy",
    "release",
    "production",
    "upgrade",
]

# Operaciones que deben auto-sincronizar con remoto
AUTO_SYNC_OPERATIONS: List[str] = [
    "commit",
    "deploy",
    "session_shutdown",
]


# ============================================================================
# SYNC ENGINE
# ============================================================================

class SyncEngine:
    """
    Motor de sincronización que conecta POTs con todos los sistemas ATLAS.
    
    Responsabilidades:
    1. Determinar qué POT usar para cada operación
    2. Ejecutar sincronización con remoto cuando necesario
    3. Notificar a canales apropiados
    4. Registrar en bitácora/ANS
    5. Actualizar Dashboard
    """
    
    def __init__(self, config: Optional[SyncConfig] = None):
        self.config = config or SyncConfig()
        self._hooks: Dict[str, List[Callable]] = {
            "pre_pot": [],
            "post_pot": [],
            "on_error": [],
            "on_sync": [],
        }
    
    def get_pot_for_operation(self, operation: str) -> Optional[str]:
        """
        Obtener el ID del POT que corresponde a una operación.
        
        Args:
            operation: Nombre de la operación (ej: "commit", "deploy")
            
        Returns:
            ID del POT o None si no hay mapeo
        """
        # Normalizar operación
        op_normalized = operation.lower().strip().replace(" ", "_").replace("-", "_")
        
        # Buscar en mapeo directo
        if op_normalized in OPERATION_POT_MAP:
            return OPERATION_POT_MAP[op_normalized]
        
        # Buscar coincidencia parcial
        for key, pot_id in OPERATION_POT_MAP.items():
            if key in op_normalized or op_normalized in key:
                return pot_id
        
        return None
    
    def requires_approval(self, operation: str) -> bool:
        """Verificar si una operación requiere aprobación."""
        op_normalized = operation.lower().strip().replace(" ", "_")
        return op_normalized in OPERATIONS_REQUIRING_APPROVAL
    
    def should_auto_sync(self, operation: str) -> bool:
        """Verificar si una operación debe auto-sincronizar con remoto."""
        op_normalized = operation.lower().strip().replace(" ", "_")
        return op_normalized in AUTO_SYNC_OPERATIONS
    
    def get_channels_for_severity(self, severity: str) -> List[str]:
        """Obtener canales de notificación según severidad."""
        return self.config.severity_channel_map.get(
            severity.lower(), 
            ["ops", "bitacora"]
        )
    
    def register_hook(self, event: str, callback: Callable):
        """Registrar hook para eventos de sincronización."""
        if event in self._hooks:
            self._hooks[event].append(callback)
    
    def _fire_hooks(self, event: str, **kwargs):
        """Disparar hooks registrados."""
        for callback in self._hooks.get(event, []):
            try:
                callback(**kwargs)
            except Exception as e:
                logger.warning(f"Hook error on {event}: {e}")
    
    async def sync_to_cerebro(
        self, 
        message: str, 
        source: str = "sync_engine",
        ok: bool = True,
        metadata: Optional[Dict] = None
    ):
        """
        Sincronizar información con el Cerebro (ANS).
        
        Args:
            message: Mensaje a registrar
            source: Fuente del mensaje
            ok: Si fue exitoso o no
            metadata: Metadatos adicionales
        """
        if not self.config.bitacora_enabled:
            return
        
        try:
            import urllib.request
            
            payload = {
                "message": message,
                "ok": ok,
                "source": source,
                "timestamp": datetime.now().isoformat(),
            }
            if metadata:
                payload["metadata"] = metadata
            
            data = json.dumps(payload).encode()
            req = urllib.request.Request(
                f"{self.config.push_url}/ans/evolution-log",
                data=data,
                headers={"Content-Type": "application/json"},
                method="POST"
            )
            
            with urllib.request.urlopen(req, timeout=10) as resp:
                logger.debug(f"Synced to cerebro: {resp.status}")
                
        except Exception as e:
            logger.warning(f"Failed to sync to cerebro: {e}")
    
    async def sync_to_dashboard(self, event: str, data: Optional[Dict] = None):
        """
        Actualizar información en el Dashboard.
        
        Args:
            event: Tipo de evento
            data: Datos del evento
        """
        if not self.config.dashboard_enabled:
            return
        
        try:
            import urllib.request
            
            # Trigger refresh del dashboard
            req = urllib.request.Request(
                f"{self.config.push_url}/status",
                method="GET"
            )
            
            with urllib.request.urlopen(req, timeout=10) as resp:
                logger.debug(f"Dashboard refreshed: {resp.status}")
                
        except Exception as e:
            logger.warning(f"Failed to sync to dashboard: {e}")
    
    async def notify_channel(
        self, 
        channel: str, 
        message: str,
        level: str = "info"
    ):
        """
        Enviar notificación a un canal específico.
        
        Args:
            channel: Canal destino (telegram, ops, etc)
            message: Mensaje a enviar
            level: Nivel del mensaje (info, warning, error, critical)
        """
        if channel == "telegram" and not self.config.telegram_enabled:
            return
        if channel == "ops" and not self.config.ops_enabled:
            return
        
        try:
            # Para Telegram, usar el endpoint específico
            if channel == "telegram":
                from modules.humanoid.notify import send_telegram
                await send_telegram(message)
            
            # Para OPS, usar el bus interno
            elif channel == "ops":
                import urllib.request
                
                payload = {
                    "channel": "ops",
                    "message": message,
                    "level": level,
                    "timestamp": datetime.now().isoformat(),
                }
                data = json.dumps(payload).encode()
                req = urllib.request.Request(
                    f"{self.config.push_url}/ops/event",
                    data=data,
                    headers={"Content-Type": "application/json"},
                    method="POST"
                )
                
                try:
                    with urllib.request.urlopen(req, timeout=5) as resp:
                        pass
                except:
                    pass  # OPS puede no estar disponible
                    
        except Exception as e:
            logger.warning(f"Failed to notify {channel}: {e}")
    
    async def notify_all_channels(
        self, 
        message: str, 
        severity: str = "medium",
        level: str = "info"
    ):
        """
        Enviar notificación a todos los canales según severidad.
        
        Args:
            message: Mensaje a enviar
            severity: Severidad para determinar canales
            level: Nivel del mensaje
        """
        channels = self.get_channels_for_severity(severity)
        
        for channel in channels:
            if channel == "bitacora":
                await self.sync_to_cerebro(message, source="broadcast", ok=True)
            elif channel == "dashboard":
                await self.sync_to_dashboard("notification", {"message": message})
            else:
                await self.notify_channel(channel, message, level)
    
    def execute_git_sync(self, include_push: bool = True) -> Dict[str, Any]:
        """
        Ejecutar sincronización Git (commit + push).
        
        Args:
            include_push: Si incluir push después del commit
            
        Returns:
            Resultado de la sincronización
        """
        import subprocess
        
        result = {
            "success": True,
            "commit_hash": None,
            "pushed": False,
            "errors": [],
        }
        
        try:
            # Check status
            status = subprocess.run(
                ["git", "status", "--porcelain"],
                capture_output=True,
                text=True,
                timeout=30
            )
            
            if status.stdout.strip():
                # Hay cambios, hacer commit
                subprocess.run(["git", "add", "-A"], timeout=30, check=True)
                
                commit_result = subprocess.run(
                    ["git", "commit", "-m", "chore: auto-sync by ATLAS SyncEngine"],
                    capture_output=True,
                    text=True,
                    timeout=30
                )
                
                if commit_result.returncode == 0:
                    # Obtener hash
                    hash_result = subprocess.run(
                        ["git", "rev-parse", "--short", "HEAD"],
                        capture_output=True,
                        text=True,
                        timeout=10
                    )
                    result["commit_hash"] = hash_result.stdout.strip()
                    
                    if include_push:
                        push_result = subprocess.run(
                            ["git", "push", "origin", "HEAD"],
                            capture_output=True,
                            text=True,
                            timeout=180
                        )
                        result["pushed"] = push_result.returncode == 0
                        if push_result.returncode != 0:
                            result["errors"].append(f"Push failed: {push_result.stderr}")
                else:
                    result["errors"].append(f"Commit failed: {commit_result.stderr}")
            else:
                logger.info("No changes to sync")
                
        except Exception as e:
            result["success"] = False
            result["errors"].append(str(e))
        
        return result


# ============================================================================
# POT CONTEXT BUILDER
# ============================================================================

def build_pot_context(
    operation: str,
    user_context: Optional[Dict] = None,
    auto_values: bool = True
) -> Dict[str, Any]:
    """
    Construir contexto para ejecución de POT.
    
    Args:
        operation: Operación que se va a ejecutar
        user_context: Contexto proporcionado por el usuario
        auto_values: Si auto-rellenar valores comunes
        
    Returns:
        Contexto completo para el POT
    """
    context = user_context.copy() if user_context else {}
    
    if auto_values:
        # Valores automáticos
        context.setdefault("timestamp", datetime.now().isoformat())
        context.setdefault("operation", operation)
        context.setdefault("workspace", os.getcwd())
        
        # Información de Git si disponible
        try:
            import subprocess
            
            branch = subprocess.run(
                ["git", "branch", "--show-current"],
                capture_output=True,
                text=True,
                timeout=5
            )
            if branch.returncode == 0:
                context.setdefault("git_branch", branch.stdout.strip())
            
            remote = subprocess.run(
                ["git", "remote", "get-url", "origin"],
                capture_output=True,
                text=True,
                timeout=5
            )
            if remote.returncode == 0:
                context.setdefault("git_remote", remote.stdout.strip())
                
        except:
            pass
    
    return context


# ============================================================================
# AUTOMATIC SYNC DECORATORS
# ============================================================================

def auto_sync_pot(operation: str):
    """
    Decorador para auto-sincronizar después de ejecutar una función.
    
    Usage:
        @auto_sync_pot("commit")
        def my_function():
            # ... hacer cosas ...
            pass
    """
    def decorator(func):
        def wrapper(*args, **kwargs):
            result = func(*args, **kwargs)
            
            engine = SyncEngine()
            if engine.should_auto_sync(operation):
                sync_result = engine.execute_git_sync()
                if sync_result.get("commit_hash"):
                    logger.info(f"Auto-synced: {sync_result['commit_hash']}")
            
            return result
        return wrapper
    return decorator


# ============================================================================
# MODULE INITIALIZATION
# ============================================================================

# Instancia global del motor de sincronización
_sync_engine: Optional[SyncEngine] = None


def get_sync_engine() -> SyncEngine:
    """Obtener instancia del motor de sincronización."""
    global _sync_engine
    if _sync_engine is None:
        _sync_engine = SyncEngine()
    return _sync_engine


def sync_operation(operation: str, **kwargs) -> Dict[str, Any]:
    """
    Ejecutar una operación con su POT asociado y sincronización automática.
    
    Args:
        operation: Nombre de la operación
        **kwargs: Contexto adicional
        
    Returns:
        Resultado de la operación
    """
    from modules.humanoid.quality import get_pot, execute_pot
    
    engine = get_sync_engine()
    pot_id = engine.get_pot_for_operation(operation)
    
    if not pot_id:
        return {
            "success": False,
            "error": f"No POT found for operation: {operation}",
            "available_operations": list(OPERATION_POT_MAP.keys())
        }
    
    pot = get_pot(pot_id)
    if not pot:
        return {
            "success": False,
            "error": f"POT {pot_id} not found",
        }
    
    # Construir contexto
    context = build_pot_context(operation, kwargs)
    
    # Ejecutar POT
    result = execute_pot(pot, context=context)
    
    # Auto-sync si necesario
    if result.ok and engine.should_auto_sync(operation):
        engine.execute_git_sync()
    
    return {
        "success": result.ok,
        "pot_id": pot_id,
        "steps_ok": result.steps_ok,
        "steps_total": result.steps_total,
        "duration_seconds": (
            (datetime.fromisoformat(result.ended_at) - 
             datetime.fromisoformat(result.started_at)).total_seconds()
            if result.ended_at and result.started_at else 0
        ),
    }


# Exportar
__all__ = [
    "SyncEngine",
    "SyncConfig",
    "OPERATION_POT_MAP",
    "OPERATIONS_REQUIRING_APPROVAL",
    "AUTO_SYNC_OPERATIONS",
    "get_sync_engine",
    "sync_operation",
    "build_pot_context",
    "auto_sync_pot",
]

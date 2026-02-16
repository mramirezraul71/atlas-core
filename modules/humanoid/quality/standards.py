"""
ATLAS Technical Standards
==========================
Estándares técnicos para coherencia en todo el sistema.

Este módulo define:
- Estructura de respuestas estándar
- Códigos de error unificados
- Formato de timestamps
- Convenciones de nombres
"""
from __future__ import annotations

from dataclasses import dataclass, field, asdict
from datetime import datetime, timezone
from enum import IntEnum
from typing import Any, Dict, List, Optional, TypeVar, Generic

T = TypeVar("T")


# ============================================================================
# TIMESTAMPS - COHERENCIA
# ============================================================================

def now_iso() -> str:
    """Timestamp ISO 8601 en UTC."""
    return datetime.now(timezone.utc).isoformat()


def now_unix() -> int:
    """Timestamp Unix en segundos."""
    return int(datetime.now(timezone.utc).timestamp())


def now_unix_ms() -> int:
    """Timestamp Unix en milisegundos."""
    return int(datetime.now(timezone.utc).timestamp() * 1000)


# ============================================================================
# CÓDIGOS DE ESTADO - COHERENCIA
# ============================================================================

class StatusCode(IntEnum):
    """Códigos de estado estándar para ATLAS."""
    # Éxito (0-99)
    OK = 0
    CREATED = 1
    ACCEPTED = 2
    PARTIAL = 10
    
    # Errores de cliente (100-199)
    BAD_REQUEST = 100
    UNAUTHORIZED = 101
    FORBIDDEN = 102
    NOT_FOUND = 103
    CONFLICT = 104
    VALIDATION_ERROR = 110
    
    # Errores de servidor (200-299)
    INTERNAL_ERROR = 200
    NOT_IMPLEMENTED = 201
    SERVICE_UNAVAILABLE = 202
    TIMEOUT = 203
    DEPENDENCY_ERROR = 210
    
    # Errores de sistema (300-399)
    HARDWARE_ERROR = 300
    SENSOR_ERROR = 301
    MOTOR_ERROR = 302
    CAMERA_ERROR = 303
    NETWORK_ERROR = 310
    
    # Estados especiales (400-499)
    PENDING = 400
    IN_PROGRESS = 401
    CANCELLED = 402
    ESCALATED = 403


# ============================================================================
# RESPUESTA ESTÁNDAR - COHERENCIA
# ============================================================================

@dataclass
class StandardResponse(Generic[T]):
    """
    Respuesta estándar para TODAS las operaciones de ATLAS.
    
    Esto garantiza coherencia en:
    - Estructura de datos
    - Manejo de errores
    - Metadatos
    """
    ok: bool
    code: int = StatusCode.OK
    message: str = ""
    data: Optional[T] = None
    error: Optional[str] = None
    timestamp: str = field(default_factory=now_iso)
    elapsed_ms: int = 0
    
    # Metadatos opcionales
    request_id: Optional[str] = None
    source: Optional[str] = None
    
    def to_dict(self) -> Dict[str, Any]:
        """Convierte a diccionario."""
        d = asdict(self)
        # Limpiar campos None
        return {k: v for k, v in d.items() if v is not None}
    
    @classmethod
    def success(
        cls,
        data: T = None,
        message: str = "OK",
        **kwargs,
    ) -> "StandardResponse[T]":
        """Crea respuesta de éxito."""
        return cls(
            ok=True,
            code=StatusCode.OK,
            message=message,
            data=data,
            **kwargs,
        )
    
    @classmethod
    def error(
        cls,
        error: str,
        code: int = StatusCode.INTERNAL_ERROR,
        **kwargs,
    ) -> "StandardResponse[T]":
        """Crea respuesta de error."""
        return cls(
            ok=False,
            code=code,
            error=error,
            **kwargs,
        )
    
    @classmethod
    def not_found(cls, resource: str) -> "StandardResponse[T]":
        """Crea respuesta 404."""
        return cls(
            ok=False,
            code=StatusCode.NOT_FOUND,
            error=f"Resource not found: {resource}",
        )


# ============================================================================
# RESULTADO DE OPERACIÓN - COHERENCIA
# ============================================================================

@dataclass
class OperationResult:
    """
    Resultado de una operación (POT, comando, etc).
    
    Estructura consistente para:
    - Ejecución de POTs
    - Comandos shell
    - Llamadas HTTP
    - Operaciones de sensores
    """
    ok: bool
    operation: str
    started_at: str
    ended_at: str
    elapsed_ms: int
    
    # Detalles
    steps_completed: int = 0
    steps_total: int = 0
    output: Optional[str] = None
    error: Optional[str] = None
    
    # Contexto
    context: Dict[str, Any] = field(default_factory=dict)
    
    @property
    def success_rate(self) -> float:
        """Tasa de éxito de pasos."""
        if self.steps_total == 0:
            return 1.0 if self.ok else 0.0
        return self.steps_completed / self.steps_total
    
    def to_dict(self) -> Dict[str, Any]:
        """Convierte a diccionario."""
        return asdict(self)


# ============================================================================
# MÉTRICAS - COHERENCIA
# ============================================================================

@dataclass
class Metrics:
    """
    Métricas estándar de rendimiento.
    """
    total: int = 0
    successful: int = 0
    failed: int = 0
    
    # Tiempos (milisegundos)
    total_time_ms: int = 0
    min_time_ms: int = 0
    max_time_ms: int = 0
    avg_time_ms: int = 0
    
    # Timestamps
    first_at: Optional[str] = None
    last_at: Optional[str] = None
    
    def record(self, ok: bool, elapsed_ms: int) -> None:
        """Registra una operación."""
        self.total += 1
        if ok:
            self.successful += 1
        else:
            self.failed += 1
        
        self.total_time_ms += elapsed_ms
        
        if self.min_time_ms == 0 or elapsed_ms < self.min_time_ms:
            self.min_time_ms = elapsed_ms
        if elapsed_ms > self.max_time_ms:
            self.max_time_ms = elapsed_ms
        
        self.avg_time_ms = self.total_time_ms // self.total
        
        now = now_iso()
        if self.first_at is None:
            self.first_at = now
        self.last_at = now
    
    @property
    def success_rate(self) -> float:
        """Tasa de éxito."""
        if self.total == 0:
            return 0.0
        return self.successful / self.total
    
    def to_dict(self) -> Dict[str, Any]:
        """Convierte a diccionario."""
        d = asdict(self)
        d["success_rate"] = self.success_rate
        return d


# ============================================================================
# VALIDACIÓN - COHERENCIA
# ============================================================================

def validate_required(data: Dict[str, Any], required: List[str]) -> Optional[str]:
    """
    Valida que los campos requeridos estén presentes.
    
    Returns:
        None si válido, mensaje de error si no
    """
    missing = [f for f in required if f not in data or data[f] is None]
    if missing:
        return f"Missing required fields: {', '.join(missing)}"
    return None


def validate_type(value: Any, expected_type: type, field_name: str) -> Optional[str]:
    """
    Valida tipo de un valor.
    
    Returns:
        None si válido, mensaje de error si no
    """
    if not isinstance(value, expected_type):
        return f"Field '{field_name}' must be {expected_type.__name__}, got {type(value).__name__}"
    return None


# ============================================================================
# HELPERS
# ============================================================================

def clamp(value: float, min_val: float, max_val: float) -> float:
    """Limita un valor entre min y max."""
    return max(min_val, min(max_val, value))


def safe_get(d: Dict[str, Any], key: str, default: Any = None) -> Any:
    """Obtiene valor de dict de forma segura."""
    try:
        return d.get(key, default)
    except (AttributeError, TypeError):
        return default


__all__ = [
    # Timestamps
    "now_iso",
    "now_unix",
    "now_unix_ms",
    
    # Status codes
    "StatusCode",
    
    # Responses
    "StandardResponse",
    "OperationResult",
    
    # Metrics
    "Metrics",
    
    # Validation
    "validate_required",
    "validate_type",
    
    # Helpers
    "clamp",
    "safe_get",
]

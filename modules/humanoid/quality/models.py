"""
Quality Models: POT, Steps, Results, Categories.
Estructuras de datos para el sistema de calidad.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
from enum import Enum
from typing import Any, Dict, List, Optional


class POTCategory(str, Enum):
    """Categorías de POTs."""
    REPAIR = "repair"           # Reparaciones correctivas
    MAINTENANCE = "maintenance" # Mantenimiento preventivo
    INCIDENT = "incident"       # Gestión de incidentes
    DIAGNOSTIC = "diagnostic"   # Diagnósticos
    DEPLOYMENT = "deployment"   # Despliegues
    RECOVERY = "recovery"       # Recuperación de desastres
    SECURITY = "security"       # Procedimientos de seguridad
    CALIBRATION = "calibration" # Calibración de sensores/cámaras
    UPGRADE = "upgrade"         # Actualizaciones de sistema
    QUALITY = "quality"         # Calidad / tutorías / auditorías (no-operacional)
    COMMUNICATION = "communication"  # Interacción humano-robot (HRI)
    ROBOTICS = "robotics"       # Operaciones robóticas generales


class POTSeverity(str, Enum):
    """Severidad/Riesgo del POT."""
    LOW = "low"           # Bajo riesgo, auto-ejecutable
    MEDIUM = "medium"     # Riesgo medio, logging extensivo
    HIGH = "high"         # Alto riesgo, requiere aprobación
    CRITICAL = "critical" # Crítico, requiere aprobación + backup


class StepType(str, Enum):
    """Tipos de pasos en un POT."""
    SHELL = "shell"           # Alias de COMMAND (compatibilidad POTs legacy)
    COMMAND = "command"       # Ejecutar comando shell
    SCRIPT = "script"         # Ejecutar script Python
    HTTP = "http"             # Llamada HTTP
    CHECK = "check"           # Verificación de estado
    WAIT = "wait"             # Esperar tiempo
    CONFIRM = "confirm"       # Requiere confirmación manual
    MANUAL = "manual"         # Alias de CONFIRM (compatibilidad tutorías)
    ROLLBACK = "rollback"     # Paso de rollback
    LOG = "log"               # Solo logging
    SNAPSHOT = "snapshot"     # Capturar snapshot
    NOTIFY = "notify"         # Notificar (Telegram/OPS)


@dataclass
class POTStep:
    """Un paso dentro de un POT."""
    id: str
    name: str
    description: str
    step_type: StepType
    
    # Configuración del paso
    command: Optional[str] = None          # Para COMMAND
    shell_command: Optional[str] = None    # Para SHELL (compatibilidad)
    expected_output: Optional[str] = None  # Para COMMAND/SHELL (substring esperado en stdout)
    script_path: Optional[str] = None      # Para SCRIPT
    script_function: Optional[str] = None  # Para SCRIPT
    http_method: Optional[str] = None      # Para HTTP
    http_url: Optional[str] = None         # Para HTTP
    http_body: Optional[Dict[str, Any]] = None
    http_headers: Optional[Dict[str, Any]] = None  # Para HTTP (headers extra)
    expected_status: Optional[int] = None           # Para HTTP (status esperado)
    expected_http_status: Optional[int] = None      # Alias legacy (compatibilidad)
    check_expression: Optional[str] = None # Para CHECK (Python expr)
    wait_seconds: Optional[int] = None     # Para WAIT
    notify_channel: Optional[str] = None   # Para NOTIFY
    notify_message: Optional[str] = None
    on_failure: Optional[str] = None       # Compatibilidad: hint (no controla flujo en executor)
    
    # Control de flujo
    timeout_seconds: int = 60
    retries: int = 0
    retry_delay_seconds: int = 5
    continue_on_failure: bool = False
    rollback_step_id: Optional[str] = None
    requires_approval: bool = False
    
    # Condiciones
    condition: Optional[str] = None  # Python expr, ejecutar si True
    skip_if: Optional[str] = None    # Python expr, saltar si True
    
    # Evidencia
    capture_output: bool = True
    capture_screenshot: bool = False
    
    # Metadata
    tutorial_notes: str = ""  # Notas tutoriales para el operador
    manual_instructions: str = ""  # Compatibilidad: instrucciones extensas (StepType.MANUAL)
    common_errors: List[str] = field(default_factory=list)
    troubleshooting: str = ""
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "description": self.description,
            "step_type": self.step_type.value if isinstance(self.step_type, StepType) else self.step_type,
            "command": self.command,
            "script_path": self.script_path,
            "timeout_seconds": self.timeout_seconds,
            "retries": self.retries,
            "continue_on_failure": self.continue_on_failure,
            "requires_approval": self.requires_approval,
            "tutorial_notes": self.tutorial_notes,
        }


@dataclass
class POT:
    """Procedimiento Operacional de Trabajo."""
    id: str
    name: str
    description: str
    category: POTCategory
    severity: POTSeverity
    
    # Pasos del procedimiento
    steps: List[POTStep] = field(default_factory=list)
    
    # Metadata
    version: str = "1.0.0"
    author: str = "ATLAS QA"
    created_at: str = ""
    updated_at: str = ""
    
    # Triggers: qué check_ids o palabras clave activan este POT
    trigger_check_ids: List[str] = field(default_factory=list)
    trigger_keywords: List[str] = field(default_factory=list)
    
    # Requisitos previos
    prerequisites: List[str] = field(default_factory=list)
    required_services: List[str] = field(default_factory=list)
    required_permissions: List[str] = field(default_factory=list)
    
    # Objetivos y métricas
    objectives: List[str] = field(default_factory=list)
    success_criteria: str = ""
    estimated_duration_minutes: int = 5
    
    # Tutorial
    tutorial_overview: str = ""
    best_practices: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    related_pots: List[str] = field(default_factory=list)
    additional_notes: str = ""  # Compatibilidad (POTs legacy)
    
    # Rollback
    has_rollback: bool = False
    rollback_steps: List[POTStep] = field(default_factory=list)
    
    # Tags para búsqueda
    tags: List[str] = field(default_factory=list)
    
    def __post_init__(self):
        if not self.created_at:
            self.created_at = datetime.now(timezone.utc).isoformat()
        if not self.updated_at:
            self.updated_at = self.created_at
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "description": self.description,
            "category": self.category.value if isinstance(self.category, POTCategory) else self.category,
            "severity": self.severity.value if isinstance(self.severity, POTSeverity) else self.severity,
            "version": self.version,
            "steps_count": len(self.steps),
            "steps": [s.to_dict() for s in self.steps],
            "trigger_check_ids": self.trigger_check_ids,
            "trigger_keywords": self.trigger_keywords,
            "prerequisites": self.prerequisites,
            "objectives": self.objectives,
            "success_criteria": self.success_criteria,
            "estimated_duration_minutes": self.estimated_duration_minutes,
            "tutorial_overview": self.tutorial_overview,
            "best_practices": self.best_practices,
            "warnings": self.warnings,
            "has_rollback": self.has_rollback,
            "tags": self.tags,
        }


@dataclass
class StepResult:
    """Resultado de ejecución de un paso."""
    step_id: str
    step_name: str
    ok: bool
    started_at: str
    ended_at: str
    elapsed_ms: int
    output: str = ""
    error: str = ""
    exit_code: Optional[int] = None
    skipped: bool = False
    skip_reason: str = ""
    retries_used: int = 0
    screenshot_path: Optional[str] = None
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "step_id": self.step_id,
            "step_name": self.step_name,
            "ok": self.ok,
            "started_at": self.started_at,
            "ended_at": self.ended_at,
            "elapsed_ms": self.elapsed_ms,
            "output": self.output[:2000] if self.output else "",
            "error": self.error,
            "exit_code": self.exit_code,
            "skipped": self.skipped,
            "retries_used": self.retries_used,
        }


@dataclass
class POTResult:
    """Resultado de ejecución completa de un POT."""
    pot_id: str
    pot_name: str
    ok: bool
    started_at: str
    ended_at: str
    elapsed_ms: int
    
    # Resultados de pasos
    step_results: List[StepResult] = field(default_factory=list)
    
    # Resumen
    steps_total: int = 0
    steps_ok: int = 0
    steps_failed: int = 0
    steps_skipped: int = 0
    
    # Contexto
    context: Dict[str, Any] = field(default_factory=dict)
    incident_id: Optional[str] = None
    ticket_id: Optional[str] = None
    
    # Verificación final
    verification_ok: bool = False
    verification_message: str = ""
    
    # Rollback
    rollback_executed: bool = False
    rollback_ok: bool = False
    
    # Reportes
    report_path: Optional[str] = None
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "pot_id": self.pot_id,
            "pot_name": self.pot_name,
            "ok": self.ok,
            "started_at": self.started_at,
            "ended_at": self.ended_at,
            "elapsed_ms": self.elapsed_ms,
            "steps_total": self.steps_total,
            "steps_ok": self.steps_ok,
            "steps_failed": self.steps_failed,
            "steps_skipped": self.steps_skipped,
            "step_results": [r.to_dict() for r in self.step_results],
            "verification_ok": self.verification_ok,
            "verification_message": self.verification_message,
            "rollback_executed": self.rollback_executed,
            "incident_id": self.incident_id,
            "report_path": self.report_path,
        }

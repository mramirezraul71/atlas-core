"""
ErrorClassifier - Categoriza errores (TRANSIENT, CONFIGURATION, RESOURCE, DEPENDENCY, FATAL)
y sugiere estrategia de recuperación.
"""
from __future__ import annotations

import logging
import re
from dataclasses import dataclass
from enum import Enum
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


class ErrorType(str, Enum):
    TRANSIENT = "transient"
    CONFIGURATION = "configuration"
    RESOURCE = "resource"
    DEPENDENCY = "dependency"
    FATAL = "fatal"


class RecoveryStrategy(str, Enum):
    RETRY = "retry"
    RESTART_SERVICE = "restart_service"
    RESTART_ALL = "restart_all"
    ROLLBACK = "rollback"
    FALLBACK = "fallback"
    DEGRADE = "degrade"
    ISOLATE = "isolate"
    ALERT_HUMAN = "alert_human"


@dataclass
class ClassificationResult:
    error_type: ErrorType
    severity: str  # "low" | "medium" | "high" | "critical"
    recoverable: bool
    suggested_strategy: RecoveryStrategy
    message: str


# Patrones para clasificación por mensaje/excepción
TRANSIENT_PATTERNS = [
    r"timeout", r"timed out", r"connection refused", r"Connection reset",
    r"Temporary failure", r"503", r"502", r"network.*unreachable",
    r"BrokenPipe", r"ConnectionError", r"urllib\.error\.URLError",
]
CONFIGURATION_PATTERNS = [
    r"port.*in use", r"Address already in use", r"10048",
    r"API key", r"invalid.*config", r"missing.*env", r"FileNotFoundError.*config",
]
RESOURCE_PATTERNS = [
    r"MemoryError", r"out of memory", r"disk full", r"No space left",
    r"too many open files", r"resource.*exhausted", r"CPU.*100",
]
DEPENDENCY_PATTERNS = [
    r"503", r"502", r"service unavailable", r"upstream", r"dependency",
    r"health.*fail", r"nexus.*disconnected", r"ollama.*refused",
]
FATAL_PATTERNS = [
    r"corrupted", r"corrupt", r"integrity", r"critical file", r"SyntaxError",
    r"ImportError", r"ModuleNotFoundError", r"database.*locked.*permanently",
]


class ErrorClassifier:
    """
    Clasifica excepciones y contexto en ErrorType y sugiere RecoveryStrategy.
    Opcionalmente aprende de errores resueltos (knowledge base en memoria por ahora).
    """

    def __init__(self, config: dict | None = None):
        self._config = config or _load_config().get("self_healing", {})
        self._learned: list[dict] = []  # [{error_sig, resolution, success}]

    def classify_error(self, exception: BaseException, context: dict[str, Any] | None = None) -> ClassificationResult:
        """Clasifica el error y devuelve tipo, severidad, recuperable y estrategia sugerida."""
        context = context or {}
        msg = (getattr(exception, "message", None) or str(exception)).lower()
        exc_name = type(exception).__name__
        full_text = f"{exc_name} {msg}"

        error_type = ErrorType.TRANSIENT
        severity = "medium"
        recoverable = True
        strategy = RecoveryStrategy.RETRY

        for pat in TRANSIENT_PATTERNS:
            if re.search(pat, full_text, re.I):
                error_type = ErrorType.TRANSIENT
                strategy = RecoveryStrategy.RETRY
                severity = "low"
                break
        if error_type == ErrorType.TRANSIENT:
            for pat in CONFIGURATION_PATTERNS:
                if re.search(pat, full_text, re.I):
                    error_type = ErrorType.CONFIGURATION
                    strategy = RecoveryStrategy.ALERT_HUMAN
                    severity = "high"
                    recoverable = False
                    break
        if error_type in (ErrorType.TRANSIENT, ErrorType.CONFIGURATION):
            for pat in RESOURCE_PATTERNS:
                if re.search(pat, full_text, re.I):
                    error_type = ErrorType.RESOURCE
                    strategy = RecoveryStrategy.DEGRADE
                    severity = "high"
                    break
        if error_type not in (ErrorType.RESOURCE, ErrorType.CONFIGURATION):
            for pat in DEPENDENCY_PATTERNS:
                if re.search(pat, full_text, re.I):
                    error_type = ErrorType.DEPENDENCY
                    strategy = RecoveryStrategy.FALLBACK
                    severity = "medium"
                    break
        for pat in FATAL_PATTERNS:
            if re.search(pat, full_text, re.I):
                error_type = ErrorType.FATAL
                strategy = RecoveryStrategy.ALERT_HUMAN
                severity = "critical"
                recoverable = False
                break

        if error_type == ErrorType.DEPENDENCY and context.get("service") == "nexus":
            strategy = RecoveryStrategy.RESTART_SERVICE

        return ClassificationResult(
            error_type=error_type,
            severity=severity,
            recoverable=recoverable,
            suggested_strategy=strategy,
            message=f"{error_type.value}: {str(exception)[:200]}",
        )

    def get_recovery_strategy(self, error_type: ErrorType) -> RecoveryStrategy:
        """Devuelve la estrategia por defecto para un tipo de error."""
        mapping = {
            ErrorType.TRANSIENT: RecoveryStrategy.RETRY,
            ErrorType.CONFIGURATION: RecoveryStrategy.ALERT_HUMAN,
            ErrorType.RESOURCE: RecoveryStrategy.DEGRADE,
            ErrorType.DEPENDENCY: RecoveryStrategy.FALLBACK,
            ErrorType.FATAL: RecoveryStrategy.ALERT_HUMAN,
        }
        return mapping.get(error_type, RecoveryStrategy.RETRY)

    def learn_from_error(self, error: BaseException, resolution: str, success: bool) -> None:
        """Registra resolución para mejorar sugerencias futuras (memoria en proceso)."""
        sig = f"{type(error).__name__}:{str(error)[:100]}"
        self._learned.append({"error_signature": sig, "resolution": resolution, "success": success})
        if len(self._learned) > 500:
            self._learned.pop(0)

"""
Auto Router: Sistema de enrutamiento inteligente de IA.

Selecciona automáticamente el modelo óptimo según:
- Tipo de tarea (código, razonamiento, chat, visión, etc.)
- Complejidad del prompt
- Disponibilidad del modelo
- Historial de rendimiento
"""
from __future__ import annotations

import os
import re
import time
import logging
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple
from enum import Enum

logger = logging.getLogger(__name__)


class TaskType(Enum):
    """Tipos de tarea para enrutamiento."""
    FAST = "fast"           # Respuestas rápidas, simples
    CHAT = "chat"           # Conversación general
    CODE = "code"           # Generación/análisis de código
    REASON = "reason"       # Razonamiento complejo
    TOOLS = "tools"         # Uso de herramientas/funciones
    VISION = "vision"       # Análisis de imágenes
    ARCHITECT = "architect" # Diseño de arquitectura
    OPTIMIZER = "optimizer" # Optimización de código
    CREATIVE = "creative"   # Generación creativa
    ANALYSIS = "analysis"   # Análisis de datos


@dataclass
class ModelConfig:
    """Configuración de modelo."""
    provider: str
    model_name: str
    full_key: str
    task_types: List[TaskType]
    max_tokens: int = 4096
    supports_vision: bool = False
    avg_latency_ms: float = 0.0
    success_rate: float = 1.0
    priority: int = 1  # Menor = mayor prioridad


@dataclass
class RouteResult:
    """Resultado del enrutamiento."""
    model_config: ModelConfig
    task_type: TaskType
    confidence: float
    reason: str
    fallback_models: List[ModelConfig] = field(default_factory=list)


class AutoRouter:
    """
    Router automático de IA.
    
    Selecciona el mejor modelo basado en:
    1. Análisis del prompt (detecta tipo de tarea)
    2. Modelos disponibles
    3. Historial de rendimiento
    """
    
    # Patrones para detectar tipo de tarea
    PATTERNS = {
        TaskType.CODE: [
            r'\b(code|código|programa|function|función|class|clase|def |import |from |async |await )\b',
            r'\b(python|javascript|typescript|java|c\+\+|rust|go|sql)\b',
            r'\b(error|bug|fix|debug|refactor|implement|implementa)\b',
            r'```\w*\n',  # Code blocks
        ],
        TaskType.REASON: [
            r'\b(why|por qué|explain|explica|analyze|analiza|reason|razon)\b',
            r'\b(step by step|paso a paso|think|piensa|consider|considera)\b',
            r'\b(compare|compara|evaluate|evalua|pros and cons)\b',
        ],
        TaskType.VISION: [
            r'\b(image|imagen|photo|foto|picture|see|ver|look|mira|visual)\b',
            r'\b(describe|describe|what is in|qué hay en|show|muestra)\b',
        ],
        TaskType.ARCHITECT: [
            r'\b(architecture|arquitectura|design|diseño|system|sistema|structure|estructura)\b',
            r'\b(pattern|patrón|module|módulo|component|componente|service|servicio)\b',
            r'\b(scalable|escalable|distributed|distribuido|microservice)\b',
        ],
        TaskType.OPTIMIZER: [
            r'\b(optimize|optimiza|improve|mejora|faster|más rápido|efficient|eficiente)\b',
            r'\b(performance|rendimiento|speed|velocidad|memory|memoria)\b',
            r'\b(refactor|reduce|complexity|complejidad)\b',
        ],
        TaskType.CREATIVE: [
            r'\b(create|crea|generate|genera|write|escribe|story|historia|poem|poema)\b',
            r'\b(creative|creativo|imagine|imagina|invent|inventa)\b',
        ],
        TaskType.ANALYSIS: [
            r'\b(analyze|analiza|data|datos|statistics|estadísticas|trend|tendencia)\b',
            r'\b(report|reporte|summary|resumen|insight|findings)\b',
        ],
        TaskType.TOOLS: [
            r'\b(tool|herramienta|function call|llamada de función|api|execute|ejecuta)\b',
            r'\b(search|busca|fetch|download|descarga|send|envía)\b',
        ],
    }
    
    def __init__(self):
        self._models: Dict[str, ModelConfig] = {}
        self._stats: Dict[str, Dict[str, Any]] = {}  # model -> {latency, success, count}
        self._load_models()
    
    def _env(self, key: str, default: str = "") -> str:
        return (os.getenv(key) or "").strip() or default
    
    def _load_models(self) -> None:
        """Carga configuración de modelos desde env."""
        model_configs = [
            ("AI_FAST_MODEL", [TaskType.FAST, TaskType.CHAT], False, 1),
            ("AI_CHAT_MODEL", [TaskType.CHAT, TaskType.CREATIVE], False, 2),
            ("AI_CODE_MODEL", [TaskType.CODE, TaskType.OPTIMIZER], False, 1),
            ("AI_REASON_MODEL", [TaskType.REASON, TaskType.ANALYSIS, TaskType.ARCHITECT], False, 1),
            ("AI_TOOLS_MODEL", [TaskType.TOOLS], False, 1),
            ("AI_VISION_MODEL", [TaskType.VISION], True, 1),
            ("AI_ARCHITECT_MODEL", [TaskType.ARCHITECT], False, 2),
            ("AI_OPTIMIZER_MODEL", [TaskType.OPTIMIZER], False, 2),
            ("AI_CREATIVE_MODEL", [TaskType.CREATIVE], False, 2),
            ("AI_ANALYSIS_MODEL", [TaskType.ANALYSIS], False, 2),
        ]
        
        for env_key, task_types, supports_vision, priority in model_configs:
            full_key = self._env(env_key)
            if not full_key:
                continue
            
            # Parse provider:model
            if ":" in full_key:
                parts = full_key.split(":", 1)
                provider = parts[0]
                model_name = parts[1]
            else:
                provider = "ollama"
                model_name = full_key
            
            config = ModelConfig(
                provider=provider,
                model_name=model_name,
                full_key=full_key,
                task_types=task_types,
                supports_vision=supports_vision,
                priority=priority,
            )
            
            self._models[full_key] = config
    
    def _detect_task_type(self, prompt: str) -> Tuple[TaskType, float]:
        """Detecta el tipo de tarea del prompt."""
        prompt_lower = prompt.lower()
        
        scores: Dict[TaskType, float] = {}
        
        for task_type, patterns in self.PATTERNS.items():
            score = 0.0
            for pattern in patterns:
                matches = re.findall(pattern, prompt_lower, re.IGNORECASE)
                score += len(matches) * 0.2
            scores[task_type] = min(score, 1.0)
        
        # Determinar tipo por longitud si no hay matches claros
        prompt_len = len(prompt)
        
        if max(scores.values(), default=0) < 0.2:
            if prompt_len < 100:
                return TaskType.FAST, 0.6
            elif prompt_len < 500:
                return TaskType.CHAT, 0.5
            else:
                return TaskType.REASON, 0.4
        
        # Retornar el de mayor score
        best_type = max(scores, key=lambda t: scores[t])
        return best_type, scores[best_type]
    
    def _get_models_for_task(self, task_type: TaskType) -> List[ModelConfig]:
        """Obtiene modelos que soportan un tipo de tarea."""
        models = []
        for config in self._models.values():
            if task_type in config.task_types:
                models.append(config)
        
        # Ordenar por prioridad y success_rate
        models.sort(key=lambda m: (m.priority, -m.success_rate))
        return models
    
    def route(
        self,
        prompt: str,
        has_image: bool = False,
        prefer_fast: bool = False,
        force_task_type: Optional[TaskType] = None,
    ) -> RouteResult:
        """
        Enruta el prompt al mejor modelo.
        
        Args:
            prompt: El prompt del usuario
            has_image: Si incluye una imagen
            prefer_fast: Preferir modelos rápidos
            force_task_type: Forzar un tipo de tarea específico
        
        Returns:
            RouteResult con el modelo seleccionado
        """
        # Si hay imagen, forzar visión
        if has_image:
            task_type = TaskType.VISION
            confidence = 1.0
        elif force_task_type:
            task_type = force_task_type
            confidence = 1.0
        else:
            task_type, confidence = self._detect_task_type(prompt)
        
        # Obtener modelos candidatos
        candidates = self._get_models_for_task(task_type)
        
        # Si no hay candidatos, fallback a CHAT
        if not candidates:
            task_type = TaskType.CHAT
            candidates = self._get_models_for_task(TaskType.CHAT)
        
        # Si aún no hay, usar el primer modelo disponible
        if not candidates and self._models:
            candidates = list(self._models.values())[:1]
        
        if not candidates:
            # Modelo por defecto hardcoded
            default = ModelConfig(
                provider="ollama",
                model_name="llama3.1:latest",
                full_key="ollama:llama3.1:latest",
                task_types=[TaskType.CHAT],
            )
            return RouteResult(
                model_config=default,
                task_type=task_type,
                confidence=0.5,
                reason="No models configured, using default",
            )
        
        # Seleccionar mejor modelo
        if prefer_fast:
            # Ordenar por latencia
            candidates.sort(key=lambda m: m.avg_latency_ms)
        
        best = candidates[0]
        fallbacks = candidates[1:3] if len(candidates) > 1 else []
        
        reason = f"Selected {best.model_name} for {task_type.value} task"
        if confidence < 0.5:
            reason += " (low confidence)"
        
        logger.info(f"AutoRouter: {reason}")
        
        return RouteResult(
            model_config=best,
            task_type=task_type,
            confidence=confidence,
            reason=reason,
            fallback_models=fallbacks,
        )
    
    def record_result(
        self,
        model_key: str,
        success: bool,
        latency_ms: float,
    ) -> None:
        """Registra resultado para mejorar routing futuro."""
        if model_key not in self._stats:
            self._stats[model_key] = {
                "total": 0,
                "success": 0,
                "total_latency": 0,
            }
        
        stats = self._stats[model_key]
        stats["total"] += 1
        if success:
            stats["success"] += 1
        stats["total_latency"] += latency_ms
        
        # Actualizar config del modelo
        if model_key in self._models:
            config = self._models[model_key]
            config.success_rate = stats["success"] / stats["total"]
            config.avg_latency_ms = stats["total_latency"] / stats["total"]
    
    def get_status(self) -> Dict[str, Any]:
        """Obtiene estado del router."""
        return {
            "models_configured": len(self._models),
            "models": {
                key: {
                    "provider": cfg.provider,
                    "model": cfg.model_name,
                    "tasks": [t.value for t in cfg.task_types],
                    "supports_vision": cfg.supports_vision,
                    "success_rate": cfg.success_rate,
                    "avg_latency_ms": cfg.avg_latency_ms,
                }
                for key, cfg in self._models.items()
            },
            "stats": self._stats,
        }


# Singleton
_auto_router: Optional[AutoRouter] = None


def get_auto_router() -> AutoRouter:
    """Obtiene el router automático singleton."""
    global _auto_router
    if _auto_router is None:
        _auto_router = AutoRouter()
    return _auto_router


def auto_route(
    prompt: str,
    has_image: bool = False,
    prefer_fast: bool = False,
) -> RouteResult:
    """Atajo para enrutar un prompt."""
    return get_auto_router().route(prompt, has_image, prefer_fast)

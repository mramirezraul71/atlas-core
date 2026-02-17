"""
InhibitoryControl: Control inhibitorio del lóbulo frontal.

Análogo biológico: Corteza orbitofrontal + corteza prefrontal ventrolateral
- Bloqueo de acciones peligrosas
- Supresión de impulsos inapropiados
- Gate de seguridad antes de ejecución
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Set

logger = logging.getLogger(__name__)


@dataclass
class InhibitionRule:
    """Regla de inhibición."""
    id: str
    name: str
    description: str
    condition: Callable[[Any, Any], bool]  # (action, context) -> should_block
    priority: int = 0  # Mayor = más importante
    is_absolute: bool = False  # Si True, no se puede override
    
    def check(self, action: Any, context: Any) -> bool:
        """Verifica si la regla bloquea la acción."""
        try:
            return self.condition(action, context)
        except Exception as e:
            logger.error(f"Error checking rule {self.id}: {e}")
            return False  # No bloquear en caso de error


@dataclass
class InhibitionVerdict:
    """Veredicto de control inhibitorio."""
    allowed: bool
    blocked_by: Optional[str] = None
    reason: Optional[str] = None
    can_override: bool = False
    warnings: List[str] = field(default_factory=list)
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "allowed": self.allowed,
            "blocked_by": self.blocked_by,
            "reason": self.reason,
            "can_override": self.can_override,
            "warnings": self.warnings,
        }


class InhibitoryControl:
    """
    Control inhibitorio del lóbulo frontal.
    
    Actúa como gate de seguridad que evalúa cada acción antes de ejecutarla.
    Implementa reglas absolutas (siempre aplican) y contextuales.
    """
    
    def __init__(self):
        self.rules: List[InhibitionRule] = []
        self.override_tokens: Set[str] = set()  # Tokens para override temporal
        
        # Registrar reglas por defecto
        self._register_default_rules()
    
    def _register_default_rules(self) -> None:
        """Registra reglas de seguridad por defecto."""
        
        # Regla: Emergency Stop activo
        self.add_rule(InhibitionRule(
            id="emergency_stop",
            name="Emergency Stop",
            description="Bloquea todas las acciones durante emergency stop",
            condition=lambda a, c: c.get("emergency_stop", False),
            priority=1000,
            is_absolute=True,
        ))
        
        # Regla: Acción destructiva sin confirmación
        self.add_rule(InhibitionRule(
            id="destructive_no_confirm",
            name="Destructive Action",
            description="Bloquea acciones destructivas sin confirmación",
            condition=self._is_destructive_unconfirmed,
            priority=900,
            is_absolute=True,
        ))
        
        # Regla: Límites de articulaciones
        self.add_rule(InhibitionRule(
            id="joint_limits",
            name="Joint Limits",
            description="Bloquea movimientos fuera de límites articulares",
            condition=self._exceeds_joint_limits,
            priority=800,
            is_absolute=True,
        ))
        
        # Regla: Velocidad excesiva cerca de humanos
        self.add_rule(InhibitionRule(
            id="human_proximity_speed",
            name="Human Proximity Speed",
            description="Bloquea velocidad alta cuando hay humanos cerca",
            condition=self._speed_with_human_nearby,
            priority=700,
            is_absolute=False,
        ))
        
        # Regla: Batería crítica
        self.add_rule(InhibitionRule(
            id="battery_critical",
            name="Battery Critical",
            description="Bloquea acciones de alto consumo con batería crítica",
            condition=self._battery_critical_action,
            priority=600,
            is_absolute=False,
        ))
        
        # Regla: Colisión inminente
        self.add_rule(InhibitionRule(
            id="collision_imminent",
            name="Collision Imminent",
            description="Bloquea movimiento cuando hay obstáculo en trayectoria",
            condition=self._collision_imminent,
            priority=850,
            is_absolute=True,
        ))
    
    def add_rule(self, rule: InhibitionRule) -> None:
        """Agrega regla de inhibición."""
        self.rules.append(rule)
        # Ordenar por prioridad (mayor primero)
        self.rules.sort(key=lambda r: r.priority, reverse=True)
    
    def remove_rule(self, rule_id: str) -> bool:
        """Remueve regla por ID."""
        for i, rule in enumerate(self.rules):
            if rule.id == rule_id:
                del self.rules[i]
                return True
        return False
    
    def check(self, action: Any, context: Dict[str, Any]) -> InhibitionVerdict:
        """
        Verifica si una acción debe ser bloqueada.
        
        Args:
            action: Acción a verificar
            context: Contexto de ejecución
        
        Returns:
            InhibitionVerdict indicando si está permitida
        """
        warnings = []
        
        for rule in self.rules:
            if rule.check(action, context):
                # Regla activada
                if rule.is_absolute:
                    # No se puede override
                    return InhibitionVerdict(
                        allowed=False,
                        blocked_by=rule.id,
                        reason=rule.description,
                        can_override=False,
                    )
                else:
                    # Se puede override con token
                    if rule.id in self.override_tokens:
                        warnings.append(f"Override activo para: {rule.name}")
                        continue
                    
                    return InhibitionVerdict(
                        allowed=False,
                        blocked_by=rule.id,
                        reason=rule.description,
                        can_override=True,
                        warnings=warnings,
                    )
        
        return InhibitionVerdict(allowed=True, warnings=warnings)
    
    def set_override(self, rule_id: str, duration_s: float = 30.0) -> bool:
        """
        Establece override temporal para una regla.
        
        Args:
            rule_id: ID de la regla
            duration_s: Duración del override en segundos
        
        Returns:
            True si se estableció el override
        """
        # Verificar que la regla existe y no es absoluta
        rule = next((r for r in self.rules if r.id == rule_id), None)
        if not rule:
            return False
        if rule.is_absolute:
            logger.warning(f"Cannot override absolute rule: {rule_id}")
            return False
        
        self.override_tokens.add(rule_id)
        
        # Programar remoción del override
        import threading
        def remove_override():
            self.override_tokens.discard(rule_id)
        
        timer = threading.Timer(duration_s, remove_override)
        timer.daemon = True
        timer.start()
        
        logger.info(f"Override set for rule {rule_id}, duration={duration_s}s")
        return True
    
    def clear_override(self, rule_id: str) -> None:
        """Limpia override de una regla."""
        self.override_tokens.discard(rule_id)
    
    # === Condiciones de Reglas ===
    
    def _is_destructive_unconfirmed(self, action: Any, context: Dict) -> bool:
        """Verifica si es acción destructiva sin confirmación."""
        destructive_types = {"delete", "remove", "destroy", "reset", "format"}
        
        action_type = ""
        if hasattr(action, "action_type"):
            action_type = action.action_type
        elif isinstance(action, dict):
            action_type = action.get("action_type", "")
        
        is_destructive = action_type.lower() in destructive_types
        has_confirmation = context.get("confirmed", False)
        
        return is_destructive and not has_confirmation
    
    def _exceeds_joint_limits(self, action: Any, context: Dict) -> bool:
        """Verifica si movimiento excede límites articulares."""
        # Solo aplica a comandos de movimiento
        if not hasattr(action, "action_type"):
            return False
        
        if action.action_type not in ("move", "position", "joint_position"):
            return False
        
        # Verificar límites (simplificado)
        params = action.parameters if hasattr(action, "parameters") else {}
        if isinstance(params, dict):
            position = params.get("position", 0)
            joint_id = params.get("joint_id", "")
            
            # Límites genéricos (deberían venir de configuración)
            limits = context.get("joint_limits", {})
            if joint_id in limits:
                min_pos, max_pos = limits[joint_id]
                if position < min_pos or position > max_pos:
                    return True
        
        return False
    
    def _speed_with_human_nearby(self, action: Any, context: Dict) -> bool:
        """Verifica si hay velocidad alta con humano cerca."""
        human_distance = context.get("human_distance", float("inf"))
        
        if human_distance > 1.5:  # Más de 1.5m, OK
            return False
        
        # Verificar velocidad del movimiento
        params = getattr(action, "parameters", {})
        if isinstance(params, dict):
            speed = params.get("speed", params.get("velocity", 0))
            
            # Velocidad máxima permitida según distancia
            max_speed = 0.5 if human_distance < 0.5 else 1.0
            
            if speed > max_speed:
                return True
        
        return False
    
    def _battery_critical_action(self, action: Any, context: Dict) -> bool:
        """Verifica si es acción de alto consumo con batería crítica."""
        battery = context.get("battery_percent", 100)
        
        if battery > 10:  # Batería OK
            return False
        
        # Acciones de alto consumo
        high_consumption = {"navigate", "run", "lift", "carry"}
        
        action_type = ""
        if hasattr(action, "action_type"):
            action_type = action.action_type
        elif isinstance(action, dict):
            action_type = action.get("action_type", "")
        
        return action_type.lower() in high_consumption
    
    def _collision_imminent(self, action: Any, context: Dict) -> bool:
        """Verifica si hay colisión inminente en trayectoria."""
        obstacles = context.get("obstacles_in_path", [])
        
        if not obstacles:
            return False
        
        # Verificar si algún obstáculo está muy cerca
        for obstacle in obstacles:
            distance = obstacle.get("distance", float("inf"))
            if distance < 0.1:  # Menos de 10cm
                return True
        
        return False

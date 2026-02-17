"""
SafetyPolicy: Politicas de seguridad del tronco encefalico.

Analogo biologico: Reflejos de proteccion
- Reglas de seguridad absolutas
- Verificacion de acciones
- Parada de emergencia
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

logger = logging.getLogger(__name__)


@dataclass
class SafetyRule:
    """Regla de seguridad."""
    id: str
    name: str
    description: str
    condition: Callable[[Any, Dict[str, Any]], bool]  # (action, context) -> should_block
    priority: int = 0  # Mayor = mas importante
    is_absolute: bool = False  # Si True, no se puede anular
    category: str = "general"  # collision, thermal, power, motion, etc.
    
    def check(self, action: Any, context: Dict[str, Any]) -> bool:
        """Verifica si la regla bloquea la accion."""
        try:
            return self.condition(action, context)
        except Exception as e:
            logger.error(f"Error checking safety rule {self.id}: {e}")
            return False  # No bloquear en caso de error interno


@dataclass
class SafetyVerdict:
    """Veredicto de verificacion de seguridad."""
    allowed: bool
    blocked_by: Optional[str] = None
    reason: Optional[str] = None
    can_override: bool = False
    warnings: List[str] = field(default_factory=list)
    timestamp_ns: int = field(default_factory=lambda: time.time_ns())
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "allowed": self.allowed,
            "blocked_by": self.blocked_by,
            "reason": self.reason,
            "can_override": self.can_override,
            "warnings": self.warnings,
        }


class SafetyPolicy:
    """
    Politicas de seguridad del tronco encefalico.
    
    Responsabilidades:
    - Verificar acciones contra reglas de seguridad
    - Implementar parada de emergencia
    - Gestionar degradacion controlada
    """
    
    def __init__(self):
        # Reglas de seguridad
        self.rules: List[SafetyRule] = []
        
        # Tokens de anulacion temporal
        self._override_tokens: Dict[str, float] = {}  # rule_id -> expiry_time
        
        # Estado de emergencia
        self._emergency_active = False
        self._emergency_reason: Optional[str] = None
        
        # Historial de bloqueos
        self._block_history: List[SafetyVerdict] = []
        
        # Callbacks
        self._on_block: List[Callable[[SafetyVerdict], None]] = []
        self._on_emergency: List[Callable[[str], None]] = []
        
        # Registrar reglas por defecto
        self._register_default_rules()
    
    def _register_default_rules(self) -> None:
        """Registra reglas de seguridad por defecto."""
        
        # === Reglas Absolutas (no anulables) ===
        
        # Emergency Stop
        self.add_rule(SafetyRule(
            id="emergency_stop",
            name="Emergency Stop",
            description="Bloquea todas las acciones durante emergency stop",
            condition=lambda a, c: c.get("emergency_stop", False),
            priority=1000,
            is_absolute=True,
            category="emergency",
        ))
        
        # Limites de articulaciones
        self.add_rule(SafetyRule(
            id="joint_limits",
            name="Joint Limits",
            description="Bloquea movimientos fuera de limites articulares",
            condition=self._check_joint_limits,
            priority=900,
            is_absolute=True,
            category="motion",
        ))
        
        # Colision inminente
        self.add_rule(SafetyRule(
            id="collision_imminent",
            name="Collision Imminent",
            description="Bloquea movimiento cuando hay obstaculo en trayectoria",
            condition=self._check_collision_imminent,
            priority=850,
            is_absolute=True,
            category="collision",
        ))
        
        # Limite de torque
        self.add_rule(SafetyRule(
            id="torque_limit",
            name="Torque Limit",
            description="Bloquea comandos que exceden torque maximo",
            condition=self._check_torque_limit,
            priority=800,
            is_absolute=True,
            category="motion",
        ))
        
        # === Reglas Anulables ===
        
        # Velocidad cerca de humanos
        self.add_rule(SafetyRule(
            id="human_proximity_speed",
            name="Human Proximity Speed",
            description="Limita velocidad cuando hay humanos cerca",
            condition=self._check_human_proximity_speed,
            priority=700,
            is_absolute=False,
            category="human_safety",
        ))
        
        # Bateria critica
        self.add_rule(SafetyRule(
            id="battery_critical",
            name="Battery Critical",
            description="Bloquea acciones de alto consumo con bateria critica",
            condition=self._check_battery_critical,
            priority=600,
            is_absolute=False,
            category="power",
        ))
        
        # Temperatura de motor alta
        self.add_rule(SafetyRule(
            id="motor_overheat",
            name="Motor Overheat",
            description="Bloquea uso de motores sobrecalentados",
            condition=self._check_motor_overheat,
            priority=550,
            is_absolute=False,
            category="thermal",
        ))
        
        # Accion destructiva sin confirmacion
        self.add_rule(SafetyRule(
            id="destructive_action",
            name="Destructive Action",
            description="Bloquea acciones destructivas sin confirmacion",
            condition=self._check_destructive_action,
            priority=500,
            is_absolute=False,
            category="general",
        ))
    
    def add_rule(self, rule: SafetyRule) -> None:
        """Agrega regla de seguridad."""
        self.rules.append(rule)
        # Ordenar por prioridad (mayor primero)
        self.rules.sort(key=lambda r: r.priority, reverse=True)
    
    def remove_rule(self, rule_id: str) -> bool:
        """Remueve regla por ID."""
        for i, rule in enumerate(self.rules):
            if rule.id == rule_id:
                if rule.is_absolute:
                    logger.warning(f"Cannot remove absolute rule: {rule_id}")
                    return False
                del self.rules[i]
                return True
        return False
    
    def check(self, action: Any, context: Dict[str, Any]) -> SafetyVerdict:
        """
        Verifica si una accion es segura.
        
        Args:
            action: Accion a verificar
            context: Contexto de ejecucion
        
        Returns:
            SafetyVerdict indicando si esta permitida
        """
        warnings = []
        
        # Limpiar overrides expirados
        self._cleanup_expired_overrides()
        
        # Verificar emergency stop primero
        if self._emergency_active:
            return SafetyVerdict(
                allowed=False,
                blocked_by="emergency_active",
                reason=self._emergency_reason or "Emergency stop active",
                can_override=False,
            )
        
        # Verificar cada regla
        for rule in self.rules:
            if rule.check(action, context):
                # Regla activada
                if rule.is_absolute:
                    # No se puede anular
                    verdict = SafetyVerdict(
                        allowed=False,
                        blocked_by=rule.id,
                        reason=rule.description,
                        can_override=False,
                        warnings=warnings,
                    )
                    self._record_block(verdict)
                    return verdict
                else:
                    # Se puede anular con override
                    if rule.id in self._override_tokens:
                        warnings.append(f"Override active for: {rule.name}")
                        continue
                    
                    verdict = SafetyVerdict(
                        allowed=False,
                        blocked_by=rule.id,
                        reason=rule.description,
                        can_override=True,
                        warnings=warnings,
                    )
                    self._record_block(verdict)
                    return verdict
        
        return SafetyVerdict(allowed=True, warnings=warnings)
    
    def _record_block(self, verdict: SafetyVerdict) -> None:
        """Registra bloqueo y notifica."""
        self._block_history.append(verdict)
        
        # Limitar historial
        if len(self._block_history) > 1000:
            self._block_history = self._block_history[-500:]
        
        logger.warning(f"Action blocked by {verdict.blocked_by}: {verdict.reason}")
        
        for callback in self._on_block:
            try:
                callback(verdict)
            except Exception as e:
                logger.error(f"Error in block callback: {e}")
    
    def set_override(self, rule_id: str, duration_s: float = 30.0) -> bool:
        """
        Establece override temporal para una regla.
        
        Args:
            rule_id: ID de la regla
            duration_s: Duracion del override en segundos
        
        Returns:
            True si se establecio el override
        """
        # Verificar que la regla existe y no es absoluta
        rule = next((r for r in self.rules if r.id == rule_id), None)
        if not rule:
            return False
        if rule.is_absolute:
            logger.warning(f"Cannot override absolute rule: {rule_id}")
            return False
        
        self._override_tokens[rule_id] = time.time() + duration_s
        logger.info(f"Override set for rule {rule_id}, duration={duration_s}s")
        return True
    
    def clear_override(self, rule_id: str) -> None:
        """Limpia override de una regla."""
        if rule_id in self._override_tokens:
            del self._override_tokens[rule_id]
    
    def _cleanup_expired_overrides(self) -> None:
        """Limpia overrides expirados."""
        now = time.time()
        expired = [rid for rid, expiry in self._override_tokens.items() if expiry < now]
        for rid in expired:
            del self._override_tokens[rid]
    
    async def emergency_stop(self, reason: str = "Manual emergency stop") -> None:
        """
        Activa parada de emergencia.
        
        Args:
            reason: Razon de la parada
        """
        self._emergency_active = True
        self._emergency_reason = reason
        
        logger.critical(f"EMERGENCY STOP ACTIVATED: {reason}")
        
        for callback in self._on_emergency:
            try:
                callback(reason)
            except Exception as e:
                logger.error(f"Error in emergency callback: {e}")
    
    async def clear_emergency(self) -> None:
        """Limpia estado de emergencia."""
        if self._emergency_active:
            logger.info("Emergency stop cleared")
            self._emergency_active = False
            self._emergency_reason = None
    
    def is_emergency_active(self) -> bool:
        """Verifica si hay emergencia activa."""
        return self._emergency_active
    
    # === Funciones de Verificacion de Reglas ===
    
    def _check_joint_limits(self, action: Any, context: Dict[str, Any]) -> bool:
        """Verifica limites articulares."""
        if not hasattr(action, "action_type"):
            return False
        
        action_type = getattr(action, "action_type", "")
        if action_type not in ("move", "position", "joint_position"):
            return False
        
        params = getattr(action, "parameters", {}) or {}
        joint_id = params.get("joint_id", "")
        position = params.get("position")
        
        if position is None:
            return False
        
        limits = context.get("joint_limits", {})
        if joint_id in limits:
            min_pos, max_pos = limits[joint_id]
            if position < min_pos or position > max_pos:
                return True
        
        return False
    
    def _check_collision_imminent(self, action: Any, context: Dict[str, Any]) -> bool:
        """Verifica colision inminente."""
        obstacles = context.get("obstacles_in_path", [])
        
        for obstacle in obstacles:
            distance = obstacle.get("distance", float("inf"))
            if distance < 0.1:  # Menos de 10cm
                return True
        
        return False
    
    def _check_torque_limit(self, action: Any, context: Dict[str, Any]) -> bool:
        """Verifica limite de torque."""
        params = getattr(action, "parameters", {}) or {}
        torque = params.get("torque")
        
        if torque is None:
            return False
        
        max_torque = context.get("max_torque", 50)  # Nm
        return abs(torque) > max_torque
    
    def _check_human_proximity_speed(self, action: Any, context: Dict[str, Any]) -> bool:
        """Verifica velocidad cerca de humanos."""
        human_distance = context.get("human_distance", float("inf"))
        
        if human_distance > 1.5:
            return False
        
        params = getattr(action, "parameters", {}) or {}
        speed = params.get("speed", params.get("velocity", 0))
        
        # Velocidad maxima segun distancia
        max_speed = 0.5 if human_distance < 0.5 else 1.0
        
        return speed > max_speed
    
    def _check_battery_critical(self, action: Any, context: Dict[str, Any]) -> bool:
        """Verifica bateria critica."""
        battery = context.get("battery_percent", 100)
        
        if battery > 10:
            return False
        
        # Acciones de alto consumo
        high_consumption = {"navigate", "run", "lift", "carry"}
        action_type = getattr(action, "action_type", "")
        
        return action_type.lower() in high_consumption
    
    def _check_motor_overheat(self, action: Any, context: Dict[str, Any]) -> bool:
        """Verifica sobrecalentamiento de motor."""
        motor_temps = context.get("motor_temps", {})
        
        # Si hay motor sobre 70C, bloquear uso de ese motor
        params = getattr(action, "parameters", {}) or {}
        motor_id = params.get("motor_id", params.get("joint_id", ""))
        
        if motor_id and motor_id in motor_temps:
            return motor_temps[motor_id] > 70
        
        return False
    
    def _check_destructive_action(self, action: Any, context: Dict[str, Any]) -> bool:
        """Verifica accion destructiva sin confirmacion."""
        destructive_types = {"delete", "remove", "destroy", "reset", "format"}
        
        action_type = getattr(action, "action_type", "")
        is_destructive = action_type.lower() in destructive_types
        has_confirmation = context.get("confirmed", False)
        
        return is_destructive and not has_confirmation
    
    # === Callbacks ===
    
    def on_block(self, callback: Callable[[SafetyVerdict], None]) -> None:
        """Registra callback para bloqueos."""
        self._on_block.append(callback)
    
    def on_emergency(self, callback: Callable[[str], None]) -> None:
        """Registra callback para emergencias."""
        self._on_emergency.append(callback)
    
    # === Estadisticas ===
    
    def get_stats(self) -> Dict[str, Any]:
        """Obtiene estadisticas."""
        blocks_by_rule = {}
        for verdict in self._block_history:
            rule_id = verdict.blocked_by or "unknown"
            blocks_by_rule[rule_id] = blocks_by_rule.get(rule_id, 0) + 1
        
        return {
            "rules_count": len(self.rules),
            "absolute_rules": sum(1 for r in self.rules if r.is_absolute),
            "active_overrides": len(self._override_tokens),
            "emergency_active": self._emergency_active,
            "total_blocks": len(self._block_history),
            "blocks_by_rule": blocks_by_rule,
        }

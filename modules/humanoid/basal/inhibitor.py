"""
Inhibitor: Control inhibitorio de acciones (Ganglios Basales - Globo Palido).

Implementa el mecanismo Go/NoGo para bloquear acciones no deseadas.
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Set
from enum import Enum

logger = logging.getLogger(__name__)


def _bitacora(msg: str, ok: bool = True) -> None:
    try:
        from modules.humanoid.ans.evolution_bitacora import append_evolution_log
        append_evolution_log(msg, ok=ok, source="basal")
    except Exception:
        pass


class InhibitionLevel(int, Enum):
    """Niveles de inhibicion."""
    NONE = 0          # Sin inhibicion
    SOFT = 1          # Advertencia, permite override
    MODERATE = 2      # Requiere confirmacion
    STRONG = 3        # Solo override de seguridad
    ABSOLUTE = 4      # No se puede ignorar


class InhibitionSource(str, Enum):
    """Fuentes de inhibicion."""
    SAFETY = "safety"          # Politica de seguridad
    GOAL = "goal"              # Conflicto con objetivo
    RESOURCE = "resource"      # Recurso no disponible
    CONTEXT = "context"        # Contexto inapropiado
    LEARNED = "learned"        # Aprendido de feedback
    USER = "user"              # Prohibicion explicita
    SYSTEM = "system"          # Sistema


@dataclass
class InhibitionRule:
    """Regla de inhibicion."""
    id: str
    name: str
    description: str
    source: InhibitionSource
    level: InhibitionLevel
    
    # Funcion de evaluacion
    condition: Callable[[Any, Dict], bool]
    
    # Configuracion
    enabled: bool = True
    expires_at_ns: Optional[int] = None  # Expiracion temporal
    
    # Estadisticas
    times_triggered: int = 0
    last_triggered_ns: Optional[int] = None
    
    def check(self, action: Any, context: Dict[str, Any]) -> bool:
        """Evalua si la regla aplica."""
        if not self.enabled:
            return False
        
        # Verificar expiracion
        if self.expires_at_ns and time.time_ns() > self.expires_at_ns:
            self.enabled = False
            return False
        
        try:
            result = self.condition(action, context)
            if result:
                self.times_triggered += 1
                self.last_triggered_ns = time.time_ns()
            return result
        except Exception as e:
            logger.error(f"Error evaluating inhibition rule {self.id}: {e}")
            return False


@dataclass
class InhibitionVerdict:
    """Resultado de evaluacion de inhibicion."""
    go: bool                    # True = permitido, False = bloqueado
    level: InhibitionLevel
    triggered_rules: List[str]
    warnings: List[str] = field(default_factory=list)
    can_override: bool = False
    override_requires: Optional[str] = None
    reasoning: str = ""


@dataclass 
class Override:
    """Token de override."""
    id: str
    rule_id: str
    granted_by: str
    reason: str
    created_at_ns: int
    expires_at_ns: int
    uses_remaining: int = -1  # -1 = ilimitado


class Inhibitor:
    """
    Sistema de inhibicion Go/NoGo.
    
    Evalua acciones propuestas contra reglas de inhibicion
    y decide si permitir (Go) o bloquear (NoGo).
    """
    
    def __init__(self):
        """Inicializa el inhibidor."""
        self._rules: Dict[str, InhibitionRule] = {}
        self._overrides: Dict[str, Override] = {}
        
        # Historial
        self._verdicts: List[InhibitionVerdict] = []
        
        # Estadisticas
        self._total_checks = 0
        self._total_blocks = 0
        self._total_overrides_used = 0
        
        # Callbacks
        self._on_block_callbacks: List[Callable[[InhibitionVerdict], None]] = []
        
        # Inicializar reglas default
        self._init_default_rules()
    
    def _init_default_rules(self) -> None:
        """Inicializa reglas de inhibicion por defecto."""
        # Regla: Emergency stop activo
        self.add_rule(InhibitionRule(
            id="emergency_stop",
            name="Emergency Stop",
            description="Sistema en parada de emergencia",
            source=InhibitionSource.SAFETY,
            level=InhibitionLevel.ABSOLUTE,
            condition=lambda a, c: c.get("emergency_active", False),
        ))
        
        # Regla: Bateria critica
        self.add_rule(InhibitionRule(
            id="low_battery",
            name="Low Battery",
            description="Bateria demasiado baja para operacion",
            source=InhibitionSource.SAFETY,
            level=InhibitionLevel.STRONG,
            condition=lambda a, c: c.get("battery_percent", 100) < 5,
        ))
        
        # Regla: Temperatura critica
        self.add_rule(InhibitionRule(
            id="high_temp",
            name="High Temperature",
            description="Temperatura de motor demasiado alta",
            source=InhibitionSource.SAFETY,
            level=InhibitionLevel.STRONG,
            condition=lambda a, c: any(
                t > 80 for t in c.get("motor_temps", {}).values()
            ),
        ))
        
        # Regla: Humano muy cerca
        self.add_rule(InhibitionRule(
            id="human_proximity",
            name="Human Too Close",
            description="Humano demasiado cerca para movimientos rapidos",
            source=InhibitionSource.SAFETY,
            level=InhibitionLevel.MODERATE,
            condition=lambda a, c: (
                c.get("nearest_human_distance", float("inf")) < 0.3 and
                getattr(a, "action_type", None) in ("move", "grasp", "navigate")
            ),
        ))
        
        # Regla: Conflicto con objetivo actual
        self.add_rule(InhibitionRule(
            id="goal_conflict",
            name="Goal Conflict",
            description="Accion conflictua con objetivo activo",
            source=InhibitionSource.GOAL,
            level=InhibitionLevel.SOFT,
            condition=lambda a, c: self._check_goal_conflict(a, c),
        ))
    
    def _check_goal_conflict(self, action: Any, context: Dict[str, Any]) -> bool:
        """Verifica conflicto con objetivo."""
        current_goal = context.get("current_goal_type")
        if not current_goal:
            return False
        
        # Ejemplos de conflictos
        conflicts = {
            "fetch": ["place", "drop"],  # No soltar mientras buscas
            "navigate": ["grasp"],        # No agarrar mientras navegas
            "speak": ["navigate"],        # No navegar mientras hablas
        }
        
        action_type = getattr(action, "action_type", None)
        if action_type:
            action_type = action_type.value if hasattr(action_type, "value") else str(action_type)
            return action_type in conflicts.get(current_goal, [])
        
        return False
    
    def add_rule(self, rule: InhibitionRule) -> None:
        """Agrega regla de inhibicion."""
        self._rules[rule.id] = rule
        logger.debug(f"Added inhibition rule: {rule.id}")
    
    def remove_rule(self, rule_id: str) -> bool:
        """Elimina regla de inhibicion."""
        if rule_id in self._rules:
            del self._rules[rule_id]
            return True
        return False
    
    def enable_rule(self, rule_id: str) -> bool:
        """Habilita regla."""
        if rule_id in self._rules:
            self._rules[rule_id].enabled = True
            return True
        return False
    
    def disable_rule(self, rule_id: str) -> bool:
        """Deshabilita regla."""
        if rule_id in self._rules:
            self._rules[rule_id].enabled = False
            return True
        return False
    
    def check(self, action: Any, context: Dict[str, Any]) -> InhibitionVerdict:
        """
        Evalua si una accion debe ser inhibida.
        
        Args:
            action: Accion a evaluar
            context: Contexto actual
        
        Returns:
            Veredicto Go/NoGo
        """
        self._total_checks += 1
        
        triggered_rules = []
        warnings = []
        max_level = InhibitionLevel.NONE
        
        # Evaluar todas las reglas
        for rule_id, rule in self._rules.items():
            if rule.check(action, context):
                # Verificar override
                if self._has_valid_override(rule_id):
                    warnings.append(f"Override activo para: {rule.name}")
                    self._total_overrides_used += 1
                    continue
                
                triggered_rules.append(rule_id)
                
                if rule.level.value > max_level.value:
                    max_level = rule.level
        
        # Determinar veredicto
        go = len(triggered_rules) == 0 or max_level == InhibitionLevel.NONE
        can_override = max_level.value < InhibitionLevel.ABSOLUTE.value
        
        # Bitácora
        if not go:
            for rule_id in triggered_rules:
                rule = self._rules.get(rule_id)
                reason = rule.name if rule else rule_id
                _bitacora(f"Inhibición: {rule_id} — {reason}", ok=False)
        
        # Determinar que se necesita para override
        override_requires = None
        if not go and can_override:
            if max_level == InhibitionLevel.SOFT:
                override_requires = "user_confirmation"
            elif max_level == InhibitionLevel.MODERATE:
                override_requires = "supervisor_approval"
            elif max_level == InhibitionLevel.STRONG:
                override_requires = "safety_override_token"
        
        # Construir razonamiento
        if go:
            reasoning = "Accion permitida"
            if warnings:
                reasoning += f" (con advertencias: {len(warnings)})"
        else:
            rule_names = [self._rules[r].name for r in triggered_rules]
            reasoning = f"Bloqueado por: {', '.join(rule_names)}"
        
        verdict = InhibitionVerdict(
            go=go,
            level=max_level,
            triggered_rules=triggered_rules,
            warnings=warnings,
            can_override=can_override,
            override_requires=override_requires,
            reasoning=reasoning,
        )
        
        # Callbacks
        if not go:
            self._total_blocks += 1
            for callback in self._on_block_callbacks:
                try:
                    callback(verdict)
                except Exception as e:
                    logger.error(f"Error in on_block callback: {e}")
        
        # Guardar historial
        self._verdicts.append(verdict)
        if len(self._verdicts) > 1000:
            self._verdicts = self._verdicts[-500:]
        
        return verdict
    
    def _has_valid_override(self, rule_id: str) -> bool:
        """Verifica si hay override valido para regla."""
        override = self._overrides.get(rule_id)
        if not override:
            return False
        
        # Verificar expiracion
        if time.time_ns() > override.expires_at_ns:
            del self._overrides[rule_id]
            return False
        
        # Verificar usos
        if override.uses_remaining == 0:
            del self._overrides[rule_id]
            return False
        
        # Decrementar usos si es limitado
        if override.uses_remaining > 0:
            override.uses_remaining -= 1
        
        return True
    
    def grant_override(
        self,
        rule_id: str,
        granted_by: str,
        reason: str,
        duration_seconds: float = 60.0,
        uses: int = -1,
    ) -> Optional[Override]:
        """
        Otorga override temporal para una regla.
        
        Args:
            rule_id: ID de la regla
            granted_by: Quien otorga el override
            reason: Razon del override
            duration_seconds: Duracion en segundos
            uses: Numero de usos (-1 = ilimitado)
        
        Returns:
            Token de override o None si no es posible
        """
        if rule_id not in self._rules:
            logger.warning(f"Cannot override unknown rule: {rule_id}")
            return None
        
        rule = self._rules[rule_id]
        
        # No se puede hacer override de reglas absolutas
        if rule.level == InhibitionLevel.ABSOLUTE:
            logger.warning(f"Cannot override ABSOLUTE rule: {rule_id}")
            return None
        
        import uuid
        override = Override(
            id=f"ovr_{uuid.uuid4().hex[:8]}",
            rule_id=rule_id,
            granted_by=granted_by,
            reason=reason,
            created_at_ns=time.time_ns(),
            expires_at_ns=time.time_ns() + int(duration_seconds * 1e9),
            uses_remaining=uses,
        )
        
        self._overrides[rule_id] = override
        logger.info(f"Override granted for {rule_id} by {granted_by}: {reason}")
        
        return override
    
    def revoke_override(self, rule_id: str) -> bool:
        """Revoca override para una regla."""
        if rule_id in self._overrides:
            del self._overrides[rule_id]
            return True
        return False
    
    def add_learned_inhibition(
        self,
        action_pattern: str,
        reason: str,
        level: InhibitionLevel = InhibitionLevel.SOFT,
        duration_seconds: Optional[float] = None,
    ) -> str:
        """
        Agrega inhibicion aprendida de feedback.
        
        Args:
            action_pattern: Patron de accion a inhibir
            reason: Razon de la inhibicion
            level: Nivel de inhibicion
            duration_seconds: Duracion (None = permanente)
        
        Returns:
            ID de la regla creada
        """
        import uuid
        rule_id = f"learned_{uuid.uuid4().hex[:8]}"
        
        expires = None
        if duration_seconds:
            expires = time.time_ns() + int(duration_seconds * 1e9)
        
        pattern_lower = action_pattern.lower()
        
        rule = InhibitionRule(
            id=rule_id,
            name=f"Learned: {action_pattern[:30]}",
            description=reason,
            source=InhibitionSource.LEARNED,
            level=level,
            condition=lambda a, c, p=pattern_lower: (
                p in str(getattr(a, "description", "")).lower() or
                p in str(getattr(a, "action_type", "")).lower()
            ),
            expires_at_ns=expires,
        )
        
        self.add_rule(rule)
        logger.info(f"Added learned inhibition: {rule_id} for '{action_pattern}'")
        
        return rule_id
    
    def add_user_prohibition(
        self,
        action_pattern: str,
        user_id: str = "user",
    ) -> str:
        """Agrega prohibicion explicita del usuario."""
        import uuid
        rule_id = f"user_{uuid.uuid4().hex[:8]}"
        
        pattern_lower = action_pattern.lower()
        
        rule = InhibitionRule(
            id=rule_id,
            name=f"User prohibition: {action_pattern[:30]}",
            description=f"Prohibido por usuario: {user_id}",
            source=InhibitionSource.USER,
            level=InhibitionLevel.STRONG,
            condition=lambda a, c, p=pattern_lower: (
                p in str(getattr(a, "description", "")).lower() or
                p in str(getattr(a, "id", "")).lower()
            ),
        )
        
        self.add_rule(rule)
        return rule_id
    
    def on_block(self, callback: Callable[[InhibitionVerdict], None]) -> None:
        """Registra callback cuando se bloquea una accion."""
        self._on_block_callbacks.append(callback)
    
    def get_active_rules(self) -> List[Dict[str, Any]]:
        """Obtiene reglas activas."""
        return [
            {
                "id": r.id,
                "name": r.name,
                "source": r.source.value,
                "level": r.level.name,
                "enabled": r.enabled,
                "times_triggered": r.times_triggered,
            }
            for r in self._rules.values()
            if r.enabled
        ]
    
    def get_active_overrides(self) -> List[Dict[str, Any]]:
        """Obtiene overrides activos."""
        now = time.time_ns()
        return [
            {
                "id": o.id,
                "rule_id": o.rule_id,
                "granted_by": o.granted_by,
                "reason": o.reason,
                "remaining_seconds": (o.expires_at_ns - now) / 1e9,
                "uses_remaining": o.uses_remaining,
            }
            for o in self._overrides.values()
            if now < o.expires_at_ns
        ]
    
    def get_stats(self) -> Dict[str, Any]:
        """Obtiene estadisticas."""
        return {
            "total_checks": self._total_checks,
            "total_blocks": self._total_blocks,
            "block_rate": self._total_blocks / max(1, self._total_checks),
            "total_overrides_used": self._total_overrides_used,
            "active_rules": len([r for r in self._rules.values() if r.enabled]),
            "active_overrides": len(self._overrides),
        }

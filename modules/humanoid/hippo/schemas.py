"""
Schemas para el Hipocampo Atlas.

Define estructuras de datos para memorias episodicas y semanticas.
"""
from __future__ import annotations

import uuid
import time
from dataclasses import dataclass, field, asdict
from datetime import datetime
from typing import Any, Dict, List, Optional
from enum import Enum


class Outcome(Enum):
    """Resultado de un episodio."""
    SUCCESS = "success"
    FAILURE = "failure"
    PARTIAL = "partial"
    INTERRUPTED = "interrupted"


@dataclass
class ActionRecord:
    """Registro de una accion dentro de un episodio."""
    action_type: str
    parameters: Dict[str, Any] = field(default_factory=dict)
    start_time_ns: int = field(default_factory=lambda: time.time_ns())
    end_time_ns: Optional[int] = None
    result: str = "pending"  # success, failure, interrupted
    error: Optional[str] = None
    observations: Dict[str, Any] = field(default_factory=dict)
    
    def duration_ms(self) -> float:
        if self.end_time_ns:
            return (self.end_time_ns - self.start_time_ns) / 1e6
        return 0.0
    
    def complete(self, success: bool, error: str = None) -> None:
        self.end_time_ns = time.time_ns()
        self.result = "success" if success else "failure"
        self.error = error
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "action_type": self.action_type,
            "parameters": self.parameters,
            "start_time_ns": self.start_time_ns,
            "end_time_ns": self.end_time_ns,
            "result": self.result,
            "error": self.error,
            "duration_ms": self.duration_ms(),
        }


@dataclass
class WorldStateSnapshot:
    """Snapshot del estado del mundo."""
    timestamp_ns: int = field(default_factory=lambda: time.time_ns())
    robot_position: Optional[tuple] = None
    objects: List[Dict[str, Any]] = field(default_factory=list)
    humans: List[Dict[str, Any]] = field(default_factory=list)
    internal_state: Dict[str, float] = field(default_factory=dict)
    
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class InternalStateSnapshot:
    """Snapshot del estado interno."""
    energy_level: float = 1.0
    stress_level: float = 0.2
    focus_level: float = 0.8
    mode: str = "normal"
    confidence: float = 0.8
    
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class Episode:
    """
    Representa una experiencia episodica.
    
    Un episodio es una secuencia de acciones con contexto,
    objetivo y resultado.
    """
    id: str = field(default_factory=lambda: f"ep_{uuid.uuid4().hex[:12]}")
    
    # Timestamps
    timestamp_ns: int = field(default_factory=lambda: time.time_ns())
    end_timestamp_ns: Optional[int] = None
    
    # Contexto inicial
    initial_state: Optional[WorldStateSnapshot] = None
    goal: str = ""
    goal_type: str = ""
    internal_state: Optional[InternalStateSnapshot] = None
    
    # Secuencia de acciones
    actions: List[ActionRecord] = field(default_factory=list)
    
    # Resultado
    final_state: Optional[WorldStateSnapshot] = None
    outcome: Outcome = Outcome.PARTIAL
    reward: float = 0.0
    
    # Metadatos
    duration_ms: float = 0.0
    tags: List[str] = field(default_factory=list)
    importance: float = 0.5  # 0-1, para priorizar consolidacion
    times_recalled: int = 0  # Se incrementa al recuperar
    
    # Embedding para busqueda semantica
    embedding: Optional[List[float]] = None  # Vector de embedding
    
    # Contexto adicional
    context: Dict[str, Any] = field(default_factory=dict)
    
    def add_action(self, action: ActionRecord) -> None:
        """Agrega accion al episodio."""
        self.actions.append(action)
    
    def complete(self, outcome: Outcome, reward: float = 0.0) -> None:
        """Marca episodio como completado."""
        self.end_timestamp_ns = time.time_ns()
        self.duration_ms = (self.end_timestamp_ns - self.timestamp_ns) / 1e6
        self.outcome = outcome
        self.reward = reward
    
    def success_rate(self) -> float:
        """Calcula tasa de exito de acciones."""
        if not self.actions:
            return 0.0
        successful = sum(1 for a in self.actions if a.result == "success")
        return successful / len(self.actions)
    
    def increment_recall(self) -> None:
        """Incrementa contador de recuperacion."""
        self.times_recalled += 1
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "timestamp_ns": self.timestamp_ns,
            "goal": self.goal,
            "goal_type": self.goal_type,
            "outcome": self.outcome.value,
            "reward": self.reward,
            "duration_ms": self.duration_ms,
            "actions_count": len(self.actions),
            "success_rate": self.success_rate(),
            "tags": self.tags,
            "importance": self.importance,
            "times_recalled": self.times_recalled,
        }
    
    def to_full_dict(self) -> Dict[str, Any]:
        """Convierte a diccionario completo incluyendo acciones."""
        d = self.to_dict()
        d["actions"] = [a.to_dict() for a in self.actions]
        d["initial_state"] = self.initial_state.to_dict() if self.initial_state else None
        d["final_state"] = self.final_state.to_dict() if self.final_state else None
        d["internal_state"] = self.internal_state.to_dict() if self.internal_state else None
        d["context"] = self.context
        return d


@dataclass
class Relation:
    """Relacion entre conceptos en el grafo de conocimiento."""
    relation_type: str  # is_a, part_of, can_do, located_at, causes, etc.
    target_id: str
    confidence: float = 1.0
    evidence_count: int = 1
    last_updated_ns: int = field(default_factory=lambda: time.time_ns())
    properties: Dict[str, Any] = field(default_factory=dict)
    
    def strengthen(self, amount: float = 0.1) -> None:
        """Fortalece la relacion."""
        self.confidence = min(1.0, self.confidence + amount)
        self.evidence_count += 1
        self.last_updated_ns = time.time_ns()
    
    def weaken(self, amount: float = 0.1) -> None:
        """Debilita la relacion."""
        self.confidence = max(0.0, self.confidence - amount)
        self.last_updated_ns = time.time_ns()
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "relation_type": self.relation_type,
            "target_id": self.target_id,
            "confidence": self.confidence,
            "evidence_count": self.evidence_count,
        }


@dataclass
class Concept:
    """
    Nodo en el grafo de conocimiento semantico.
    
    Representa un concepto (objeto, accion, lugar, persona, propiedad).
    """
    id: str = field(default_factory=lambda: f"concept_{uuid.uuid4().hex[:12]}")
    name: str = ""
    concept_type: str = "object"  # object, action, place, person, property, event
    
    # Propiedades del concepto
    properties: Dict[str, Any] = field(default_factory=dict)
    
    # Relaciones con otros conceptos
    relations: List[Relation] = field(default_factory=list)
    
    # Embedding para busqueda semantica
    embedding: Optional[List[float]] = None
    
    # Metadatos
    confidence: float = 1.0
    source: str = "initial"  # initial, learned, told
    created_at_ns: int = field(default_factory=lambda: time.time_ns())
    last_updated_ns: int = field(default_factory=lambda: time.time_ns())
    access_count: int = 0
    
    def add_relation(self, relation_type: str, target_id: str, 
                    confidence: float = 1.0) -> Relation:
        """Agrega relacion a otro concepto."""
        # Verificar si ya existe
        for rel in self.relations:
            if rel.relation_type == relation_type and rel.target_id == target_id:
                rel.strengthen()
                return rel
        
        # Crear nueva relacion
        relation = Relation(
            relation_type=relation_type,
            target_id=target_id,
            confidence=confidence,
        )
        self.relations.append(relation)
        return relation
    
    def get_relations(self, relation_type: str = None) -> List[Relation]:
        """Obtiene relaciones, opcionalmente filtradas por tipo."""
        if relation_type:
            return [r for r in self.relations if r.relation_type == relation_type]
        return self.relations
    
    def remove_relation(self, relation_type: str, target_id: str) -> bool:
        """Remueve una relacion."""
        for i, rel in enumerate(self.relations):
            if rel.relation_type == relation_type and rel.target_id == target_id:
                del self.relations[i]
                return True
        return False
    
    def update_property(self, key: str, value: Any) -> None:
        """Actualiza propiedad del concepto."""
        self.properties[key] = value
        self.last_updated_ns = time.time_ns()
    
    def access(self) -> None:
        """Registra acceso al concepto."""
        self.access_count += 1
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "concept_type": self.concept_type,
            "properties": self.properties,
            "relations_count": len(self.relations),
            "confidence": self.confidence,
            "source": self.source,
            "access_count": self.access_count,
        }
    
    def to_full_dict(self) -> Dict[str, Any]:
        """Convierte a diccionario completo incluyendo relaciones."""
        d = self.to_dict()
        d["relations"] = [r.to_dict() for r in self.relations]
        return d

"""
Consolidator: Consolidacion de memorias del hipocampo.

Analogo biologico: Proceso de consolidacion durante el sueno
- Transferencia de episodios a conocimiento semantico
- Generalizacion de experiencias
- Fortalecimiento de memorias importantes
"""
from __future__ import annotations

import asyncio
import logging
import time
from collections import defaultdict
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional

from .schemas import Episode, Concept, Relation, Outcome
from .episodic_memory import EpisodicMemory
from .semantic_memory import SemanticMemory

logger = logging.getLogger(__name__)


@dataclass
class ConsolidationResult:
    """Resultado de un ciclo de consolidacion."""
    timestamp_ns: int
    episodes_processed: int
    concepts_created: int
    concepts_updated: int
    relations_added: int
    patterns_found: int
    duration_ms: float


class Consolidator:
    """
    Consolidador de memorias del hipocampo.
    
    Responsabilidades:
    - Extraer patrones de episodios
    - Crear/actualizar conceptos semanticos
    - Fortalecer memorias importantes
    - Ejecutar consolidacion periodica
    """
    
    def __init__(self,
                 episodic_memory: EpisodicMemory,
                 semantic_memory: SemanticMemory,
                 consolidation_interval_hours: float = 4.0,
                 min_episodes_for_pattern: int = 3):
        """
        Inicializa el consolidador.
        
        Args:
            episodic_memory: Memoria episodica
            semantic_memory: Memoria semantica
            consolidation_interval_hours: Intervalo entre consolidaciones
            min_episodes_for_pattern: Minimo de episodios para detectar patron
        """
        self.episodic_memory = episodic_memory
        self.semantic_memory = semantic_memory
        self.consolidation_interval_hours = consolidation_interval_hours
        self.min_episodes_for_pattern = min_episodes_for_pattern
        
        # Estado
        self._last_consolidation_ns: int = 0
        self._consolidation_history: List[ConsolidationResult] = []
        
        # Callbacks
        self._on_consolidation_complete: List[Callable[[ConsolidationResult], None]] = []
        
        # Patrones detectados
        self._action_patterns: Dict[str, int] = defaultdict(int)
        self._goal_success_rates: Dict[str, Dict[str, int]] = defaultdict(lambda: {"success": 0, "total": 0})
    
    async def consolidate(self, force: bool = False) -> Optional[ConsolidationResult]:
        """
        Ejecuta ciclo de consolidacion.
        
        Args:
            force: Forzar consolidacion aunque no haya pasado el intervalo
        
        Returns:
            Resultado de la consolidacion o None si no se ejecuto
        """
        # Verificar si es momento de consolidar
        if not force:
            elapsed_hours = (time.time_ns() - self._last_consolidation_ns) / (3600 * 1e9)
            if elapsed_hours < self.consolidation_interval_hours:
                return None
        
        logger.info("Starting memory consolidation")
        start_time = time.time()
        
        # Contadores
        episodes_processed = 0
        concepts_created = 0
        concepts_updated = 0
        relations_added = 0
        patterns_found = 0
        
        # Obtener episodios recientes (desde ultima consolidacion)
        recent_episodes = self._get_episodes_since_last_consolidation()
        
        for episode in recent_episodes:
            # Procesar episodio
            result = self._process_episode(episode)
            
            episodes_processed += 1
            concepts_created += result.get("concepts_created", 0)
            concepts_updated += result.get("concepts_updated", 0)
            relations_added += result.get("relations_added", 0)
        
        # Detectar patrones
        patterns = self._detect_patterns()
        patterns_found = len(patterns)
        
        # Crear conceptos de patrones
        for pattern in patterns:
            self._create_pattern_concept(pattern)
            concepts_created += 1
        
        # Fortalecer memorias importantes
        self._strengthen_important_memories()
        
        # Actualizar timestamp
        self._last_consolidation_ns = time.time_ns()
        
        # Crear resultado
        duration_ms = (time.time() - start_time) * 1000
        result = ConsolidationResult(
            timestamp_ns=self._last_consolidation_ns,
            episodes_processed=episodes_processed,
            concepts_created=concepts_created,
            concepts_updated=concepts_updated,
            relations_added=relations_added,
            patterns_found=patterns_found,
            duration_ms=duration_ms,
        )
        
        self._consolidation_history.append(result)
        
        logger.info(f"Consolidation complete: {episodes_processed} episodes, "
                   f"{concepts_created} concepts created, {patterns_found} patterns")
        
        # Notificar
        for callback in self._on_consolidation_complete:
            try:
                callback(result)
            except Exception as e:
                logger.error(f"Error in consolidation callback: {e}")
        
        return result
    
    def _get_episodes_since_last_consolidation(self) -> List[Episode]:
        """Obtiene episodios desde la ultima consolidacion."""
        all_episodes = self.episodic_memory.get_recent(1000)
        
        if not self._last_consolidation_ns:
            return all_episodes
        
        return [
            ep for ep in all_episodes
            if ep.timestamp_ns > self._last_consolidation_ns
        ]
    
    def _process_episode(self, episode: Episode) -> Dict[str, int]:
        """
        Procesa un episodio para extraer conocimiento.
        
        Returns:
            Contadores de cambios
        """
        result = {
            "concepts_created": 0,
            "concepts_updated": 0,
            "relations_added": 0,
        }
        
        # Actualizar tasas de exito por tipo de objetivo
        goal_type = episode.goal_type or "unknown"
        self._goal_success_rates[goal_type]["total"] += 1
        if episode.outcome == Outcome.SUCCESS:
            self._goal_success_rates[goal_type]["success"] += 1
        
        # Extraer entidades del episodio
        entities = self._extract_entities(episode)
        
        for entity_name, entity_type in entities:
            # Verificar si el concepto existe
            existing = self.semantic_memory.get_by_name(entity_name)
            
            if existing:
                # Actualizar concepto existente
                existing.access()
                result["concepts_updated"] += 1
            else:
                # Crear nuevo concepto
                concept = self.semantic_memory.create_concept(
                    name=entity_name,
                    concept_type=entity_type,
                    source="learned",
                )
                result["concepts_created"] += 1
        
        # Extraer patrones de acciones
        for action in episode.actions:
            action_key = f"{action.action_type}:{action.result}"
            self._action_patterns[action_key] += 1
        
        return result
    
    def _extract_entities(self, episode: Episode) -> List[tuple]:
        """Extrae entidades (nombre, tipo) de un episodio."""
        entities = []
        
        # Extraer del objetivo
        goal_words = episode.goal.split()
        for word in goal_words:
            word_lower = word.lower().strip(".,!?")
            if len(word_lower) > 2:
                # Heuristica simple para detectar objetos
                entities.append((word_lower, "object"))
        
        # Extraer de parametros de acciones
        for action in episode.actions:
            for key, value in action.parameters.items():
                if key in ("object", "target", "location") and isinstance(value, str):
                    entity_type = "location" if key == "location" else "object"
                    entities.append((value.lower(), entity_type))
        
        # Eliminar duplicados
        return list(set(entities))
    
    def _detect_patterns(self) -> List[Dict[str, Any]]:
        """Detecta patrones en las acciones."""
        patterns = []
        
        # Patrones de accion frecuentes
        for action_key, count in self._action_patterns.items():
            if count >= self.min_episodes_for_pattern:
                action_type, result = action_key.split(":")
                patterns.append({
                    "type": "action_pattern",
                    "action_type": action_type,
                    "result": result,
                    "count": count,
                })
        
        # Patrones de exito/fracaso por tipo de objetivo
        for goal_type, stats in self._goal_success_rates.items():
            total = stats["total"]
            if total >= self.min_episodes_for_pattern:
                success_rate = stats["success"] / total
                patterns.append({
                    "type": "goal_success_pattern",
                    "goal_type": goal_type,
                    "success_rate": success_rate,
                    "total": total,
                })
        
        return patterns
    
    def _create_pattern_concept(self, pattern: Dict[str, Any]) -> None:
        """Crea concepto semantico a partir de un patron."""
        pattern_type = pattern.get("type")
        
        if pattern_type == "action_pattern":
            name = f"pattern_{pattern['action_type']}_{pattern['result']}"
            existing = self.semantic_memory.get_by_name(name)
            
            if existing:
                # Actualizar frecuencia
                existing.update_property("frequency", pattern["count"])
            else:
                self.semantic_memory.create_concept(
                    name=name,
                    concept_type="pattern",
                    properties={
                        "action_type": pattern["action_type"],
                        "typical_result": pattern["result"],
                        "frequency": pattern["count"],
                    },
                    source="consolidation",
                )
        
        elif pattern_type == "goal_success_pattern":
            name = f"success_rate_{pattern['goal_type']}"
            existing = self.semantic_memory.get_by_name(name)
            
            if existing:
                existing.update_property("success_rate", pattern["success_rate"])
                existing.update_property("sample_size", pattern["total"])
            else:
                self.semantic_memory.create_concept(
                    name=name,
                    concept_type="metric",
                    properties={
                        "goal_type": pattern["goal_type"],
                        "success_rate": pattern["success_rate"],
                        "sample_size": pattern["total"],
                    },
                    source="consolidation",
                )
    
    def _strengthen_important_memories(self) -> None:
        """Fortalece memorias importantes."""
        # Las memorias importantes son:
        # 1. Episodios con alta recompensa
        # 2. Episodios frecuentemente recordados
        # 3. Episodios recientes
        
        all_episodes = self.episodic_memory.get_recent(100)
        
        for episode in all_episodes:
            # Calcular nueva importancia
            base_importance = 0.5
            
            # Bonus por recompensa alta
            if episode.reward > 0.5:
                base_importance += 0.2
            elif episode.reward < -0.5:
                base_importance += 0.1  # Fracasos tambien son importantes para aprender
            
            # Bonus por recuperacion frecuente
            if episode.times_recalled > 5:
                base_importance += 0.1
            
            # Bonus por exito
            if episode.outcome == Outcome.SUCCESS:
                base_importance += 0.1
            
            episode.importance = min(1.0, base_importance)
    
    def on_consolidation_complete(self, 
                                  callback: Callable[[ConsolidationResult], None]) -> None:
        """Registra callback para consolidacion completada."""
        self._on_consolidation_complete.append(callback)
    
    def get_goal_success_rate(self, goal_type: str) -> Optional[float]:
        """Obtiene tasa de exito para un tipo de objetivo."""
        stats = self._goal_success_rates.get(goal_type)
        if stats and stats["total"] > 0:
            return stats["success"] / stats["total"]
        return None
    
    def get_last_consolidation(self) -> Optional[ConsolidationResult]:
        """Obtiene resultado de ultima consolidacion."""
        if self._consolidation_history:
            return self._consolidation_history[-1]
        return None
    
    def get_stats(self) -> Dict[str, Any]:
        """Obtiene estadisticas del consolidador."""
        return {
            "last_consolidation_ns": self._last_consolidation_ns,
            "consolidation_count": len(self._consolidation_history),
            "action_patterns_count": len(self._action_patterns),
            "goal_types_tracked": len(self._goal_success_rates),
            "goal_success_rates": {
                gt: stats["success"] / stats["total"] if stats["total"] > 0 else 0
                for gt, stats in self._goal_success_rates.items()
            },
        }

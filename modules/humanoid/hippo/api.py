"""
HippoAPI: API unificada del hipocampo.

Proporciona interfaz unificada para acceder a todas las funciones de memoria.
"""
from __future__ import annotations

import asyncio
import logging
from pathlib import Path
from typing import Any, Dict, List, Optional

from .schemas import Episode, ActionRecord, Concept, Relation, Outcome
from .episodic_memory import EpisodicMemory
from .semantic_memory import SemanticMemory
from .consolidator import Consolidator, ConsolidationResult

logger = logging.getLogger(__name__)


def _bitacora(msg: str, ok: bool = True) -> None:
    try:
        from modules.humanoid.ans.evolution_bitacora import append_evolution_log
        append_evolution_log(msg, ok=ok, source="hippo")
    except Exception:
        pass


class HippoAPI:
    """
    API unificada del hipocampo.
    
    Proporciona acceso simplificado a:
    - Memoria episodica
    - Memoria semantica
    - Consolidacion
    """
    
    def __init__(self,
                 storage_path: str = None,
                 embedding_model: Any = None,
                 auto_consolidate: bool = True,
                 consolidation_interval_hours: float = 4.0):
        """
        Inicializa el sistema de memoria.
        
        Args:
            storage_path: Ruta base para persistencia
            embedding_model: Modelo para generar embeddings
            auto_consolidate: Habilitar consolidacion automatica
            consolidation_interval_hours: Intervalo de consolidacion
        """
        self.storage_path = Path(storage_path) if storage_path else None
        self.embedding_model = embedding_model
        self.auto_consolidate = auto_consolidate
        
        # Inicializar subsistemas
        episodic_path = self.storage_path / "episodic" if self.storage_path else None
        semantic_path = self.storage_path / "semantic" if self.storage_path else None
        
        self.episodic = EpisodicMemory(
            storage_path=str(episodic_path) if episodic_path else None,
            embedding_model=embedding_model,
        )
        
        self.semantic = SemanticMemory(
            storage_path=str(semantic_path) if semantic_path else None,
            embedding_model=embedding_model,
        )
        
        self.consolidator = Consolidator(
            episodic_memory=self.episodic,
            semantic_memory=self.semantic,
            consolidation_interval_hours=consolidation_interval_hours,
        )
        
        # Tarea de consolidacion en background
        self._consolidation_task: Optional[asyncio.Task] = None
        
        # Inicializar conocimiento basico si esta vacio
        if not self.semantic._concepts:
            self.semantic.initialize_basic_knowledge()
    
    # === Memoria Episodica ===
    
    async def record_episode(self, episode: Episode) -> str:
        """
        Graba nueva experiencia.
        
        Args:
            episode: Episodio a grabar
        
        Returns:
            ID del episodio
        """
        episode_id = self.episodic.record(episode)
        
        _bitacora(f"Episodio registrado vía API: {episode_id} goal={episode.goal[:50]}")
        
        # Verificar si es momento de consolidar
        if self.auto_consolidate:
            await self._maybe_consolidate()
        
        return episode_id
    
    async def start_episode(self, goal: str, goal_type: str = "",
                           context: Dict[str, Any] = None) -> Episode:
        """
        Inicia un nuevo episodio.
        
        Args:
            goal: Objetivo del episodio
            goal_type: Tipo de objetivo
            context: Contexto adicional
        
        Returns:
            Episodio iniciado
        """
        from .schemas import WorldStateSnapshot, InternalStateSnapshot
        
        episode = Episode(
            goal=goal,
            goal_type=goal_type,
            initial_state=WorldStateSnapshot(),
            internal_state=InternalStateSnapshot(),
            context=context or {},
        )
        
        return episode
    
    async def add_action_to_episode(self, episode: Episode, 
                                   action_type: str,
                                   parameters: Dict[str, Any] = None) -> ActionRecord:
        """
        Agrega accion a un episodio en curso.
        
        Args:
            episode: Episodio activo
            action_type: Tipo de accion
            parameters: Parametros de la accion
        
        Returns:
            Registro de accion creado
        """
        action = ActionRecord(
            action_type=action_type,
            parameters=parameters or {},
        )
        
        episode.add_action(action)
        return action
    
    async def complete_episode(self, episode: Episode, 
                              success: bool,
                              reward: float = None) -> str:
        """
        Completa y graba un episodio.
        
        Args:
            episode: Episodio a completar
            success: Si fue exitoso
            reward: Recompensa (auto-calculada si None)
        
        Returns:
            ID del episodio grabado
        """
        outcome = Outcome.SUCCESS if success else Outcome.FAILURE
        
        if reward is None:
            reward = 1.0 if success else -0.5
        
        episode.complete(outcome, reward)
        
        return await self.record_episode(episode)
    
    async def recall_episodes(self, query: str,
                             filters: Dict[str, Any] = None,
                             limit: int = 10) -> List[Episode]:
        """
        Recupera episodios por busqueda semantica.
        
        Args:
            query: Texto de consulta
            filters: Filtros adicionales (goal_type, outcome, etc.)
            limit: Maximo de resultados
        
        Returns:
            Lista de episodios
        """
        episodes = self.episodic.recall_similar(query, limit * 2)
        
        # Aplicar filtros
        if filters:
            filtered = []
            for ep in episodes:
                match = True
                
                if "goal_type" in filters and ep.goal_type != filters["goal_type"]:
                    match = False
                if "outcome" in filters and ep.outcome.value != filters["outcome"]:
                    match = False
                if "min_reward" in filters and ep.reward < filters["min_reward"]:
                    match = False
                
                if match:
                    filtered.append(ep)
            
            episodes = filtered
        
        return episodes[:limit]
    
    async def recall_similar_situations(self, current_state: Dict[str, Any],
                                       threshold: float = 0.7,
                                       limit: int = 5) -> List[Episode]:
        """
        Encuentra situaciones similares a la actual.
        
        Args:
            current_state: Estado actual
            threshold: Umbral de similitud
            limit: Maximo de resultados
        
        Returns:
            Lista de episodios similares
        """
        # Construir query desde estado
        query_parts = []
        
        if "goal" in current_state:
            query_parts.append(current_state["goal"])
        if "objects" in current_state:
            query_parts.extend(current_state["objects"])
        if "location" in current_state:
            query_parts.append(current_state["location"])
        
        query = " ".join(query_parts) if query_parts else "general situation"
        
        return self.episodic.recall_similar(query, limit, threshold)
    
    # === Memoria Semantica ===
    
    async def get_concept(self, concept_id: str) -> Optional[Concept]:
        """
        Obtiene un concepto por ID.
        
        Args:
            concept_id: ID del concepto
        
        Returns:
            Concepto o None
        """
        return self.semantic.get(concept_id)
    
    async def find_concept(self, name: str) -> Optional[Concept]:
        """
        Busca concepto por nombre.
        
        Args:
            name: Nombre del concepto
        
        Returns:
            Concepto o None
        """
        return self.semantic.get_by_name(name)
    
    async def query_knowledge(self, query: str,
                             relation_type: str = None,
                             limit: int = 10) -> List[Concept]:
        """
        Consulta el grafo de conocimiento.
        
        Args:
            query: Texto de consulta
            relation_type: Filtrar por tipo de relacion
            limit: Maximo de resultados
        
        Returns:
            Lista de conceptos
        """
        results = self.semantic.search(query, limit * 2)
        
        if relation_type:
            filtered = []
            for concept in results:
                if concept.get_relations(relation_type):
                    filtered.append(concept)
            results = filtered
        
        return results[:limit]
    
    async def update_knowledge(self, concept: Concept,
                              source: str = "learned") -> str:
        """
        Actualiza o crea conocimiento.
        
        Args:
            concept: Concepto a guardar
            source: Fuente del conocimiento
        
        Returns:
            ID del concepto
        """
        concept.source = source
        return self.semantic.add_concept(concept)
    
    async def add_knowledge_relation(self, source_name: str,
                                    relation_type: str,
                                    target_name: str,
                                    create_if_missing: bool = True) -> bool:
        """
        Agrega relacion entre conceptos (por nombre).
        
        Args:
            source_name: Nombre del concepto fuente
            relation_type: Tipo de relacion
            target_name: Nombre del concepto destino
            create_if_missing: Crear conceptos si no existen
        
        Returns:
            True si se agrego exitosamente
        """
        source = self.semantic.get_by_name(source_name)
        target = self.semantic.get_by_name(target_name)
        
        if not source and create_if_missing:
            source = self.semantic.create_concept(source_name, "object", source="inferred")
        if not target and create_if_missing:
            target = self.semantic.create_concept(target_name, "object", source="inferred")
        
        if source and target:
            return self.semantic.add_relation(source.id, relation_type, target.id)
        
        return False
    
    async def get_related_concepts(self, concept_name: str,
                                  relation_type: str = None) -> List[Dict[str, Any]]:
        """
        Obtiene conceptos relacionados (por nombre).
        
        Args:
            concept_name: Nombre del concepto
            relation_type: Filtrar por tipo de relacion
        
        Returns:
            Lista de {concept, relation}
        """
        concept = self.semantic.get_by_name(concept_name)
        if not concept:
            return []
        
        related = self.semantic.get_related(concept.id, relation_type)
        
        return [
            {
                "concept": c.to_dict(),
                "relation": r.to_dict(),
            }
            for c, r in related
        ]
    
    # === Consolidacion ===
    
    async def consolidate(self, force: bool = False) -> Optional[ConsolidationResult]:
        """
        Ejecuta consolidacion de memoria.
        
        Args:
            force: Forzar consolidacion
        
        Returns:
            Resultado de consolidacion o None
        """
        result = await self.consolidator.consolidate(force)
        if result:
            _bitacora(f"Consolidación ejecutada vía API: {result.episodes_processed} episodios, {result.concepts_created} conceptos")
        return result
    
    async def _maybe_consolidate(self) -> None:
        """Ejecuta consolidacion si es necesario."""
        try:
            await self.consolidator.consolidate()
        except Exception as e:
            logger.error(f"Error in auto-consolidation: {e}")
    
    def start_background_consolidation(self) -> None:
        """Inicia consolidacion periodica en background."""
        if self._consolidation_task:
            return
        
        async def consolidation_loop():
            while True:
                try:
                    await asyncio.sleep(self.consolidator.consolidation_interval_hours * 3600)
                    await self.consolidate()
                except asyncio.CancelledError:
                    break
                except Exception as e:
                    logger.error(f"Error in consolidation loop: {e}")
        
        self._consolidation_task = asyncio.create_task(consolidation_loop())
    
    def stop_background_consolidation(self) -> None:
        """Detiene consolidacion en background."""
        if self._consolidation_task:
            self._consolidation_task.cancel()
            self._consolidation_task = None
    
    # === Utilidades ===
    
    def get_stats(self) -> Dict[str, Any]:
        """Obtiene estadisticas completas del sistema de memoria."""
        return {
            "episodic": self.episodic.get_stats(),
            "semantic": self.semantic.get_stats(),
            "consolidation": self.consolidator.get_stats(),
        }
    
    def get_goal_success_rate(self, goal_type: str) -> Optional[float]:
        """Obtiene tasa de exito historica para un tipo de objetivo."""
        return self.consolidator.get_goal_success_rate(goal_type)

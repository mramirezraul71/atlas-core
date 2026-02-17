"""
EpisodicRecall: Recuperacion de memorias episodicas (Corteza Temporal Medial).

Permite buscar y recuperar experiencias pasadas relevantes al contexto actual.
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)


@dataclass
class RecallQuery:
    """Consulta de recuperacion de memoria."""
    # Texto libre
    text: Optional[str] = None
    
    # Filtros
    goal_type: Optional[str] = None
    action_type: Optional[str] = None
    outcome: Optional[str] = None  # "success", "failure", "partial"
    
    # Rango temporal
    min_timestamp_ns: Optional[int] = None
    max_timestamp_ns: Optional[int] = None
    
    # Contexto actual (para similitud)
    current_state: Optional[Dict[str, Any]] = None
    current_goal: Optional[str] = None
    
    # Limites
    limit: int = 10
    min_similarity: float = 0.5


@dataclass
class RecalledEpisode:
    """Episodio recuperado."""
    episode_id: str
    goal: str
    outcome: str
    reward: float
    
    # Similitud con query
    similarity: float
    relevance_score: float
    
    # Resumen
    summary: str
    action_count: int
    duration_ms: int
    
    # Acciones clave
    key_actions: List[str]
    
    # Timestamp
    timestamp_ns: int
    
    # Datos completos (opcional)
    full_episode: Optional[Any] = None
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.episode_id,
            "goal": self.goal,
            "outcome": self.outcome,
            "reward": self.reward,
            "similarity": self.similarity,
            "summary": self.summary,
            "key_actions": self.key_actions,
        }


@dataclass
class RecallResult:
    """Resultado de recuperacion."""
    episodes: List[RecalledEpisode]
    query: RecallQuery
    total_searched: int
    processing_time_ms: float
    timestamp_ns: int = field(default_factory=lambda: time.time_ns())
    
    def has_results(self) -> bool:
        return len(self.episodes) > 0
    
    def get_best(self) -> Optional[RecalledEpisode]:
        if not self.episodes:
            return None
        return self.episodes[0]
    
    def get_successful_episodes(self) -> List[RecalledEpisode]:
        return [e for e in self.episodes if e.outcome == "success"]


class EpisodicRecall:
    """
    Sistema de recuperacion de memorias episodicas.
    
    Funcionalidades:
    - Busqueda semantica por texto
    - Busqueda por similitud de contexto
    - Filtrado por atributos
    - Ranking por relevancia
    """
    
    def __init__(
        self,
        hippo_api: Optional[Any] = None,
        embedding_model: Optional[Any] = None,
    ):
        """
        Inicializa el sistema de recall.
        
        Args:
            hippo_api: Referencia al HippoAPI para acceso a memorias
            embedding_model: Modelo para generar embeddings de queries
        """
        self.hippo_api = hippo_api
        self.embedding_model = embedding_model
        
        # Cache de recuperaciones recientes
        self._cache: Dict[str, RecallResult] = {}
        self._cache_ttl_ns = 60 * 1e9  # 60 segundos
        
        # Estadisticas
        self._recall_count = 0
        self._cache_hits = 0
        self._total_time_ms = 0
        
        # Callbacks
        self._callbacks: List[Callable[[RecallResult], None]] = []
    
    async def recall(self, query: RecallQuery) -> RecallResult:
        """
        Recupera episodios relevantes.
        
        Args:
            query: Consulta de recuperacion
        
        Returns:
            Resultado con episodios recuperados
        """
        start_time = time.time()
        
        # Verificar cache
        cache_key = self._make_cache_key(query)
        cached = self._get_from_cache(cache_key)
        if cached:
            self._cache_hits += 1
            return cached
        
        # Recuperar de memoria
        episodes = await self._search_episodes(query)
        
        # Calcular relevancia
        scored_episodes = self._score_episodes(episodes, query)
        
        # Ordenar y limitar
        scored_episodes.sort(key=lambda x: x.relevance_score, reverse=True)
        scored_episodes = scored_episodes[:query.limit]
        
        processing_time = (time.time() - start_time) * 1000
        
        result = RecallResult(
            episodes=scored_episodes,
            query=query,
            total_searched=len(episodes),
            processing_time_ms=processing_time,
        )
        
        # Actualizar cache
        self._cache[cache_key] = result
        
        # Estadisticas
        self._recall_count += 1
        self._total_time_ms += processing_time
        
        # Callbacks
        for callback in self._callbacks:
            try:
                callback(result)
            except Exception as e:
                logger.error(f"Error in recall callback: {e}")
        
        return result
    
    async def _search_episodes(self, query: RecallQuery) -> List[Any]:
        """Busca episodios en memoria."""
        episodes = []
        
        if self.hippo_api:
            # Buscar en HippoAPI
            try:
                if query.text:
                    # Busqueda semantica
                    episodes = self.hippo_api.episodic.recall_similar(
                        query.text,
                        limit=query.limit * 2,
                        min_similarity=query.min_similarity,
                    )
                else:
                    # Busqueda por filtros
                    episodes = self.hippo_api.episodic.get_recent(
                        limit=query.limit * 2,
                    )
            except Exception as e:
                logger.error(f"Error searching episodes: {e}")
        else:
            # Mock: generar episodios de prueba
            episodes = self._generate_mock_episodes(query)
        
        # Aplicar filtros
        filtered = []
        for ep in episodes:
            if self._matches_filters(ep, query):
                filtered.append(ep)
        
        return filtered
    
    def _matches_filters(self, episode: Any, query: RecallQuery) -> bool:
        """Verifica si episodio cumple filtros."""
        # Goal type
        if query.goal_type:
            ep_goal = getattr(episode, "goal", "") or ""
            if query.goal_type.lower() not in ep_goal.lower():
                return False
        
        # Outcome
        if query.outcome:
            ep_outcome = getattr(episode, "outcome", None)
            if ep_outcome:
                outcome_str = ep_outcome.value if hasattr(ep_outcome, "value") else str(ep_outcome)
                if outcome_str.lower() != query.outcome.lower():
                    return False
        
        # Rango temporal
        ep_ts = getattr(episode, "timestamp_ns", 0)
        if query.min_timestamp_ns and ep_ts < query.min_timestamp_ns:
            return False
        if query.max_timestamp_ns and ep_ts > query.max_timestamp_ns:
            return False
        
        return True
    
    def _score_episodes(
        self,
        episodes: List[Any],
        query: RecallQuery,
    ) -> List[RecalledEpisode]:
        """Calcula scores de relevancia."""
        scored = []
        
        for ep in episodes:
            # Extraer datos del episodio
            ep_id = getattr(ep, "id", str(id(ep)))
            goal = getattr(ep, "goal", "unknown")
            
            outcome_raw = getattr(ep, "outcome", "partial")
            outcome = outcome_raw.value if hasattr(outcome_raw, "value") else str(outcome_raw)
            
            reward = getattr(ep, "reward", 0.0)
            timestamp_ns = getattr(ep, "timestamp_ns", time.time_ns())
            
            actions = getattr(ep, "actions", [])
            action_count = len(actions)
            
            # Calcular duracion
            if actions:
                first_ts = getattr(actions[0], "timestamp_ns", timestamp_ns)
                last_ts = getattr(actions[-1], "timestamp_ns", timestamp_ns)
                duration_ms = (last_ts - first_ts) / 1e6
            else:
                duration_ms = 0
            
            # Extraer acciones clave
            key_actions = []
            for action in actions[:5]:
                action_type = getattr(action, "action_type", "unknown")
                key_actions.append(str(action_type))
            
            # Calcular similitud
            similarity = self._compute_similarity(ep, query)
            
            # Calcular relevancia
            relevance = self._compute_relevance(ep, query, similarity)
            
            # Generar resumen
            summary = f"{goal}: {outcome} (reward={reward:.2f})"
            
            scored.append(RecalledEpisode(
                episode_id=ep_id,
                goal=goal,
                outcome=outcome,
                reward=reward,
                similarity=similarity,
                relevance_score=relevance,
                summary=summary,
                action_count=action_count,
                duration_ms=int(duration_ms),
                key_actions=key_actions,
                timestamp_ns=timestamp_ns,
                full_episode=ep,
            ))
        
        return scored
    
    def _compute_similarity(self, episode: Any, query: RecallQuery) -> float:
        """Computa similitud semantica."""
        # Si hay embedding
        ep_embedding = getattr(episode, "embedding", None)
        
        if ep_embedding and query.text and self.embedding_model:
            # Calcular similitud de embeddings
            try:
                query_embedding = self.embedding_model.encode(query.text)
                
                import numpy as np
                similarity = np.dot(ep_embedding, query_embedding) / (
                    np.linalg.norm(ep_embedding) * np.linalg.norm(query_embedding) + 1e-8
                )
                return float(max(0, min(1, similarity)))
            except:
                pass
        
        # Fallback: similitud basada en texto
        if query.text:
            goal = getattr(episode, "goal", "")
            text_lower = query.text.lower()
            goal_lower = goal.lower()
            
            # Overlap de palabras
            query_words = set(text_lower.split())
            goal_words = set(goal_lower.split())
            
            if query_words and goal_words:
                overlap = len(query_words & goal_words)
                similarity = overlap / max(len(query_words), len(goal_words))
                return similarity
        
        return 0.5  # Default
    
    def _compute_relevance(
        self,
        episode: Any,
        query: RecallQuery,
        similarity: float,
    ) -> float:
        """Computa score de relevancia total."""
        relevance = similarity
        
        # Bonus por exito
        outcome = getattr(episode, "outcome", None)
        if outcome:
            outcome_str = outcome.value if hasattr(outcome, "value") else str(outcome)
            if outcome_str == "success":
                relevance += 0.2
            elif outcome_str == "failure":
                relevance -= 0.1
        
        # Bonus por reward positivo
        reward = getattr(episode, "reward", 0)
        if reward > 0:
            relevance += min(0.2, reward * 0.1)
        
        # Penalizacion por antiguedad (episodios muy viejos son menos relevantes)
        timestamp = getattr(episode, "timestamp_ns", time.time_ns())
        age_hours = (time.time_ns() - timestamp) / (3600 * 1e9)
        if age_hours > 24:
            relevance -= min(0.3, age_hours * 0.01)
        
        return max(0, min(1, relevance))
    
    def _generate_mock_episodes(self, query: RecallQuery) -> List[Any]:
        """Genera episodios mock para pruebas."""
        from dataclasses import dataclass as dc
        
        @dc
        class MockEpisode:
            id: str
            goal: str
            outcome: str
            reward: float
            timestamp_ns: int
            actions: list
        
        episodes = [
            MockEpisode(
                id="ep_001",
                goal="fetch red cup from table",
                outcome="success",
                reward=1.0,
                timestamp_ns=time.time_ns() - int(3600 * 1e9),
                actions=[],
            ),
            MockEpisode(
                id="ep_002",
                goal="navigate to kitchen",
                outcome="success",
                reward=0.8,
                timestamp_ns=time.time_ns() - int(7200 * 1e9),
                actions=[],
            ),
            MockEpisode(
                id="ep_003",
                goal="grasp bottle",
                outcome="failure",
                reward=-0.5,
                timestamp_ns=time.time_ns() - int(86400 * 1e9),
                actions=[],
            ),
        ]
        
        return episodes
    
    def _make_cache_key(self, query: RecallQuery) -> str:
        """Genera clave de cache para query."""
        parts = [
            query.text or "",
            query.goal_type or "",
            query.outcome or "",
            str(query.limit),
        ]
        return "|".join(parts)
    
    def _get_from_cache(self, key: str) -> Optional[RecallResult]:
        """Obtiene resultado de cache si es valido."""
        if key not in self._cache:
            return None
        
        result = self._cache[key]
        age_ns = time.time_ns() - result.timestamp_ns
        
        if age_ns > self._cache_ttl_ns:
            del self._cache[key]
            return None
        
        return result
    
    async def recall_for_goal(self, goal: str, limit: int = 5) -> RecallResult:
        """Shortcut para recuperar episodios de un goal especifico."""
        query = RecallQuery(
            text=goal,
            limit=limit,
        )
        return await self.recall(query)
    
    async def recall_successful_for_action(
        self,
        action_type: str,
        limit: int = 5,
    ) -> RecallResult:
        """Recupera episodios exitosos con cierto tipo de accion."""
        query = RecallQuery(
            action_type=action_type,
            outcome="success",
            limit=limit,
        )
        return await self.recall(query)
    
    async def recall_similar_situation(
        self,
        current_state: Dict[str, Any],
        limit: int = 5,
    ) -> RecallResult:
        """Recupera episodios de situaciones similares."""
        query = RecallQuery(
            current_state=current_state,
            limit=limit,
        )
        return await self.recall(query)
    
    def set_hippo_api(self, hippo_api: Any) -> None:
        """Configura referencia al HippoAPI."""
        self.hippo_api = hippo_api
    
    def on_recall(self, callback: Callable[[RecallResult], None]) -> None:
        """Registra callback para recalls."""
        self._callbacks.append(callback)
    
    def clear_cache(self) -> None:
        """Limpia cache."""
        self._cache.clear()
    
    def get_stats(self) -> Dict[str, Any]:
        """Obtiene estadisticas."""
        return {
            "recall_count": self._recall_count,
            "cache_hits": self._cache_hits,
            "cache_hit_rate": self._cache_hits / max(1, self._recall_count),
            "cache_size": len(self._cache),
            "avg_processing_time_ms": self._total_time_ms / max(1, self._recall_count),
        }

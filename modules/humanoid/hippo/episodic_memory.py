"""
EpisodicMemory: Memoria episodica del hipocampo.

Analogo biologico: Hipocampo CA1/CA3
- Almacenamiento de experiencias
- Recuperacion por similitud
- Indexacion por embedding
"""
from __future__ import annotations

import json
import logging
import os
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Callable

from .schemas import Episode, Outcome

logger = logging.getLogger(__name__)


class EpisodicMemory:
    """
    Memoria episodica del hipocampo.
    
    Almacena y recupera experiencias completas (episodios).
    Soporta busqueda por similitud usando embeddings.
    """
    
    def __init__(self, 
                 storage_path: str = None,
                 embedding_model: Any = None,
                 max_episodes: int = 10000):
        """
        Inicializa memoria episodica.
        
        Args:
            storage_path: Ruta para persistencia
            embedding_model: Modelo para generar embeddings
            max_episodes: Maximo de episodios a mantener
        """
        self.storage_path = Path(storage_path) if storage_path else None
        self.embedding_model = embedding_model
        self.max_episodes = max_episodes
        
        # Almacenamiento en memoria
        self._episodes: Dict[str, Episode] = {}
        
        # Indice FAISS para busqueda por similitud
        self._faiss_index = None
        self._episode_ids_index: List[str] = []  # Mapeo indice FAISS -> episode_id
        
        # Callbacks
        self._on_episode_recorded: List[Callable[[Episode], None]] = []
        
        # Cargar datos persistidos
        if self.storage_path:
            self._load_from_disk()
    
    def _load_from_disk(self) -> None:
        """Carga episodios desde disco."""
        if not self.storage_path or not self.storage_path.exists():
            return
        
        episodes_file = self.storage_path / "episodes.json"
        if not episodes_file.exists():
            return
        
        try:
            with open(episodes_file, "r", encoding="utf-8") as f:
                data = json.load(f)
            
            for ep_data in data.get("episodes", []):
                episode = self._deserialize_episode(ep_data)
                if episode:
                    self._episodes[episode.id] = episode
            
            logger.info(f"Loaded {len(self._episodes)} episodes from disk")
            
            # Reconstruir indice FAISS si hay embeddings
            self._rebuild_index()
            
        except Exception as e:
            logger.error(f"Error loading episodes: {e}")
    
    def _save_to_disk(self) -> None:
        """Guarda episodios a disco."""
        if not self.storage_path:
            return
        
        self.storage_path.mkdir(parents=True, exist_ok=True)
        episodes_file = self.storage_path / "episodes.json"
        
        try:
            data = {
                "version": "1.0",
                "count": len(self._episodes),
                "episodes": [
                    ep.to_full_dict() for ep in self._episodes.values()
                ],
            }
            
            with open(episodes_file, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
                
        except Exception as e:
            logger.error(f"Error saving episodes: {e}")
    
    def _deserialize_episode(self, data: Dict) -> Optional[Episode]:
        """Deserializa episodio desde diccionario."""
        try:
            from .schemas import ActionRecord, WorldStateSnapshot, InternalStateSnapshot
            
            actions = []
            for a_data in data.get("actions", []):
                actions.append(ActionRecord(**a_data))
            
            episode = Episode(
                id=data.get("id"),
                timestamp_ns=data.get("timestamp_ns", time.time_ns()),
                goal=data.get("goal", ""),
                goal_type=data.get("goal_type", ""),
                actions=actions,
                outcome=Outcome(data.get("outcome", "partial")),
                reward=data.get("reward", 0.0),
                duration_ms=data.get("duration_ms", 0.0),
                tags=data.get("tags", []),
                importance=data.get("importance", 0.5),
                times_recalled=data.get("times_recalled", 0),
                embedding=data.get("embedding"),
                context=data.get("context", {}),
            )
            
            return episode
            
        except Exception as e:
            logger.error(f"Error deserializing episode: {e}")
            return None
    
    def _generate_embedding(self, episode: Episode) -> Optional[List[float]]:
        """Genera embedding para un episodio."""
        if not self.embedding_model:
            return None
        
        # Construir texto para embedding
        text = f"{episode.goal} {episode.goal_type}"
        for action in episode.actions:
            text += f" {action.action_type}"
        text += f" {' '.join(episode.tags)}"
        
        try:
            if hasattr(self.embedding_model, "encode"):
                embedding = self.embedding_model.encode(text).tolist()
                return embedding
            elif hasattr(self.embedding_model, "embed"):
                return self.embedding_model.embed(text)
        except Exception as e:
            logger.error(f"Error generating embedding: {e}")
        
        return None
    
    def _rebuild_index(self) -> None:
        """Reconstruye indice FAISS."""
        try:
            import numpy as np
            
            # Recolectar episodios con embeddings
            episodes_with_embeddings = [
                (ep_id, ep) for ep_id, ep in self._episodes.items()
                if ep.embedding
            ]
            
            if not episodes_with_embeddings:
                self._faiss_index = None
                self._episode_ids_index = []
                return
            
            # Intentar usar FAISS
            try:
                import faiss
                
                embeddings = np.array([ep.embedding for _, ep in episodes_with_embeddings], dtype=np.float32)
                dim = embeddings.shape[1]
                
                self._faiss_index = faiss.IndexFlatL2(dim)
                self._faiss_index.add(embeddings)
                self._episode_ids_index = [ep_id for ep_id, _ in episodes_with_embeddings]
                
                logger.info(f"FAISS index rebuilt with {len(self._episode_ids_index)} episodes")
                
            except ImportError:
                logger.warning("FAISS not available, using numpy for similarity")
                self._faiss_index = None
                
        except Exception as e:
            logger.error(f"Error rebuilding index: {e}")
    
    def record(self, episode: Episode) -> str:
        """
        Graba un nuevo episodio.
        
        Args:
            episode: Episodio a grabar
        
        Returns:
            ID del episodio
        """
        # Generar embedding si no tiene
        if not episode.embedding and self.embedding_model:
            episode.embedding = self._generate_embedding(episode)
        
        # Guardar
        self._episodes[episode.id] = episode
        
        # Actualizar indice FAISS
        if episode.embedding:
            self._add_to_index(episode)
        
        # Limitar tamano
        if len(self._episodes) > self.max_episodes:
            self._evict_old_episodes()
        
        # Persistir
        self._save_to_disk()
        
        # Notificar
        for callback in self._on_episode_recorded:
            try:
                callback(episode)
            except Exception as e:
                logger.error(f"Error in episode callback: {e}")
        
        logger.debug(f"Episode recorded: {episode.id}")
        return episode.id
    
    def _add_to_index(self, episode: Episode) -> None:
        """Agrega episodio al indice FAISS."""
        if not episode.embedding:
            return
        
        try:
            import numpy as np
            
            if self._faiss_index is None:
                self._rebuild_index()
            else:
                try:
                    import faiss
                    embedding = np.array([episode.embedding], dtype=np.float32)
                    self._faiss_index.add(embedding)
                    self._episode_ids_index.append(episode.id)
                except ImportError:
                    pass
                    
        except Exception as e:
            logger.error(f"Error adding to index: {e}")
    
    def _evict_old_episodes(self) -> None:
        """Elimina episodios antiguos de baja importancia."""
        if len(self._episodes) <= self.max_episodes:
            return
        
        # Ordenar por importancia y recencia
        episodes = sorted(
            self._episodes.values(),
            key=lambda e: (e.importance, e.timestamp_ns)
        )
        
        # Eliminar los menos importantes
        to_remove = len(self._episodes) - self.max_episodes
        for i in range(to_remove):
            ep_id = episodes[i].id
            del self._episodes[ep_id]
        
        # Reconstruir indice
        self._rebuild_index()
    
    def get(self, episode_id: str) -> Optional[Episode]:
        """Obtiene episodio por ID."""
        episode = self._episodes.get(episode_id)
        if episode:
            episode.increment_recall()
        return episode
    
    def recall_similar(self, query: str, limit: int = 10, 
                      min_similarity: float = 0.5) -> List[Episode]:
        """
        Recupera episodios similares por busqueda semantica.
        
        Args:
            query: Texto de consulta
            limit: Maximo de resultados
            min_similarity: Similitud minima
        
        Returns:
            Lista de episodios similares
        """
        if not self.embedding_model:
            # Fallback a busqueda por keywords
            return self._search_by_keywords(query, limit)
        
        try:
            import numpy as np
            
            # Generar embedding de query
            if hasattr(self.embedding_model, "encode"):
                query_embedding = self.embedding_model.encode(query)
            elif hasattr(self.embedding_model, "embed"):
                query_embedding = np.array(self.embedding_model.embed(query))
            else:
                return self._search_by_keywords(query, limit)
            
            # Buscar en FAISS
            if self._faiss_index is not None:
                try:
                    import faiss
                    
                    query_vec = np.array([query_embedding], dtype=np.float32)
                    distances, indices = self._faiss_index.search(query_vec, min(limit, len(self._episode_ids_index)))
                    
                    results = []
                    for i, idx in enumerate(indices[0]):
                        if idx < 0 or idx >= len(self._episode_ids_index):
                            continue
                        
                        ep_id = self._episode_ids_index[idx]
                        episode = self._episodes.get(ep_id)
                        if episode:
                            # Convertir distancia a similitud (aproximada)
                            similarity = 1.0 / (1.0 + distances[0][i])
                            if similarity >= min_similarity:
                                episode.increment_recall()
                                results.append(episode)
                    
                    return results
                    
                except ImportError:
                    pass
            
            # Fallback a busqueda manual
            return self._search_by_keywords(query, limit)
            
        except Exception as e:
            logger.error(f"Error in similarity search: {e}")
            return self._search_by_keywords(query, limit)
    
    def _search_by_keywords(self, query: str, limit: int) -> List[Episode]:
        """Busqueda simple por keywords."""
        query_lower = query.lower()
        keywords = query_lower.split()
        
        scored = []
        for episode in self._episodes.values():
            score = 0
            text = f"{episode.goal} {episode.goal_type} {' '.join(episode.tags)}".lower()
            
            for keyword in keywords:
                if keyword in text:
                    score += 1
            
            if score > 0:
                scored.append((episode, score))
        
        scored.sort(key=lambda x: (-x[1], -x[0].timestamp_ns))
        
        results = []
        for episode, _ in scored[:limit]:
            episode.increment_recall()
            results.append(episode)
        
        return results
    
    def recall_by_goal(self, goal_type: str, limit: int = 10) -> List[Episode]:
        """Recupera episodios por tipo de objetivo."""
        episodes = [
            ep for ep in self._episodes.values()
            if ep.goal_type == goal_type
        ]
        
        # Ordenar por recencia
        episodes.sort(key=lambda e: e.timestamp_ns, reverse=True)
        
        for ep in episodes[:limit]:
            ep.increment_recall()
        
        return episodes[:limit]
    
    def recall_successful(self, limit: int = 10) -> List[Episode]:
        """Recupera episodios exitosos recientes."""
        episodes = [
            ep for ep in self._episodes.values()
            if ep.outcome == Outcome.SUCCESS
        ]
        
        episodes.sort(key=lambda e: e.timestamp_ns, reverse=True)
        
        return episodes[:limit]
    
    def recall_failures(self, limit: int = 10) -> List[Episode]:
        """Recupera episodios fallidos recientes."""
        episodes = [
            ep for ep in self._episodes.values()
            if ep.outcome == Outcome.FAILURE
        ]
        
        episodes.sort(key=lambda e: e.timestamp_ns, reverse=True)
        
        return episodes[:limit]
    
    def get_recent(self, limit: int = 10) -> List[Episode]:
        """Obtiene episodios mas recientes."""
        episodes = list(self._episodes.values())
        episodes.sort(key=lambda e: e.timestamp_ns, reverse=True)
        return episodes[:limit]
    
    def on_episode_recorded(self, callback: Callable[[Episode], None]) -> None:
        """Registra callback para nuevos episodios."""
        self._on_episode_recorded.append(callback)
    
    def get_stats(self) -> Dict[str, Any]:
        """Obtiene estadisticas de la memoria."""
        by_outcome = {}
        by_goal_type = {}
        total_reward = 0.0
        
        for ep in self._episodes.values():
            outcome = ep.outcome.value
            by_outcome[outcome] = by_outcome.get(outcome, 0) + 1
            
            gt = ep.goal_type or "unknown"
            by_goal_type[gt] = by_goal_type.get(gt, 0) + 1
            
            total_reward += ep.reward
        
        return {
            "total_episodes": len(self._episodes),
            "by_outcome": by_outcome,
            "by_goal_type": by_goal_type,
            "total_reward": total_reward,
            "average_reward": total_reward / len(self._episodes) if self._episodes else 0,
            "has_faiss_index": self._faiss_index is not None,
            "indexed_episodes": len(self._episode_ids_index),
        }

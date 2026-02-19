"""
Unified Memory Cortex — Interfaz unica a todos los sistemas de memoria de ATLAS.

Inspirado en RoboMemory (2025) que unifica Spatial, Temporal, Episodic y Semantic.
Provee una API unificada de consulta que busca en TODOS los subsistemas:

1. Memoria Episodica (hippo)
2. Memoria Episodica (brain/learning)
3. Memoria Semantica (hippo - grafo de conceptos)
4. Memoria Semantica (memory_engine - embeddings)
5. Memory Engine (threads, tasks, runs, FTS)
6. Lifelog (registro continuo)
7. Autobiographical Memory (narrativa, identidad)
8. World Model (estado del entorno)
9. Cortex Temporal (episodic recall con scoring)

Resultado: una consulta unificada que devuelve resultados rankeados
de todos los sistemas con su fuente, relevancia y tipo.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class MemoryResult:
    source: str
    memory_type: str
    content: str
    relevance: float = 0.5
    timestamp: float = 0.0
    metadata: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "source": self.source,
            "memory_type": self.memory_type,
            "content": self.content,
            "relevance": round(self.relevance, 3),
            "timestamp": self.timestamp,
            "metadata": self.metadata,
        }


class UnifiedMemoryCortex:
    """Busca en todos los sistemas de memoria y devuelve resultados unificados."""

    def recall(self, query: str, memory_types: List[str] = None,
               limit: int = 20, min_relevance: float = 0.0) -> List[MemoryResult]:
        """Busqueda unificada en todos los sistemas de memoria."""
        results: List[MemoryResult] = []
        types = set(memory_types or [
            "episodic", "semantic", "procedural", "lifelog",
            "autobiographical", "world_model", "knowledge_base"
        ])

        if "episodic" in types:
            results.extend(self._search_episodic_hippo(query, limit))
            results.extend(self._search_episodic_brain(query, limit))

        if "semantic" in types:
            results.extend(self._search_semantic_hippo(query, limit))
            results.extend(self._search_semantic_engine(query, limit))

        if "knowledge_base" in types or "procedural" in types:
            results.extend(self._search_memory_engine(query, limit))

        if "lifelog" in types:
            results.extend(self._search_lifelog(query, limit))

        if "autobiographical" in types:
            results.extend(self._search_autobiographical(query))

        if "world_model" in types:
            results.extend(self._search_world_model(query))

        if min_relevance > 0:
            results = [r for r in results if r.relevance >= min_relevance]

        results.sort(key=lambda r: r.relevance, reverse=True)
        return results[:limit]

    def get_context_for_action(self, action: str, goal: str = "",
                                limit: int = 10) -> Dict[str, Any]:
        """Recopila todo el contexto relevante para una accion."""
        query = f"{action} {goal}".strip()
        memories = self.recall(query, limit=limit)

        context = {
            "relevant_memories": [m.to_dict() for m in memories],
            "memory_sources": list(set(m.source for m in memories)),
            "total_found": len(memories),
            "avg_relevance": sum(m.relevance for m in memories) / max(1, len(memories)),
        }

        try:
            from modules.humanoid.world_model import WorldModel
            from modules.humanoid.world_model.engine import get_world_model
            wm = get_world_model()
            context["world_state"] = wm.get_state_summary()
        except Exception:
            pass

        try:
            from modules.humanoid.memory_engine.autobiographical import get_autobiographical_memory
            am = get_autobiographical_memory()
            context["identity"] = am.get_identity()
        except Exception:
            pass

        return context

    def get_all_stats(self) -> Dict[str, Any]:
        """Estadisticas de todos los sistemas de memoria."""
        stats: Dict[str, Any] = {}

        for name, fn in [
            ("episodic_hippo", self._stats_episodic_hippo),
            ("semantic_hippo", self._stats_semantic_hippo),
            ("semantic_engine", self._stats_semantic_engine),
            ("memory_engine", self._stats_memory_engine),
            ("lifelog", self._stats_lifelog),
            ("autobiographical", self._stats_autobiographical),
            ("world_model", self._stats_world_model),
        ]:
            try:
                stats[name] = fn()
            except Exception as e:
                stats[name] = {"error": str(e)}

        return stats

    # ── Busquedas individuales ─────────────────────────────

    def _search_episodic_hippo(self, query: str, limit: int) -> List[MemoryResult]:
        results = []
        try:
            from modules.humanoid.hippo import EpisodicMemory
            em = EpisodicMemory()
            episodes = em.recall_by_goal(query, limit=limit)
            for ep in episodes:
                d = ep.to_dict() if hasattr(ep, "to_dict") else ep
                results.append(MemoryResult(
                    source="hippo_episodic",
                    memory_type="episodic",
                    content=f"[{d.get('outcome', '?')}] {d.get('goal', '')}",
                    relevance=0.7,
                    timestamp=d.get("timestamp_ns", 0) / 1e9 if d.get("timestamp_ns") else 0,
                    metadata=d,
                ))
        except Exception:
            pass
        return results

    def _search_episodic_brain(self, query: str, limit: int) -> List[MemoryResult]:
        results = []
        try:
            from modules.humanoid.brain.learning.episodic_memory import EpisodicMemory
            em = EpisodicMemory()
            episodes = em.get_recent_episodes(limit=limit)
            for ep in episodes:
                sit = ep.get("situation", "")
                if query.lower() in sit.lower() or query.lower() in (ep.get("action_taken") or "").lower():
                    results.append(MemoryResult(
                        source="brain_episodic",
                        memory_type="episodic",
                        content=f"[{'OK' if ep.get('success') else 'FAIL'}] {sit}",
                        relevance=0.6,
                        timestamp=ep.get("timestamp", 0),
                        metadata=ep,
                    ))
        except Exception:
            pass
        return results

    def _search_semantic_hippo(self, query: str, limit: int) -> List[MemoryResult]:
        results = []
        try:
            from modules.humanoid.hippo import SemanticMemory
            sm = SemanticMemory()
            concepts = sm.search(query, limit=limit)
            for c in concepts:
                d = c.to_dict() if hasattr(c, "to_dict") else c
                results.append(MemoryResult(
                    source="hippo_semantic",
                    memory_type="semantic",
                    content=f"{d.get('name', '')} ({d.get('concept_type', '')})",
                    relevance=d.get("confidence", 0.5),
                    metadata=d,
                ))
        except Exception:
            pass
        return results

    def _search_semantic_engine(self, query: str, limit: int) -> List[MemoryResult]:
        results = []
        try:
            from modules.humanoid.memory_engine.semantic_memory import SemanticMemory
            sm = SemanticMemory.get_instance()
            experiences = sm.recall_similar(query, top_k=limit)
            for exp in experiences:
                results.append(MemoryResult(
                    source="engine_semantic",
                    memory_type="semantic",
                    content=exp.get("description", ""),
                    relevance=exp.get("similarity", 0.5),
                    timestamp=exp.get("timestamp", 0),
                    metadata=exp,
                ))
        except Exception:
            pass
        return results

    def _search_memory_engine(self, query: str, limit: int) -> List[MemoryResult]:
        results = []
        try:
            from modules.humanoid.memory_engine.recall import recall_by_query
            items = recall_by_query(query, limit=limit)
            for item in items:
                kind = item.get("kind", "unknown")
                content = item.get("content_preview", item.get("goal", item.get("result_json", "")))
                results.append(MemoryResult(
                    source="memory_engine",
                    memory_type="procedural" if kind in ("run", "task") else "knowledge_base",
                    content=str(content)[:200],
                    relevance=0.5,
                    timestamp=0,
                    metadata=item,
                ))
        except Exception:
            pass
        return results

    def _search_lifelog(self, query: str, limit: int) -> List[MemoryResult]:
        results = []
        try:
            from modules.humanoid.memory_engine.lifelog import get_lifelog
            ll = get_lifelog()
            entries = ll.search(query, limit=limit)
            for e in entries:
                content = f"[{e.get('event_type','')}] {e.get('perception', e.get('action', ''))}"
                if e.get("outcome"):
                    content += f" -> {e['outcome']}"
                results.append(MemoryResult(
                    source="lifelog",
                    memory_type="lifelog",
                    content=content[:200],
                    relevance=e.get("importance", 0.5),
                    timestamp=e.get("timestamp_ts", 0),
                    metadata=e,
                ))
        except Exception:
            pass
        return results

    def _search_autobiographical(self, query: str) -> List[MemoryResult]:
        results = []
        try:
            from modules.humanoid.memory_engine.autobiographical import get_autobiographical_memory
            am = get_autobiographical_memory()
            milestones = am.get_milestones(limit=20)
            for m in milestones:
                if query.lower() in (m.get("title", "") + m.get("description", "")).lower():
                    results.append(MemoryResult(
                        source="autobiographical",
                        memory_type="autobiographical",
                        content=f"[{m.get('category','')}] {m.get('title', '')}",
                        relevance=m.get("importance", 0.7),
                        timestamp=m.get("timestamp_ts", 0),
                        metadata=m,
                    ))

            narrative = am.get_latest_narrative()
            if narrative and query.lower() in narrative.lower():
                results.append(MemoryResult(
                    source="autobiographical",
                    memory_type="autobiographical",
                    content=narrative[:200],
                    relevance=0.8,
                ))
        except Exception:
            pass
        return results

    def _search_world_model(self, query: str) -> List[MemoryResult]:
        results = []
        try:
            from modules.humanoid.world_model.engine import get_world_model
            wm = get_world_model()
            entities = wm.query_entities()
            for e in entities:
                if query.lower() in (e.name + e.entity_type + str(e.state)).lower():
                    state_str = json.dumps(e.state, default=str)
                    if len(state_str) > 100:
                        state_str = state_str[:100] + "..."
                    results.append(MemoryResult(
                        source="world_model",
                        memory_type="world_model",
                        content=f"{e.name} ({e.entity_type}): {state_str}",
                        relevance=e.confidence,
                        timestamp=e.last_seen_ts,
                        metadata={"id": e.id, "state": e.state, "properties": e.properties},
                    ))
        except Exception:
            pass
        return results

    # ── Stats individuales ─────────────────────────────────

    def _stats_episodic_hippo(self) -> Dict:
        from modules.humanoid.hippo import EpisodicMemory
        return EpisodicMemory().get_stats()

    def _stats_semantic_hippo(self) -> Dict:
        from modules.humanoid.hippo import SemanticMemory
        return SemanticMemory().get_stats()

    def _stats_semantic_engine(self) -> Dict:
        from modules.humanoid.memory_engine.semantic_memory import SemanticMemory
        return SemanticMemory.get_instance().get_statistics()

    def _stats_memory_engine(self) -> Dict:
        from modules.humanoid.memory_engine import recall
        return {"available": True}

    def _stats_lifelog(self) -> Dict:
        from modules.humanoid.memory_engine.lifelog import get_lifelog
        return get_lifelog().get_stats()

    def _stats_autobiographical(self) -> Dict:
        from modules.humanoid.memory_engine.autobiographical import get_autobiographical_memory
        return get_autobiographical_memory().get_stats()

    def _stats_world_model(self) -> Dict:
        from modules.humanoid.world_model.engine import get_world_model
        return get_world_model().get_state_summary()


import json

_instance: Optional[UnifiedMemoryCortex] = None


def get_unified_memory() -> UnifiedMemoryCortex:
    global _instance
    if _instance is None:
        _instance = UnifiedMemoryCortex()
    return _instance

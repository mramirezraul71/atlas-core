"""Consolida experiencias en conocimiento generalizado (aprendizaje progresivo ATLAS)."""
from __future__ import annotations

import time
from collections import defaultdict
from typing import Any, Dict, List, Tuple


class KnowledgeConsolidator:
    """
    Consolida experiencias en conocimiento generalizado.
    Similar a consolidación durante 'sueño': extrae patrones de experiencias.
    """

    def __init__(
        self,
        semantic_memory: Any,
        episodic_memory: Any,
        knowledge_base: Any,
    ) -> None:
        self.semantic_memory = semantic_memory
        self.episodic_memory = episodic_memory
        self.knowledge_base = knowledge_base
        self.last_consolidation = time.time()

    def should_consolidate(self) -> bool:
        """Determinar si es hora de consolidar (p. ej. cada hora)."""
        time_threshold = 3600
        return (time.time() - self.last_consolidation) > time_threshold

    def consolidate_knowledge(self) -> Dict[str, Any]:
        """
        Proceso de consolidación completo.
        Returns: reporte de qué se aprendió/actualizó.
        """
        report: Dict[str, Any] = {
            "new_concepts": [],
            "new_rules": [],
            "strengthened_patterns": [],
            "weakened_beliefs": [],
            "generalizations": [],
        }
        report["strengthened_patterns"] = self._find_patterns_in_experiences()
        report["generalizations"] = self._generalize_from_cases()
        report["new_concepts"] = self._identify_new_concepts()
        report["new_rules"] = self._update_causal_rules()
        report["weakened_beliefs"] = self._clean_contradictions()
        self.last_consolidation = time.time()
        return report

    def _get_recent_episodes(self, limit: int = 200) -> List[Dict[str, Any]]:
        """Obtener episodios recientes (episódica o semántica como fallback)."""
        if getattr(self.episodic_memory, "get_recent", None):
            try:
                return self.episodic_memory.get_recent(limit=limit)
            except Exception:
                pass
        # Fallback: usar recall_similar de semántica para "success" y "failure"
        out: List[Dict[str, Any]] = []
        if getattr(self.semantic_memory, "recall_similar", None):
            for query in ("success outcome", "failure outcome"):
                try:
                    items = self.semantic_memory.recall_similar(
                        query, top_k=limit // 2, min_similarity=0.3
                    )
                    for it in items:
                        desc = (it.get("description") or "").split(" → ")
                        situation = desc[0] if desc else ""
                        action = desc[1] if len(desc) > 1 else ""
                        out.append({
                            "situation_type": (it.get("tags") or ["general"])[0],
                            "description": it.get("description", ""),
                            "action": action,
                            "outcome": it.get("outcome", ""),
                            "success": "success" in (it.get("outcome") or "").lower()
                            or "success" in query,
                        })
                except Exception:
                    pass
        return out[:limit]

    def _find_patterns_in_experiences(self) -> List[Dict[str, Any]]:
        """Buscar patrones recurrentes a partir de episodios y memoria semántica."""
        patterns: List[Dict[str, Any]] = []
        episodes = self._get_recent_episodes(limit=150)
        if not episodes:
            if getattr(self.semantic_memory, "recall_similar", None):
                try:
                    recent = self.semantic_memory.recall_similar(
                        "success outcome", top_k=20, min_similarity=0.5
                    )
                    if len(recent) >= 3:
                        patterns.append({
                            "pattern": "repeated_success",
                            "count": len(recent),
                            "source": "semantic",
                        })
                except Exception:
                    pass
            return patterns
        success_count = sum(1 for e in episodes if e.get("success"))
        if success_count >= 3:
            patterns.append({
                "pattern": "repeated_success",
                "count": success_count,
                "total": len(episodes),
                "source": "episodic",
            })
        by_type: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
        for e in episodes:
            t = (e.get("situation_type") or "general").strip() or "general"
            by_type[t].append(e)
        for sit_type, group in by_type.items():
            if len(group) >= 3 and sum(1 for e in group if e.get("success")) >= 2:
                patterns.append({
                    "pattern": "reliable_situation_type",
                    "situation_type": sit_type,
                    "count": len(group),
                    "successes": sum(1 for e in group if e.get("success")),
                })
        return patterns

    def _generalize_from_cases(self) -> List[Dict[str, Any]]:
        """Generalizar de casos (situation_type + action) a reglas de éxito/fallo."""
        generalizations: List[Dict[str, Any]] = []
        episodes = self._get_recent_episodes(limit=200)
        key_to_outcomes: Dict[Tuple[str, str], List[bool]] = defaultdict(list)
        for e in episodes:
            sit = (e.get("situation_type") or "general").strip() or "general"
            action = (e.get("action") or "").strip() or "unknown"
            key_to_outcomes[(sit, action)].append(bool(e.get("success")))
        for (sit, action), outcomes in key_to_outcomes.items():
            if len(outcomes) < 2:
                continue
            success_rate = sum(outcomes) / len(outcomes)
            if success_rate >= 0.75:
                generalizations.append({
                    "situation_type": sit,
                    "action": action,
                    "success_rate": round(success_rate, 2),
                    "n": len(outcomes),
                    "generalization": f"{sit} + {action} → success (conf ~{success_rate:.2f})",
                })
            elif success_rate <= 0.25:
                generalizations.append({
                    "situation_type": sit,
                    "action": action,
                    "success_rate": round(success_rate, 2),
                    "n": len(outcomes),
                    "generalization": f"{sit} + {action} → often fails (conf ~{1 - success_rate:.2f})",
                })
        return generalizations

    def _identify_new_concepts(self) -> List[Dict[str, Any]]:
        """Identificar conceptos que emergen de descripciones repetidas (nombres/tipos)."""
        new_concepts: List[Dict[str, Any]] = []
        episodes = self._get_recent_episodes(limit=100)
        type_counts: Dict[str, int] = defaultdict(int)
        for e in episodes:
            t = (e.get("situation_type") or "general").strip() or "general"
            type_counts[t] += 1
        kb = self.knowledge_base
        existing = set(getattr(kb, "concepts", {}).keys())
        for sit_type, count in type_counts.items():
            if count >= 3 and sit_type not in existing and sit_type != "general":
                new_concepts.append({
                    "name": sit_type,
                    "suggested_type": "situation_type",
                    "occurrences": count,
                    "definition": f"Tipo de situación observado {count} veces en episodios.",
                })
        return new_concepts

    def _update_causal_rules(self) -> List[Dict[str, Any]]:
        """Actualizar confianza de reglas causales según episodios (acción → efecto)."""
        updated: List[Dict[str, Any]] = []
        kb = self.knowledge_base
        relations = getattr(kb, "relations", {})
        if not relations:
            return updated
        episodes = self._get_recent_episodes(limit=150)
        # Mapear acciones en episodios a claves de relations (acción, objeto/concepto)
        for key, rel in list(relations.items()):
            if not isinstance(rel, dict):
                continue
            action_key = key[0] if isinstance(key, (tuple, list)) else str(key)
            conf = rel.get("confidence", 0.8)
            # Contar episodios donde esta acción aparece y resultado positivo/negativo
            success, failure = 0, 0
            for e in episodes:
                act = (e.get("action") or "").lower()
                if action_key.lower() in act:
                    if e.get("success"):
                        success += 1
                    else:
                        failure += 1
            total = success + failure
            if total >= 2:
                new_conf = success / total if total else conf
                new_conf = max(0.2, min(0.98, new_conf))
                rel["confidence"] = round(new_conf, 2)
                updated.append({
                    "rule": str(key),
                    "old_confidence": conf,
                    "new_confidence": rel["confidence"],
                    "evidence": total,
                })
        if updated:
            kb.save_to_disk()
        return updated

    def _clean_contradictions(self) -> List[Dict[str, Any]]:
        """Detectar (situation_type, action) con éxitos y fallos y debilitar creencias."""
        weakened: List[Dict[str, Any]] = []
        episodes = self._get_recent_episodes(limit=200)
        key_to_outcomes: Dict[Tuple[str, str], List[bool]] = defaultdict(list)
        for e in episodes:
            sit = (e.get("situation_type") or "general").strip() or "general"
            action = (e.get("action") or "").strip() or "unknown"
            key_to_outcomes[(sit, action)].append(bool(e.get("success")))
        for (sit, action), outcomes in key_to_outcomes.items():
            if len(outcomes) < 3:
                continue
            has_success = any(outcomes)
            has_failure = not all(outcomes)
            if has_success and has_failure:
                weakened.append({
                    "situation_type": sit,
                    "action": action,
                    "contradiction": "both success and failure observed",
                    "n": len(outcomes),
                    "success_count": sum(outcomes),
                    "failure_count": len(outcomes) - sum(outcomes),
                })
        return weakened

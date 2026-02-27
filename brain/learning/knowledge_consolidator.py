"""Consolida experiencias en conocimiento generalizado (aprendizaje progresivo ATLAS)."""
from __future__ import annotations

import time
from collections import defaultdict
from datetime import datetime, timedelta
from typing import Any, Dict, List, Tuple


class KnowledgeConsolidator:
    """
    Consolida experiencias en conocimiento generalizado.
    Similar a consolidación durante 'sueño': extrae patrones de experiencias.
    Funciones: patrones recurrentes, generalizar casos, conceptos emergentes,
    actualizar relaciones causales, limpiar contradicciones.
    """

    def __init__(
        self,
        semantic_memory: Any,
        episodic_memory: Any,
        knowledge_base: Any,
        consolidation_threshold_hours: int = 4,
    ) -> None:
        self.semantic_memory = semantic_memory
        self.episodic_memory = episodic_memory
        self.knowledge_base = knowledge_base
        self.consolidation_interval = timedelta(hours=consolidation_threshold_hours)
        self.last_consolidation = datetime.now()
        self.consolidation_history: List[Dict[str, Any]] = []
        self.stats: Dict[str, int] = {
            "total_consolidations": 0,
            "patterns_found": 0,
            "concepts_created": 0,
            "rules_updated": 0,
            "contradictions_resolved": 0,
        }

    def should_consolidate(self, force: bool = False) -> bool:
        """Determinar si es momento de consolidar."""
        if force:
            return True
        time_elapsed = datetime.now() - self.last_consolidation
        if time_elapsed > self.consolidation_interval:
            return True
        get_recent = getattr(self.episodic_memory, "get_recent_episodes", None) or getattr(
            self.episodic_memory, "get_recent", None
        )
        if get_recent:
            try:
                recent = get_recent(limit=100) if callable(get_recent) else get_recent(limit=100)
                if len(recent) >= 50:
                    return True
            except Exception:
                pass
        return False

    def consolidate_knowledge(self) -> Dict[str, Any]:
        """
        Ejecutar consolidación completa de conocimiento.
        Returns: reporte de qué se aprendió/consolidó.
        """
        print("[CONSOLIDATOR] 🌙 Iniciando consolidación de conocimiento...")
        self.stats["total_consolidations"] += 1
        start_time = datetime.now()
        report: Dict[str, Any] = {
            "consolidation_id": f"consol_{int(start_time.timestamp())}",
            "timestamp": start_time.isoformat(),
            "new_concepts": [],
            "new_rules": [],
            "updated_relations": [],
            "strengthened_patterns": [],
            "weakened_beliefs": [],
            "generalizations": [],
            "contradictions_resolved": [],
        }
        print("[CONSOLIDATOR] Buscando patrones...")
        patterns = self._find_patterns_in_experiences()
        report["strengthened_patterns"] = patterns
        self.stats["patterns_found"] += len(patterns)
        print("[CONSOLIDATOR] Generalizando conocimiento...")
        report["generalizations"] = self._generalize_from_cases()
        print("[CONSOLIDATOR] Identificando conceptos emergentes...")
        new_concepts = self._identify_new_concepts()
        report["new_concepts"] = new_concepts
        self.stats["concepts_created"] += len(new_concepts)
        print("[CONSOLIDATOR] Actualizando relaciones causales...")
        updated = self._update_causal_rules()
        report["updated_relations"] = updated
        self.stats["rules_updated"] += len(updated)
        print("[CONSOLIDATOR] Resolviendo contradicciones...")
        contradictions = self._resolve_contradictions()
        report["contradictions_resolved"] = contradictions
        self.stats["contradictions_resolved"] += len(contradictions)
        self._apply_consolidated_knowledge(report)
        self.last_consolidation = datetime.now()
        self.consolidation_history.append(report)
        duration = (datetime.now() - start_time).total_seconds()
        print(
            f"[CONSOLIDATOR] ✓ Consolidación completada en {duration:.1f}s - "
            f"{len(patterns)} patrones, {len(new_concepts)} conceptos, {len(updated)} relaciones"
        )
        return report

    def get_statistics(self) -> Dict[str, Any]:
        """Estadísticas del consolidador."""
        return {
            "total_consolidations": self.stats["total_consolidations"],
            "patterns_found_total": self.stats["patterns_found"],
            "concepts_created_total": self.stats["concepts_created"],
            "rules_updated_total": self.stats["rules_updated"],
            "contradictions_resolved_total": self.stats["contradictions_resolved"],
            "last_consolidation": self.last_consolidation.isoformat(),
            "hours_since_last": (datetime.now() - self.last_consolidation).total_seconds() / 3600,
        }

    def _get_recent_episodes(self, limit: int = 200) -> List[Dict[str, Any]]:
        """Obtener episodios recientes (episódica o semántica como fallback)."""
        get_recent = getattr(self.episodic_memory, "get_recent_episodes", None) or getattr(
            self.episodic_memory, "get_recent", None
        )
        if get_recent:
            try:
                raw = get_recent(limit=limit) if callable(get_recent) else get_recent(limit=limit)
                out: List[Dict[str, Any]] = []
                for ep in raw:
                    e = dict(ep)
                    e.setdefault("situation_type", e.get("task_type", "general"))
                    e.setdefault("task_type", e.get("situation_type", "general"))
                    e.setdefault("action", e.get("action_taken", ""))
                    e.setdefault("action_taken", e.get("action", ""))
                    e.setdefault("situation", e.get("description", e.get("situation", "")))
                    out.append(e)
                return out[:limit]
            except Exception:
                pass
        out = []
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
                            "task_type": (it.get("tags") or ["general"])[0],
                            "description": it.get("description", ""),
                            "situation": situation,
                            "action": action,
                            "action_taken": action,
                            "outcome": it.get("outcome", ""),
                            "result": it.get("outcome", ""),
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
                    "task_type": sit_type,
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

    def _resolve_contradictions(self) -> List[Dict[str, Any]]:
        """Detectar y resolver contradicciones en conocimiento (reglas con baja confirmación)."""
        contradictions: List[Dict[str, Any]] = []
        rules = getattr(self.knowledge_base, "rules", {})
        for rule_id, rule in list(rules.items()):
            if not isinstance(rule, dict):
                continue
            times_triggered = rule.get("times_triggered", 0)
            times_successful = rule.get("times_successful", 0)
            if times_triggered >= 5:
                success_rate = times_successful / times_triggered
                if success_rate < 0.4:
                    contradictions.append({
                        "rule_id": rule_id,
                        "rule": rule,
                        "success_rate": success_rate,
                        "action": "weaken_or_remove",
                        "reason": f"Regla solo exitosa {success_rate:.0%} del tiempo",
                    })
                    rule["confidence"] = success_rate
        return contradictions

    def _apply_consolidated_knowledge(self, report: Dict[str, Any]) -> None:
        """Aplicar conocimiento consolidado a la base de conocimiento."""
        kb = self.knowledge_base
        for concept in report.get("new_concepts", []):
            name = concept.get("concept_name") or concept.get("name")
            if not name:
                continue
            if concept.get("needs_definition", True):
                if not hasattr(kb, "concepts"):
                    continue
                kb.concepts[name] = {
                    "type": "emergent",
                    "definition": concept.get("definition", "PENDING_DEFINITION"),
                    "frequency": concept.get("frequency", concept.get("occurrences", 0)),
                    "confidence": concept.get("confidence", 0.7),
                    "learned_from": "consolidation",
                    "learned_at": datetime.now().isoformat(),
                }
            elif hasattr(kb, "add_learned_concept"):
                kb.add_learned_concept(
                    name=name,
                    definition=concept.get("definition", ""),
                    concept_type=concept.get("suggested_type", "learned"),
                )
        for gen in report.get("generalizations", []):
            action = gen.get("action") or (gen.get("situation_type", "") + "_" + gen.get("action", ""))
            if not action:
                continue
            rule_id = f"rule_{str(action).replace(' ', '_')[:50]}"
            if hasattr(kb, "add_learned_rule") and rule_id not in getattr(kb, "rules", {}):
                typical = gen.get("typical_result", "success" if gen.get("success_rate", 0) >= 0.5 else "failure")
                kb.add_learned_rule(
                    rule_id=rule_id,
                    condition=f"action == '{action}'",
                    action=f"expect_{typical}",
                    confidence=gen.get("confidence", gen.get("success_rate", 0.7)),
                    source="consolidation",
                )
        if hasattr(kb, "save_to_disk"):
            kb.save_to_disk()

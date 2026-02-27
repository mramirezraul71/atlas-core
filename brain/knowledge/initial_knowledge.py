"""Base de conocimiento inicial del robot. Lo que el robot 'sabe' al nacer (aprendizaje progresivo ATLAS)."""
from __future__ import annotations

import json
import os
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional


def _knowledge_path() -> Path:
    root = os.getenv("ATLAS_REPO_PATH") or os.getenv("ATLAS_PUSH_ROOT") or ""
    if root:
        return Path(root).resolve() / "brain" / "knowledge" / "initial_kb.json"
    return Path(__file__).resolve().parent / "initial_kb.json"


class InitialKnowledgeBase:
    """
    Base de conocimiento inicial del robot.
    Esto es lo que el robot "sabe" al nacer. Después crece con experiencias y consultas a LLM.
    """

    def __init__(self, storage_path: Optional[str] = None) -> None:
        self.storage_path = Path(storage_path) if storage_path else _knowledge_path()
        self.concepts: Dict[str, Any] = {}
        self.skills: Dict[str, Any] = {}
        self.rules: Dict[str, Any] = {}
        self.relations: Dict[tuple, Any] = {}
        self.metadata: Dict[str, Any] = {
            "created_at": datetime.now().isoformat(),
            "version": "1.0",
            "total_learned_concepts": 0,
        }
        if not self.load_from_disk():
            self._initialize_base_knowledge()
            self.save_to_disk()

    def _initialize_base_knowledge(self) -> None:
        """Cargar conocimiento básico inicial."""

        # ============ CONCEPTOS BÁSICOS ============
        self.concepts = {
            "object": {
                "type": "physical_entity",
                "definition": "Any physical thing that can be perceived and manipulated",
                "properties": ["position", "size", "color", "weight", "shape"],
                "can": ["move", "grab", "observe", "identify"],
                "is_a": None,
                "learned_from": "initial_knowledge",
            },
            "container": {
                "type": "object",
                "definition": "Object that can hold other objects inside",
                "is_a": "object",
                "properties": ["capacity", "contents", "open_status"],
                "can": ["contain", "store", "protect"],
                "examples": ["box", "bag", "cup", "drawer"],
                "learned_from": "initial_knowledge",
            },
            "tool": {
                "type": "object",
                "definition": "Object designed to perform specific tasks",
                "is_a": "object",
                "properties": ["function", "usage_method", "maintenance_needs"],
                "can": ["perform_task", "extend_capability"],
                "examples": ["hammer", "wrench", "knife", "screwdriver"],
                "learned_from": "initial_knowledge",
            },
            "surface": {
                "type": "physical_entity",
                "definition": "Flat or curved area where objects can be placed",
                "properties": ["height", "material", "stability", "area"],
                "can": ["support_object", "provide_workspace"],
                "examples": ["table", "floor", "shelf", "counter"],
                "learned_from": "initial_knowledge",
            },
            "move": {
                "type": "action",
                "definition": "Change position from one location to another",
                "requires": ["target_position"],
                "effects": ["position_changed"],
                "can_fail": True,
                "failure_reasons": ["obstacle_blocking", "unreachable", "collision"],
                "learned_from": "initial_knowledge",
            },
            "grab": {
                "type": "action",
                "definition": "Use gripper to hold an object",
                "requires": ["object", "gripper_available"],
                "effects": ["object_held", "gripper_occupied"],
                "preconditions": ["object_reachable", "object_graspable"],
                "can_fail": True,
                "failure_reasons": ["object_too_far", "object_too_heavy", "object_slippery"],
                "learned_from": "initial_knowledge",
            },
            "place": {
                "type": "action",
                "definition": "Release held object at target position",
                "requires": ["object_held", "target_position"],
                "effects": ["object_at_position", "gripper_free"],
                "preconditions": ["holding_object"],
                "can_fail": True,
                "failure_reasons": ["target_unreachable", "unstable_placement"],
                "learned_from": "initial_knowledge",
            },
            "push": {
                "type": "action",
                "definition": "Apply force to move object without grasping",
                "requires": ["object", "direction", "force"],
                "effects": ["object_displaced"],
                "can_fail": True,
                "failure_reasons": ["object_too_heavy", "object_fixed"],
                "learned_from": "initial_knowledge",
            },
            "pull": {
                "type": "action",
                "definition": "Apply force to bring object closer",
                "requires": ["object", "contact_point"],
                "effects": ["object_moved_closer"],
                "can_fail": True,
                "learned_from": "initial_knowledge",
            },
            "success": {
                "type": "state",
                "definition": "Task completed successfully",
                "means": "goal_achieved",
                "should": "reinforce_strategy",
                "learned_from": "initial_knowledge",
            },
            "failure": {
                "type": "state",
                "definition": "Task did not complete successfully",
                "means": "goal_not_achieved",
                "should": "analyze_reason_and_learn",
                "learned_from": "initial_knowledge",
            },
            "uncertain": {
                "type": "state",
                "definition": "Not enough information or confidence to proceed",
                "means": "need_more_information",
                "should": "ask_ai_or_human_or_explore",
                "learned_from": "initial_knowledge",
            },
            "blocked": {
                "type": "state",
                "definition": "Cannot proceed due to obstacle or constraint",
                "means": "path_or_action_obstructed",
                "should": "find_alternative_or_remove_obstacle",
                "learned_from": "initial_knowledge",
            },
            "fragile": {
                "type": "property",
                "definition": "Easily broken or damaged",
                "applies_to": "object",
                "handling_rule": "handle_carefully",
                "examples": ["glass", "egg", "ceramic"],
                "learned_from": "initial_knowledge",
            },
            "heavy": {
                "type": "property",
                "definition": "Has significant weight requiring more force",
                "applies_to": "object",
                "handling_rule": "use_more_force_or_assistance",
                "threshold": "weight > gripper_capacity * 0.8",
                "learned_from": "initial_knowledge",
            },
            "slippery": {
                "type": "property",
                "definition": "Difficult to grip due to smooth or wet surface",
                "applies_to": "object",
                "handling_rule": "increase_grip_force_or_use_tool",
                "causes": ["wet", "smooth_material", "oily"],
                "learned_from": "initial_knowledge",
            },
        }

        # ============ SKILLS BÁSICAS ============
        self.skills = {
            "identify_object": {
                "description": "Identify what an object is using vision",
                "category": "perception",
                "inputs": ["image", "bounding_box"],
                "outputs": ["object_name", "confidence", "properties"],
                "uses_tools": ["yolo_detection", "scene_understanding"],
                "when_uncertain": "confidence < 0.7",
                "fallback": "ask_llm_for_identification",
                "learned_from": "initial_knowledge",
            },
            "pick_and_place": {
                "description": "Pick up object and place it at target location",
                "category": "manipulation",
                "inputs": ["object_id", "target_position"],
                "outputs": ["success", "final_position", "errors"],
                "steps": [
                    "approach_object",
                    "open_gripper",
                    "align_gripper",
                    "close_gripper",
                    "lift_object",
                    "move_to_target",
                    "lower_object",
                    "open_gripper",
                ],
                "can_fail": True,
                "failure_recovery": "retry_with_adjustment_or_ask_help",
                "learned_from": "initial_knowledge",
            },
            "search_memory": {
                "description": "Search for similar past experiences in memory",
                "category": "reasoning",
                "inputs": ["situation_description", "context"],
                "outputs": ["similar_experiences", "similarity_scores"],
                "uses_tools": ["semantic_memory"],
                "when_to_use": "facing_new_or_uncertain_situation",
                "learned_from": "initial_knowledge",
            },
            "ask_for_help": {
                "description": "Request assistance when uncertain or failed repeatedly",
                "category": "meta",
                "inputs": ["situation", "uncertainty_reason", "previous_attempts"],
                "outputs": ["guidance", "new_knowledge", "suggested_action"],
                "uses_tools": ["llm_consultant"],
                "triggers": [
                    "uncertainty_score > 0.6",
                    "repeated_failures >= 3",
                    "completely_novel_situation",
                ],
                "learned_from": "initial_knowledge",
            },
            "explore_object": {
                "description": "Learn about unfamiliar object through interaction",
                "category": "exploration",
                "inputs": ["object_id"],
                "outputs": ["properties_discovered", "affordances"],
                "steps": [
                    "observe_visual_properties",
                    "touch_gently",
                    "test_weight",
                    "try_basic_interactions",
                ],
                "safety_constraint": "low_risk_actions_only",
                "learned_from": "initial_knowledge",
            },
            "plan_task": {
                "description": "Decompose high-level goal into actionable steps",
                "category": "planning",
                "inputs": ["goal", "current_state", "available_actions"],
                "outputs": ["action_sequence", "expected_outcome"],
                "uses_tools": ["world_model", "causal_reasoning"],
                "when_uncertain": "no_clear_plan_or_multiple_options",
                "learned_from": "initial_knowledge",
            },
        }

        # ============ REGLAS HEURÍSTICAS ============
        self.rules = {
            "rule_uncertainty_threshold": {
                "name": "Ask for help when uncertain",
                "condition": "confidence < 0.6",
                "action": "call_llm_consultant",
                "priority": "high",
                "explanation": "Low confidence indicates lack of knowledge - consult AI",
                "learned_from": "initial_knowledge",
            },
            "rule_repeated_failure": {
                "name": "Change strategy after repeated failures",
                "condition": "same_task_failed >= 3 times",
                "action": "try_different_approach_or_ask_help",
                "priority": "high",
                "explanation": "Repeating failed approach is inefficient - need new strategy",
                "learned_from": "initial_knowledge",
            },
            "rule_novel_situation": {
                "name": "Explore cautiously in new situations",
                "condition": "no_similar_experience_in_memory",
                "action": "explore_cautiously_and_ask_if_risky",
                "priority": "medium",
                "explanation": "Novel situations require careful exploration to learn safely",
                "learned_from": "initial_knowledge",
            },
            "rule_consolidation": {
                "name": "Consolidate knowledge periodically",
                "condition": "time_since_last_consolidation > 1_hour OR new_experiences > 50",
                "action": "run_knowledge_consolidation",
                "priority": "low",
                "explanation": "Regular consolidation extracts patterns from experiences",
                "learned_from": "initial_knowledge",
            },
            "rule_fragile_objects": {
                "name": "Handle fragile objects carefully",
                "condition": "object.has_property('fragile')",
                "action": "reduce_grip_force AND move_slowly",
                "priority": "high",
                "explanation": "Fragile objects break easily - require gentle handling",
                "learned_from": "initial_knowledge",
            },
            "rule_verify_before_irreversible": {
                "name": "Verify before irreversible actions",
                "condition": "action.is_irreversible AND uncertainty > 0.3",
                "action": "ask_for_confirmation",
                "priority": "critical",
                "explanation": "Irreversible actions (cut, break, delete) need high confidence",
                "learned_from": "initial_knowledge",
            },
        }

        # ============ RELACIONES CAUSALES ============
        self.relations = {
            ("push", "object"): {
                "relation": "causes",
                "effect": "object_moves",
                "confidence": 0.95,
                "conditions": ["object_movable", "sufficient_force"],
                "learned_from": "initial_knowledge",
            },
            ("grab", "object"): {
                "relation": "requires",
                "precondition": "object_reachable AND object_graspable",
                "confidence": 0.99,
                "learned_from": "initial_knowledge",
            },
            ("drop", "fragile_object"): {
                "relation": "causes",
                "effect": "object_breaks",
                "confidence": 0.8,
                "probability": "depends_on_height_and_material",
                "learned_from": "initial_knowledge",
            },
            ("place_on_unstable_surface", "object"): {
                "relation": "causes",
                "effect": "object_falls",
                "confidence": 0.7,
                "learned_from": "initial_knowledge",
            },
            ("wet_surface", "slippery"): {
                "relation": "causes",
                "effect": "difficult_to_grip",
                "confidence": 0.9,
                "learned_from": "initial_knowledge",
            },
        }

    # ============ MÉTODOS DE CONSULTA ============

    def get_concept(self, name: str) -> Optional[Dict[str, Any]]:
        """Obtener definición de concepto."""
        return self.concepts.get(name)

    def get_skill(self, name: str) -> Optional[Dict[str, Any]]:
        """Obtener definición de skill."""
        return self.skills.get(name)

    def get_rule(self, name: str) -> Optional[Dict[str, Any]]:
        """Obtener definición de regla."""
        return self.rules.get(name)

    def get_related_concepts(self, concept: str) -> List[str]:
        """Obtener conceptos relacionados vía jerarquía is_a (padres e hijos)."""
        related: List[str] = []
        concept_def = self.concepts.get(concept, {})
        if isinstance(concept_def, dict) and concept_def.get("is_a"):
            parent = concept_def["is_a"]
            related.append(parent)
            related.extend(self.get_related_concepts(parent))
        for name, defn in self.concepts.items():
            if isinstance(defn, dict) and defn.get("is_a") == concept:
                related.append(name)
        return list(dict.fromkeys(related))

    def find_applicable_rules(self, situation: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Encontrar reglas aplicables a la situación (ordenadas por prioridad)."""
        applicable = []
        for rule_id, rule in self.rules.items():
            applicable.append({
                "rule_id": rule_id,
                "rule": rule,
                "priority": rule.get("priority", "medium"),
            })
        priority_order = {"critical": 0, "high": 1, "medium": 2, "low": 3}
        applicable.sort(key=lambda x: priority_order.get(x["priority"], 4))
        return applicable

    # ============ APRENDIZAJE DINÁMICO ============

    def add_learned_concept(
        self,
        name: str,
        definition: str,
        concept_type: str = "learned",
        properties: Optional[List[str]] = None,
        examples: Optional[List[str]] = None,
        confidence: float = 0.8,
        source: str = "llm_consultation",
    ) -> bool:
        """Añadir concepto aprendido durante operación. Retorna True si era nuevo."""
        if name in self.concepts:
            existing = self.concepts[name]
            existing["confidence"] = max(existing.get("confidence", 0.5), confidence)
            existing["last_reinforced"] = datetime.now().isoformat()
            self.save_to_disk()
            return False
        self.concepts[name] = {
            "type": concept_type,
            "definition": definition,
            "properties": properties or [],
            "examples": examples or [],
            "confidence": confidence,
            "learned_from": source,
            "learned_at": datetime.now().isoformat(),
            "times_used": 0,
            "times_confirmed": 0,
        }
        self.metadata["total_learned_concepts"] = self.metadata.get("total_learned_concepts", 0) + 1
        self.save_to_disk()
        return True

    def add_learned_skill(
        self,
        name: str,
        description: str,
        category: str,
        code: Optional[str] = None,
        confidence: float = 0.7,
        source: str = "self_programming",
    ) -> bool:
        """Añadir skill aprendida o generada. Retorna True si era nueva."""
        if name in self.skills:
            return False
        self.skills[name] = {
            "description": description,
            "category": category,
            "code": code,
            "confidence": confidence,
            "learned_from": source,
            "learned_at": datetime.now().isoformat(),
            "times_used": 0,
            "success_rate": 0.0,
        }
        self.save_to_disk()
        return True

    def add_learned_rule(
        self,
        rule_id: str,
        condition: str,
        action: str,
        confidence: float = 0.7,
        source: str = "experience",
    ) -> bool:
        """Añadir regla aprendida de experiencia. Retorna True si era nueva."""
        if rule_id in self.rules:
            return False
        self.rules[rule_id] = {
            "name": rule_id,
            "condition": condition,
            "action": action,
            "confidence": confidence,
            "learned_from": source,
            "learned_at": datetime.now().isoformat(),
            "times_triggered": 0,
            "times_successful": 0,
        }
        self.save_to_disk()
        return True

    def update_causal_relation(
        self,
        cause: str,
        effect: str,
        observed: bool,
        strength: float = 0.1,
    ) -> None:
        """Actualizar relación causal según observación (reforzar o debilitar)."""
        key = (cause, effect)
        if key not in self.relations:
            self.relations[key] = {
                "relation": "causes",
                "effect": effect,
                "confidence": strength if observed else 0.1,
                "observations": 1,
                "confirmations": 1 if observed else 0,
                "learned_from": "experience",
                "learned_at": datetime.now().isoformat(),
            }
        else:
            rel = self.relations[key]
            rel["observations"] = rel.get("observations", 0) + 1
            if observed:
                rel["confirmations"] = rel.get("confirmations", 0) + 1
            rel["confidence"] = rel["confirmations"] / rel["observations"]
            rel["last_observed"] = datetime.now().isoformat()
        self.save_to_disk()

    # ============ PERSISTENCIA ============

    def save_to_disk(self, path: Optional[str] = None) -> str:
        """Guardar base de conocimiento a disco. Retorna ruta usada."""
        p = Path(path) if path else self.storage_path
        p.parent.mkdir(parents=True, exist_ok=True)
        self.metadata["last_saved"] = datetime.now().isoformat()
        relations_ser = {f"{k[0]}->{k[1]}": v for k, v in self.relations.items()}
        data = {
            "metadata": self.metadata,
            "concepts": self.concepts,
            "skills": self.skills,
            "rules": self.rules,
            "relations": relations_ser,
            "saved_at": datetime.now().isoformat(),
        }
        with open(p, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        return str(p)

    def load_from_disk(self, path: Optional[str] = None) -> bool:
        """Cargar base de conocimiento desde disco. Retorna True si hubo archivo."""
        p = Path(path) if path else self.storage_path
        if not p.exists():
            return False
        try:
            with open(p, "r", encoding="utf-8") as f:
                data = json.load(f)
            self.metadata = data.get("metadata", self.metadata)
            self.concepts = data.get("concepts", {})
            self.skills = data.get("skills", {})
            self.rules = data.get("rules", {})
            rel = data.get("relations", {})
            self.relations = {}
            for kstr, v in rel.items():
                if "->" in kstr:
                    cause, effect = kstr.split("->", 1)
                    self.relations[(cause.strip(), effect.strip())] = v
                else:
                    try:
                        k = tuple(json.loads(kstr))
                        self.relations[k] = v
                    except Exception:
                        pass
            return True
        except Exception:
            return False

    # ============ ESTADÍSTICAS ============

    def get_statistics(self) -> Dict[str, Any]:
        """Estadísticas de la base de conocimiento."""
        learned_concepts = sum(
            1 for c in self.concepts.values()
            if isinstance(c, dict) and c.get("learned_from") != "initial_knowledge"
        )
        return {
            "total_concepts": len(self.concepts),
            "initial_concepts": len(self.concepts) - learned_concepts,
            "learned_concepts": learned_concepts,
            "total_skills": len(self.skills),
            "total_rules": len(self.rules),
            "total_relations": len(self.relations),
            "storage_path": str(self.storage_path),
            "last_saved": self.metadata.get("last_saved", "never"),
        }

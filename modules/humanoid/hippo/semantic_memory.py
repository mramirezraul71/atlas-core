"""
SemanticMemory: Memoria semantica del hipocampo.

Analogo biologico: Corteza entorrinal + neocorteza
- Conocimiento conceptual (grafo)
- Relaciones entre conceptos
- Inferencia basica
"""
from __future__ import annotations

import json
import logging
import os
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple

from .schemas import Concept, Relation

logger = logging.getLogger(__name__)


class SemanticMemory:
    """
    Memoria semantica del hipocampo.
    
    Almacena conocimiento conceptual como un grafo de conceptos y relaciones.
    """
    
    def __init__(self, 
                 storage_path: str = None,
                 embedding_model: Any = None):
        """
        Inicializa memoria semantica.
        
        Args:
            storage_path: Ruta para persistencia
            embedding_model: Modelo para generar embeddings
        """
        self.storage_path = Path(storage_path) if storage_path else None
        self.embedding_model = embedding_model
        
        # Grafo de conceptos
        self._concepts: Dict[str, Concept] = {}
        
        # Indices
        self._by_name: Dict[str, str] = {}  # nombre -> concept_id
        self._by_type: Dict[str, Set[str]] = {}  # tipo -> set(concept_ids)
        
        # Cargar datos persistidos
        if self.storage_path:
            self._load_from_disk()
    
    def _load_from_disk(self) -> None:
        """Carga conceptos desde disco."""
        if not self.storage_path or not self.storage_path.exists():
            return
        
        concepts_file = self.storage_path / "concepts.json"
        if not concepts_file.exists():
            return
        
        try:
            with open(concepts_file, "r", encoding="utf-8") as f:
                data = json.load(f)
            
            for c_data in data.get("concepts", []):
                concept = self._deserialize_concept(c_data)
                if concept:
                    self._add_to_indices(concept)
            
            logger.info(f"Loaded {len(self._concepts)} concepts from disk")
            
        except Exception as e:
            logger.error(f"Error loading concepts: {e}")
    
    def _save_to_disk(self) -> None:
        """Guarda conceptos a disco."""
        if not self.storage_path:
            return
        
        self.storage_path.mkdir(parents=True, exist_ok=True)
        concepts_file = self.storage_path / "concepts.json"
        
        try:
            data = {
                "version": "1.0",
                "count": len(self._concepts),
                "concepts": [
                    c.to_full_dict() for c in self._concepts.values()
                ],
            }
            
            with open(concepts_file, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
                
        except Exception as e:
            logger.error(f"Error saving concepts: {e}")
    
    def _deserialize_concept(self, data: Dict) -> Optional[Concept]:
        """Deserializa concepto desde diccionario."""
        try:
            relations = []
            for r_data in data.get("relations", []):
                relations.append(Relation(**r_data))
            
            concept = Concept(
                id=data.get("id"),
                name=data.get("name", ""),
                concept_type=data.get("concept_type", "object"),
                properties=data.get("properties", {}),
                relations=relations,
                embedding=data.get("embedding"),
                confidence=data.get("confidence", 1.0),
                source=data.get("source", "loaded"),
                access_count=data.get("access_count", 0),
            )
            
            self._concepts[concept.id] = concept
            return concept
            
        except Exception as e:
            logger.error(f"Error deserializing concept: {e}")
            return None
    
    def _add_to_indices(self, concept: Concept) -> None:
        """Agrega concepto a indices."""
        self._concepts[concept.id] = concept
        
        # Indice por nombre
        name_lower = concept.name.lower()
        self._by_name[name_lower] = concept.id
        
        # Indice por tipo
        if concept.concept_type not in self._by_type:
            self._by_type[concept.concept_type] = set()
        self._by_type[concept.concept_type].add(concept.id)
    
    def _remove_from_indices(self, concept: Concept) -> None:
        """Remueve concepto de indices."""
        name_lower = concept.name.lower()
        if name_lower in self._by_name:
            del self._by_name[name_lower]
        
        if concept.concept_type in self._by_type:
            self._by_type[concept.concept_type].discard(concept.id)
    
    def add_concept(self, concept: Concept) -> str:
        """
        Agrega o actualiza un concepto.
        
        Args:
            concept: Concepto a agregar
        
        Returns:
            ID del concepto
        """
        # Generar embedding si no tiene
        if not concept.embedding and self.embedding_model:
            concept.embedding = self._generate_embedding(concept)
        
        self._add_to_indices(concept)
        self._save_to_disk()
        
        logger.debug(f"Concept added: {concept.id} - {concept.name}")
        return concept.id
    
    def create_concept(self, name: str, concept_type: str = "object",
                      properties: Dict[str, Any] = None,
                      source: str = "created") -> Concept:
        """
        Crea y agrega un nuevo concepto.
        
        Args:
            name: Nombre del concepto
            concept_type: Tipo de concepto
            properties: Propiedades iniciales
            source: Fuente del concepto
        
        Returns:
            Concepto creado
        """
        concept = Concept(
            name=name,
            concept_type=concept_type,
            properties=properties or {},
            source=source,
        )
        
        self.add_concept(concept)
        return concept
    
    def _generate_embedding(self, concept: Concept) -> Optional[List[float]]:
        """Genera embedding para un concepto."""
        if not self.embedding_model:
            return None
        
        # Construir texto para embedding
        text = f"{concept.name} {concept.concept_type}"
        for key, value in concept.properties.items():
            text += f" {key} {value}"
        
        try:
            if hasattr(self.embedding_model, "encode"):
                return self.embedding_model.encode(text).tolist()
            elif hasattr(self.embedding_model, "embed"):
                return self.embedding_model.embed(text)
        except Exception as e:
            logger.error(f"Error generating embedding: {e}")
        
        return None
    
    def get(self, concept_id: str) -> Optional[Concept]:
        """Obtiene concepto por ID."""
        concept = self._concepts.get(concept_id)
        if concept:
            concept.access()
        return concept
    
    def get_by_name(self, name: str) -> Optional[Concept]:
        """Obtiene concepto por nombre."""
        name_lower = name.lower()
        concept_id = self._by_name.get(name_lower)
        if concept_id:
            return self.get(concept_id)
        return None
    
    def get_by_type(self, concept_type: str) -> List[Concept]:
        """Obtiene todos los conceptos de un tipo."""
        concept_ids = self._by_type.get(concept_type, set())
        return [self._concepts[cid] for cid in concept_ids if cid in self._concepts]
    
    def search(self, query: str, limit: int = 10) -> List[Concept]:
        """
        Busca conceptos por texto.
        
        Args:
            query: Texto de busqueda
            limit: Maximo de resultados
        
        Returns:
            Lista de conceptos encontrados
        """
        query_lower = query.lower()
        results = []
        
        # Busqueda por nombre
        for name, concept_id in self._by_name.items():
            if query_lower in name:
                concept = self._concepts.get(concept_id)
                if concept:
                    results.append((concept, 2))  # Score alto por match de nombre
        
        # Busqueda en propiedades
        for concept in self._concepts.values():
            if concept in [r[0] for r in results]:
                continue
            
            score = 0
            for key, value in concept.properties.items():
                if query_lower in str(value).lower():
                    score += 1
            
            if score > 0:
                results.append((concept, score))
        
        # Ordenar por score y limitar
        results.sort(key=lambda x: -x[1])
        return [r[0] for r in results[:limit]]
    
    def add_relation(self, source_id: str, relation_type: str, 
                    target_id: str, confidence: float = 1.0) -> bool:
        """
        Agrega relacion entre conceptos.
        
        Args:
            source_id: ID del concepto fuente
            relation_type: Tipo de relacion
            target_id: ID del concepto destino
            confidence: Confianza en la relacion
        
        Returns:
            True si se agrego exitosamente
        """
        source = self._concepts.get(source_id)
        target = self._concepts.get(target_id)
        
        if not source or not target:
            return False
        
        source.add_relation(relation_type, target_id, confidence)
        self._save_to_disk()
        
        return True
    
    def get_related(self, concept_id: str, 
                   relation_type: str = None) -> List[Tuple[Concept, Relation]]:
        """
        Obtiene conceptos relacionados.
        
        Args:
            concept_id: ID del concepto
            relation_type: Filtrar por tipo de relacion (opcional)
        
        Returns:
            Lista de (concepto_relacionado, relacion)
        """
        concept = self._concepts.get(concept_id)
        if not concept:
            return []
        
        results = []
        relations = concept.get_relations(relation_type)
        
        for relation in relations:
            related = self._concepts.get(relation.target_id)
            if related:
                results.append((related, relation))
        
        return results
    
    def query_graph(self, start_id: str, path: List[str], 
                   max_depth: int = 3) -> List[Concept]:
        """
        Consulta el grafo siguiendo un camino de relaciones.
        
        Args:
            start_id: ID del concepto inicial
            path: Lista de tipos de relacion a seguir
            max_depth: Profundidad maxima
        
        Returns:
            Lista de conceptos encontrados
        """
        if not path or max_depth <= 0:
            return []
        
        current = self._concepts.get(start_id)
        if not current:
            return []
        
        results = [current]
        
        for relation_type in path[:max_depth]:
            next_results = []
            for concept in results:
                related = self.get_related(concept.id, relation_type)
                for rel_concept, _ in related:
                    if rel_concept not in next_results:
                        next_results.append(rel_concept)
            
            if not next_results:
                break
            results = next_results
        
        return results
    
    def infer_type(self, concept_id: str) -> Optional[str]:
        """
        Infiere el tipo de un concepto basado en sus relaciones.
        
        Args:
            concept_id: ID del concepto
        
        Returns:
            Tipo inferido o None
        """
        concept = self._concepts.get(concept_id)
        if not concept:
            return None
        
        # Buscar relacion "is_a"
        is_a = concept.get_relations("is_a")
        if is_a:
            parent = self._concepts.get(is_a[0].target_id)
            if parent:
                return parent.concept_type
        
        return concept.concept_type
    
    def remove_concept(self, concept_id: str) -> bool:
        """Elimina un concepto."""
        concept = self._concepts.get(concept_id)
        if not concept:
            return False
        
        self._remove_from_indices(concept)
        del self._concepts[concept_id]
        
        # Eliminar relaciones que apuntan a este concepto
        for c in self._concepts.values():
            c.relations = [r for r in c.relations if r.target_id != concept_id]
        
        self._save_to_disk()
        return True
    
    def get_stats(self) -> Dict[str, Any]:
        """Obtiene estadisticas de la memoria."""
        by_type = {}
        total_relations = 0
        
        for concept in self._concepts.values():
            ct = concept.concept_type
            by_type[ct] = by_type.get(ct, 0) + 1
            total_relations += len(concept.relations)
        
        return {
            "total_concepts": len(self._concepts),
            "by_type": by_type,
            "total_relations": total_relations,
            "avg_relations_per_concept": total_relations / len(self._concepts) if self._concepts else 0,
        }
    
    def initialize_basic_knowledge(self) -> None:
        """Inicializa conocimiento basico si esta vacio."""
        if self._concepts:
            return
        
        # Conceptos basicos de tipos
        self.create_concept("object", "category", {"description": "Physical thing"}, "initial")
        self.create_concept("location", "category", {"description": "Place or area"}, "initial")
        self.create_concept("person", "category", {"description": "Human being"}, "initial")
        self.create_concept("action", "category", {"description": "Something that can be done"}, "initial")
        
        # Objetos comunes
        self.create_concept("cup", "object", {"graspable": True, "contains": "liquid"}, "initial")
        self.create_concept("table", "object", {"surface": True, "furniture": True}, "initial")
        self.create_concept("chair", "object", {"sittable": True, "furniture": True}, "initial")
        self.create_concept("bottle", "object", {"graspable": True, "contains": "liquid"}, "initial")
        
        # Ubicaciones comunes
        self.create_concept("kitchen", "location", {"room_type": "service"}, "initial")
        self.create_concept("living_room", "location", {"room_type": "social"}, "initial")
        
        # Agregar algunas relaciones
        cup = self.get_by_name("cup")
        table = self.get_by_name("table")
        kitchen = self.get_by_name("kitchen")
        
        if cup and table:
            self.add_relation(cup.id, "can_be_on", table.id)
        if table and kitchen:
            self.add_relation(table.id, "located_in", kitchen.id)
        
        logger.info("Initialized basic semantic knowledge")

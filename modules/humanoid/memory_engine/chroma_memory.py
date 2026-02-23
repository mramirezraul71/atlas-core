"""
ChromaDB Memory Engine - Vector database para memoria semántica y episódica
Integrado con Unified Memory Cortex de ATLAS
"""
from __future__ import annotations

import json
import uuid
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Union

import chromadb
from chromadb.config import Settings
from chromadb.utils import embedding_functions


class ChromaMemoryEngine:
    """
    Motor de memoria vectorial usando ChromaDB.
    
    Características:
    - Almacenamiento vectorial de embeddings
    - Búsqueda semántica por similaridad
    - Metadatos flexibles para episodios y conceptos
    - Integración con Unified Memory Cortex
    """
    
    def __init__(self, persist_directory: Optional[str] = None, collection_name: str = "atlas_memory"):
        if persist_directory is None:
            root = Path(__file__).resolve().parents[3]
            persist_directory = str(root / "logs" / "chroma_db")
        
        self.persist_directory = Path(persist_directory)
        self.persist_directory.mkdir(parents=True, exist_ok=True)
        
        # Cliente ChromaDB con persistencia
        self.client = chromadb.PersistentClient(path=str(self.persist_directory))
        
        # Función de embeddings (sentence-transformers)
        self.embedding_function = embedding_functions.SentenceTransformerEmbeddingFunction(
            model_name="all-MiniLM-L6-v2"
        )
        
        # Colección principal
        self.collection_name = collection_name
        self.collection = self._get_or_create_collection()
        
        print(f"✅ ChromaDB Memory Engine iniciado en {self.persist_directory}")
    
    def _get_or_create_collection(self):
        """Obtener o crear colección con configuración."""
        try:
            collection = self.client.get_collection(
                name=self.collection_name,
                embedding_function=self.embedding_function
            )
        except Exception:
            collection = self.client.create_collection(
                name=self.collection_name,
                embedding_function=self.embedding_function,
                metadata={"description": "Atlas Unified Memory"}
            )
        return collection
    
    def add_memory(
        self,
        content: str,
        memory_type: str,
        metadata: Optional[Dict[str, Any]] = None,
        ids: Optional[str] = None
    ) -> str:
        """
        Añadir memoria a ChromaDB.
        
        Args:
            content: Contenido textual de la memoria
            memory_type: episodic, semantic, procedural, etc
            metadata: Metadatos adicionales
            ids: ID único (opcional, se genera si no se proporciona)
        
        Returns:
            ID de la memoria añadida
        """
        if ids is None:
            ids = str(uuid.uuid4())
        
        # Metadatos base
        base_metadata = {
            "memory_type": memory_type,
            "timestamp": datetime.now().isoformat(),
            "created_at": datetime.now().timestamp()
        }
        
        # Combinar metadatos
        if metadata:
            base_metadata.update(metadata)
        
        # Añadir a ChromaDB
        self.collection.add(
            documents=[content],
            metadatas=[base_metadata],
            ids=[ids]
        )
        
        return ids
    
    def search_memories(
        self,
        query: str,
        memory_types: Optional[List[str]] = None,
        limit: int = 10,
        min_score: float = 0.0
    ) -> List[Dict[str, Any]]:
        """
        Buscar memorias por similaridad semántica.
        
        Args:
            query: Query de búsqueda
            memory_types: Tipos de memoria a filtrar
            limit: Límite de resultados
            min_score: Puntuación mínima de similaridad
        
        Returns:
            Lista de memorias encontradas
        """
        # Construir where clause para filtrar tipos
        where_clause = {}
        if memory_types:
            where_clause["memory_type"] = {"$in": memory_types}
        
        # Búsqueda en ChromaDB
        results = self.collection.query(
            query_texts=[query],
            n_results=limit,
            where=where_clause if where_clause else None
        )
        
        # Procesar resultados
        memories = []
        for i, doc_id in enumerate(results["ids"][0]):
            distance = results["distances"][0][i]
            score = 1 - distance  # Convertir distance a similarity score
            
            if score >= min_score:
                memory = {
                    "id": doc_id,
                    "content": results["documents"][0][i],
                    "metadata": results["metadatas"][0][i],
                    "score": score,
                    "source": "chromadb"
                }
                memories.append(memory)
        
        return memories
    
    def get_memory_by_id(self, memory_id: str) -> Optional[Dict[str, Any]]:
        """Obtener memoria específica por ID."""
        try:
            results = self.collection.get(ids=[memory_id])
            
            if results["ids"]:
                return {
                    "id": results["ids"][0],
                    "content": results["documents"][0],
                    "metadata": results["metadatas"][0],
                    "source": "chromadb"
                }
        except Exception:
            pass
        
        return None
    
    def update_memory(
        self,
        memory_id: str,
        content: Optional[str] = None,
        metadata: Optional[Dict[str, Any]] = None
    ) -> bool:
        """Actualizar memoria existente."""
        try:
            # Obtener memoria actual
            current = self.get_memory_by_id(memory_id)
            if not current:
                return False
            
            # Preparar actualización
            new_content = content if content is not None else current["content"]
            new_metadata = metadata if metadata is not None else current["metadata"]
            new_metadata["updated_at"] = datetime.now().timestamp()
            
            # Actualizar en ChromaDB
            self.collection.update(
                ids=[memory_id],
                documents=[new_content],
                metadatas=[new_metadata]
            )
            
            return True
        except Exception:
            return False
    
    def delete_memory(self, memory_id: str) -> bool:
        """Eliminar memoria."""
        try:
            self.collection.delete(ids=[memory_id])
            return True
        except Exception:
            return False
    
    def get_statistics(self) -> Dict[str, Any]:
        """Obtener estadísticas del motor de memoria."""
        try:
            count = self.collection.count()
            
            # Obtener distribución por tipos
            all_results = self.collection.get()
            type_counts = {}
            
            for metadata in all_results["metadatas"]:
                memory_type = metadata.get("memory_type", "unknown")
                type_counts[memory_type] = type_counts.get(memory_type, 0) + 1
            
            return {
                "total_memories": count,
                "collection_name": self.collection_name,
                "persist_directory": str(self.persist_directory),
                "type_distribution": type_counts,
                "embedding_model": "all-MiniLM-L6-v2",
                "status": "active"
            }
        except Exception as e:
            return {
                "status": "error",
                "error": str(e),
                "total_memories": 0
            }
    
    def clear_all(self) -> bool:
        """Eliminar todas las memorias (cuidado!)."""
        try:
            self.client.delete_collection(self.collection_name)
            self.collection = self._get_or_create_collection()
            return True
        except Exception:
            return False


# Instancia global para compatibilidad con Unified Memory Cortex
_instance: Optional[ChromaMemoryEngine] = None


def get_chroma_memory() -> ChromaMemoryEngine:
    """Obtener instancia global del motor ChromaDB."""
    global _instance
    if _instance is None:
        _instance = ChromaMemoryEngine()
    return _instance


# Funciones de compatibilidad para Unified Memory Cortex
def _search_chromadb(query: str, limit: int = 10) -> List[Dict[str, Any]]:
    """Función de búsqueda para integración con Unified Memory Cortex."""
    chroma = get_chroma_memory()
    return chroma.search_memories(query, limit=limit)


def _stats_chromadb() -> Dict[str, Any]:
    """Función de estadísticas para Unified Memory Cortex."""
    chroma = get_chroma_memory()
    return chroma.get_statistics()


if __name__ == "__main__":
    # Test básico
    engine = ChromaMemoryEngine()
    
    # Añadir memorias de prueba
    engine.add_memory(
        content="El usuario solicitó implementar ChromaDB en Atlas para mejorar la memoria semántica",
        memory_type="episodic",
        metadata={"context": "development", "priority": "high"}
    )
    
    engine.add_memory(
        content="ChromaDB es una base de datos vectorial optimizada para embeddings",
        memory_type="semantic",
        metadata={"domain": "technology", "confidence": 0.9}
    )
    
    # Buscar
    results = engine.search_memories("memoria vectorial", limit=5)
    print("Resultados búsqueda:")
    for r in results:
        print(f"  [{r['metadata']['memory_type']}] {r['content'][:80]}... (score: {r['score']:.3f})")
    
    # Estadísticas
    stats = engine.get_statistics()
    print(f"\nEstadísticas: {stats}")

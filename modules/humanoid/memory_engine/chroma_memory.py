"""
ChromaDB Memory Engine - vector memory backend for ATLAS.
"""
from __future__ import annotations

import shutil
import uuid
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

import chromadb
from chromadb.utils import embedding_functions


class ChromaMemoryEngine:
    """
    Vector memory engine using ChromaDB with auto-recovery on common DB corruption.

    Recovery strategy:
    - Detect known SQLite/metadata-segment corruption errors.
    - Backup current store to logs/chroma_db_corrupt_<timestamp>.
    - Recreate clean persistent store and continue service.
    """

    def __init__(
        self,
        persist_directory: Optional[str] = None,
        collection_name: str = "atlas_memory",
    ):
        if persist_directory is None:
            root = Path(__file__).resolve().parents[3]
            persist_directory = str(root / "logs" / "chroma_db")

        self.persist_directory = Path(persist_directory)
        self.persist_directory.mkdir(parents=True, exist_ok=True)

        self.embedding_function = (
            embedding_functions.SentenceTransformerEmbeddingFunction(
                model_name="all-MiniLM-L6-v2"
            )
        )

        self.collection_name = collection_name
        self.client = None
        self.collection = None
        self.recovered_from_corruption = False
        self.recovery_backup_path: Optional[str] = None

        self._initialize_client_and_collection()
        print(f"ChromaDB Memory Engine iniciado en {self.persist_directory}")

    @staticmethod
    def _is_recoverable_error(exc: Exception) -> bool:
        msg = str(exc).lower()
        markers = (
            "mismatched types",
            "metadata segment",
            "error reading from metadata segment reader",
            "compaction",
            "sqlite",
            "decode",
            "backfill request",
        )
        return any(marker in msg for marker in markers)

    def _backup_and_reset_store(self) -> None:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup = self.persist_directory.parent / f"chroma_db_corrupt_{ts}"
        if self.persist_directory.exists():
            shutil.move(str(self.persist_directory), str(backup))
        self.persist_directory.mkdir(parents=True, exist_ok=True)
        self.recovered_from_corruption = True
        self.recovery_backup_path = str(backup)
        print(f"ChromaDB recovery aplicado. Backup en: {backup}")

    def _initialize_client_and_collection(self) -> None:
        try:
            self.client = chromadb.PersistentClient(path=str(self.persist_directory))
            self.collection = self._get_or_create_collection()
        except Exception as e:
            if not self._is_recoverable_error(e):
                raise
            self._backup_and_reset_store()
            self.client = chromadb.PersistentClient(path=str(self.persist_directory))
            self.collection = self._get_or_create_collection()

    def _recover_and_retry(self, exc: Exception) -> bool:
        if not self._is_recoverable_error(exc):
            return False
        self._backup_and_reset_store()
        self.client = chromadb.PersistentClient(path=str(self.persist_directory))
        self.collection = self._get_or_create_collection()
        return True

    def _get_or_create_collection(self):
        """Get or create memory collection."""
        try:
            return self.client.get_collection(
                name=self.collection_name,
                embedding_function=self.embedding_function,
            )
        except Exception:
            return self.client.create_collection(
                name=self.collection_name,
                embedding_function=self.embedding_function,
                metadata={"description": "Atlas Unified Memory"},
            )

    def add_memory(
        self,
        content: str,
        memory_type: str,
        metadata: Optional[Dict[str, Any]] = None,
        ids: Optional[str] = None,
    ) -> str:
        """Add memory item to ChromaDB."""
        if ids is None:
            ids = str(uuid.uuid4())

        base_metadata = {
            "memory_type": memory_type,
            "timestamp": datetime.now().isoformat(),
            "created_at": datetime.now().timestamp(),
        }
        if metadata:
            base_metadata.update(metadata)

        try:
            self.collection.add(documents=[content], metadatas=[base_metadata], ids=[ids])
        except Exception as e:
            if not self._recover_and_retry(e):
                raise
            self.collection.add(documents=[content], metadatas=[base_metadata], ids=[ids])
        return ids

    def search_memories(
        self,
        query: str,
        memory_types: Optional[List[str]] = None,
        limit: int = 10,
        min_score: float = 0.0,
    ) -> List[Dict[str, Any]]:
        """Search memories by semantic similarity."""
        where_clause = {}
        if memory_types:
            where_clause["memory_type"] = {"$in": memory_types}

        try:
            results = self.collection.query(
                query_texts=[query],
                n_results=limit,
                where=where_clause if where_clause else None,
            )
        except Exception as e:
            if not self._recover_and_retry(e):
                raise
            results = self.collection.query(
                query_texts=[query],
                n_results=limit,
                where=where_clause if where_clause else None,
            )

        memories = []
        for i, doc_id in enumerate(results.get("ids", [[]])[0]):
            distance = results["distances"][0][i]
            score = 1 - distance
            if score >= min_score:
                memories.append(
                    {
                        "id": doc_id,
                        "content": results["documents"][0][i],
                        "metadata": results["metadatas"][0][i],
                        "score": score,
                        "source": "chromadb",
                    }
                )
        return memories

    def get_memory_by_id(self, memory_id: str) -> Optional[Dict[str, Any]]:
        """Get memory by ID."""
        try:
            results = self.collection.get(ids=[memory_id])
            if results.get("ids"):
                return {
                    "id": results["ids"][0],
                    "content": results["documents"][0],
                    "metadata": results["metadatas"][0],
                    "source": "chromadb",
                }
        except Exception as e:
            if self._recover_and_retry(e):
                try:
                    results = self.collection.get(ids=[memory_id])
                    if results.get("ids"):
                        return {
                            "id": results["ids"][0],
                            "content": results["documents"][0],
                            "metadata": results["metadatas"][0],
                            "source": "chromadb",
                        }
                except Exception:
                    pass
        return None

    def update_memory(
        self,
        memory_id: str,
        content: Optional[str] = None,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> bool:
        """Update existing memory."""
        try:
            current = self.get_memory_by_id(memory_id)
            if not current:
                return False
            new_content = content if content is not None else current["content"]
            new_metadata = metadata if metadata is not None else current["metadata"]
            new_metadata["updated_at"] = datetime.now().timestamp()
            self.collection.update(
                ids=[memory_id],
                documents=[new_content],
                metadatas=[new_metadata],
            )
            return True
        except Exception:
            return False

    def delete_memory(self, memory_id: str) -> bool:
        """Delete memory."""
        try:
            self.collection.delete(ids=[memory_id])
            return True
        except Exception:
            return False

    def get_statistics(self) -> Dict[str, Any]:
        """Get memory engine statistics."""
        try:
            count = self.collection.count()
            all_results = self.collection.get()
            type_counts: Dict[str, int] = {}
            for metadata in all_results.get("metadatas", []):
                memory_type = metadata.get("memory_type", "unknown")
                type_counts[memory_type] = type_counts.get(memory_type, 0) + 1
            return {
                "total_memories": count,
                "collection_name": self.collection_name,
                "persist_directory": str(self.persist_directory),
                "type_distribution": type_counts,
                "embedding_model": "all-MiniLM-L6-v2",
                "status": "active",
                "recovered_from_corruption": self.recovered_from_corruption,
                "recovery_backup_path": self.recovery_backup_path,
            }
        except Exception as e:
            return {"status": "error", "error": str(e), "total_memories": 0}

    def clear_all(self) -> bool:
        """Delete all memories."""
        try:
            self.client.delete_collection(self.collection_name)
            self.collection = self._get_or_create_collection()
            return True
        except Exception:
            return False


_instance: Optional[ChromaMemoryEngine] = None


def get_chroma_memory() -> ChromaMemoryEngine:
    """Get global Chroma memory engine instance."""
    global _instance
    if _instance is None:
        _instance = ChromaMemoryEngine()
    return _instance


def _search_chromadb(query: str, limit: int = 10) -> List[Dict[str, Any]]:
    """Search adapter for Unified Memory Cortex."""
    chroma = get_chroma_memory()
    return chroma.search_memories(query, limit=limit)


def _stats_chromadb() -> Dict[str, Any]:
    """Stats adapter for Unified Memory Cortex."""
    chroma = get_chroma_memory()
    return chroma.get_statistics()


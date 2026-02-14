"""Memoria semántica con embeddings (Sentence-BERT) y búsqueda por similaridad (FAISS)."""
from __future__ import annotations

import os
import sqlite3
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

# Lazy imports for heavy deps (sentence_transformers, faiss)
_encoder = None
_faiss = None


def _get_encoder():
    global _encoder
    if _encoder is None:
        from sentence_transformers import SentenceTransformer
        _encoder = SentenceTransformer("all-MiniLM-L6-v2")
    return _encoder


def _get_faiss():
    import faiss
    return faiss


class SemanticMemory:
    """Memoria semántica con embeddings y búsqueda por similaridad."""

    def __init__(self, db_path: Optional[str] = None):
        if db_path is None:
            base = os.getenv("ATLAS_MEMORY_DB_PATH", "")
            if base:
                db_path = str(Path(base).parent / "semantic_memory.sqlite")
            else:
                db_path = "logs/semantic_memory.sqlite"
        self.db_path = Path(db_path)
        self.db_path.parent.mkdir(parents=True, exist_ok=True)

        self.encoder = _get_encoder()
        faiss = _get_faiss()
        self.dim = 384
        self.index = faiss.IndexFlatIP(self.dim)
        self._init_db()
        self._load_index()

    def _init_db(self) -> None:
        conn = sqlite3.connect(self.db_path)
        conn.execute("""
            CREATE TABLE IF NOT EXISTS experiences (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                description TEXT NOT NULL,
                context TEXT,
                outcome TEXT,
                metadata TEXT,
                tags TEXT,
                timestamp REAL,
                embedding_index INTEGER
            )
        """)
        conn.commit()
        conn.close()

    def _load_index(self) -> None:
        faiss = _get_faiss()
        index_path = self.db_path.parent / "embeddings.faiss"
        if index_path.exists():
            self.index = faiss.read_index(str(index_path))

    def _save_index(self) -> None:
        faiss = _get_faiss()
        index_path = self.db_path.parent / "embeddings.faiss"
        faiss.write_index(self.index, str(index_path))

    def add_experience(
        self,
        description: str,
        context: Optional[str] = None,
        outcome: Optional[str] = None,
        metadata: Optional[Dict[str, Any]] = None,
        tags: Optional[List[str]] = None,
    ) -> int:
        """Añade una experiencia con embedding semántico."""
        embedding = self.encoder.encode(description, convert_to_numpy=True)
        embedding = embedding.astype("float32")
        faiss = _get_faiss()
        faiss.normalize_L2(embedding.reshape(1, -1))

        embedding_index = self.index.ntotal
        self.index.add(embedding.reshape(1, -1))

        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        cursor.execute(
            """
            INSERT INTO experiences
            (description, context, outcome, metadata, tags, timestamp, embedding_index)
            VALUES (?, ?, ?, ?, ?, ?, ?)
            """,
            (
                description,
                context,
                outcome,
                str(metadata) if metadata else None,
                ",".join(tags) if tags else None,
                datetime.now().timestamp(),
                embedding_index,
            ),
        )
        exp_id = cursor.lastrowid or 0
        conn.commit()
        conn.close()

        if self.index.ntotal % 100 == 0:
            self._save_index()

        return exp_id

    def recall_similar(
        self,
        query: str,
        top_k: int = 5,
        min_similarity: float = 0.6,
        tags_filter: Optional[List[str]] = None,
    ) -> List[Dict[str, Any]]:
        """Búsqueda por similaridad semántica."""
        if self.index.ntotal == 0:
            return []
        query_embedding = self.encoder.encode(query, convert_to_numpy=True)
        query_embedding = query_embedding.astype("float32")
        faiss = _get_faiss()
        faiss.normalize_L2(query_embedding.reshape(1, -1))

        k = min(top_k * 2, max(1, self.index.ntotal))
        similarities, indices = self.index.search(
            query_embedding.reshape(1, -1), k
        )

        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()

        results: List[Dict[str, Any]] = []
        for similarity, idx in zip(similarities[0], indices[0]):
            if idx < 0 or similarity < min_similarity:
                continue

            cursor.execute(
                """
                SELECT id, description, context, outcome, metadata, tags, timestamp
                FROM experiences WHERE embedding_index = ?
                """,
                (int(idx),),
            )
            row = cursor.fetchone()
            if not row:
                continue

            exp_id, desc, ctx, out, meta, tags_str, ts = row
            exp_tags = tags_str.split(",") if tags_str else []

            if tags_filter:
                if not any(t in exp_tags for t in tags_filter):
                    continue

            try:
                meta_dict = eval(meta) if meta else {}
            except Exception:
                meta_dict = {}

            results.append({
                "id": exp_id,
                "description": desc,
                "context": ctx,
                "outcome": out,
                "metadata": meta_dict,
                "tags": exp_tags,
                "timestamp": ts,
                "similarity": float(similarity),
            })
            if len(results) >= top_k:
                break

        conn.close()
        return results

    def get_statistics(self) -> Dict[str, Any]:
        """Estadísticas de la memoria."""
        size_mb = 0.0
        if self.db_path.exists():
            size_mb = self.db_path.stat().st_size / (1024 * 1024)
        index_path = self.db_path.parent / "embeddings.faiss"
        if index_path.exists():
            size_mb += index_path.stat().st_size / (1024 * 1024)
        return {
            "total_experiences": self.index.ntotal,
            "embedding_dimension": self.dim,
            "storage_size_mb": round(size_mb, 2),
        }


# Singleton para uso desde store y API
_semantic_memory: Optional[SemanticMemory] = None


def get_semantic_memory() -> SemanticMemory:
    global _semantic_memory
    if _semantic_memory is None:
        _semantic_memory = SemanticMemory()
    return _semantic_memory

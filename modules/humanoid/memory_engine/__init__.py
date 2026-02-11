"""Persistent memory engine: threads, tasks, runs, artifacts, decisions, summaries. FTS or LIKE.

Embeddings: futuro. Búsqueda semántica (recall_by_similarity) planeada cuando exista
dependencia ligera (p. ej. Ollama embeddings o sentence-transformers); por ahora
recall_by_query con FTS/LIKE es suficiente."""
from __future__ import annotations

from .db import fts_available, new_id
from .store import (
    add_summary,
    ensure_thread,
    store_artifact,
    store_decision,
    store_plan,
    store_run,
)
from .recall import recall_by_query, recall_by_thread
from .export import export_markdown, snapshot

__all__ = [
    "fts_available", "new_id",
    "store_plan", "store_run", "store_artifact", "store_decision", "add_summary", "ensure_thread",
    "recall_by_query", "recall_by_thread",
    "export_markdown", "snapshot",
]

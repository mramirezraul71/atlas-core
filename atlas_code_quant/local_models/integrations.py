"""Safe integration helpers for local model roles."""
from __future__ import annotations

import math
from typing import Any

from .router import get_classifier_provider, get_embedding_provider, get_vision_provider


def _entry_to_text(entry: dict[str, Any]) -> str:
    fields = [
        str(entry.get("journal_key") or ""),
        str(entry.get("strategy_type") or ""),
        str(entry.get("symbol") or ""),
        str(entry.get("status") or ""),
        str(entry.get("thesis_rich_text") or ""),
        str(entry.get("post_mortem_text") or ""),
    ]
    return " | ".join(part for part in fields if part)


def _cosine(a: list[float], b: list[float]) -> float:
    if not a or not b:
        return 0.0
    n = min(len(a), len(b))
    a = a[:n]
    b = b[:n]
    dot = sum(float(x) * float(y) for x, y in zip(a, b))
    na = math.sqrt(sum(float(x) * float(x) for x in a))
    nb = math.sqrt(sum(float(y) * float(y) for y in b))
    if na <= 0 or nb <= 0:
        return 0.0
    return dot / (na * nb)


def semantic_journal_search(query: str, entries: list[dict[str, Any]], *, top_k: int = 5) -> dict[str, Any]:
    provider = get_embedding_provider()
    if not query.strip():
        return {"ok": True, "matches": [], "query": query}
    corpus = [_entry_to_text(item) for item in entries]
    vectors = provider.embed_text([query, *corpus])
    if not vectors or len(vectors) < 2:
        return {"ok": True, "matches": [], "query": query}
    qv = vectors[0]
    scored: list[dict[str, Any]] = []
    for idx, entry in enumerate(entries):
        score = _cosine(qv, vectors[idx + 1])
        scored.append({"score": round(score, 6), "entry": entry})
    scored.sort(key=lambda item: item["score"], reverse=True)
    return {"ok": True, "query": query, "matches": scored[: max(1, int(top_k))]}


def classify_journal_event(payload: dict[str, Any]) -> dict[str, Any]:
    return get_classifier_provider().classify(payload)


def analyze_dashboard_screenshot(path: str, *, prompt: str | None = None, premium: bool = False) -> dict[str, Any]:
    return get_vision_provider().analyze_screenshot(path, prompt=prompt, premium=premium)

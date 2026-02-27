"""
KnowledgeGraph - Memoria semántica: entidades, relaciones; queries y búsqueda por relación.
"""
from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)


def _load_config() -> dict:
    cfg_path = Path(__file__).resolve().parent.parent.parent / "config" / "autonomous.yaml"
    if not cfg_path.exists():
        return {}
    try:
        import yaml
        with open(cfg_path, encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    except Exception:
        return {}


class KnowledgeGraph:
    """
    add_entity(type, name, attributes); add_relation(e1, relation, e2);
    find_related(entity, relation_type, depth).
    """

    def __init__(self, config: dict | None = None):
        self._config = config or _load_config().get("learning", {})
        self._kg_cfg = self._config.get("knowledge_graph", {})
        self._max_nodes = int(self._kg_cfg.get("max_nodes", 10000))
        self._nodes: dict[str, dict] = {}
        self._edges: list[tuple[str, str, str]] = []

    def add_entity(self, entity_type: str, name: str, attributes: dict | None = None) -> None:
        """Añade nodo. id = type:name."""
        if len(self._nodes) >= self._max_nodes:
            return
        nid = f"{entity_type}:{name}"
        self._nodes[nid] = {"type": entity_type, "name": name, "attributes": attributes or {}}

    def add_relation(self, entity1: str, relation: str, entity2: str) -> None:
        """Añade arista (entity1, relation, entity2)."""
        self._edges.append((entity1, relation, entity2))

    def find_related(self, entity: str, relation_type: str | None = None, depth: int = 1) -> list[dict]:
        """Devuelve nodos relacionados por relation_type hasta depth."""
        out = []
        seen = {entity}
        frontier = [entity]
        for _ in range(depth):
            next_frontier = []
            for e in frontier:
                for a, rel, b in self._edges:
                    if a != e and b != e:
                        continue
                    if relation_type and rel != relation_type:
                        continue
                    other = b if a == e else a
                    if other in seen:
                        continue
                    seen.add(other)
                    next_frontier.append(other)
                    out.append({"entity": other, "relation": rel, "from": e})
            frontier = next_frontier
        return out

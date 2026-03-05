"""Semantic episodic memory for atlas_agent sessions."""
from __future__ import annotations

import json
import math
import sqlite3
from typing import Any, Dict, List, Tuple

try:
    from .config import AgentConfig
    from .openai_runner import OpenAIPlanner
except Exception:  # pragma: no cover - script-mode fallback
    from config import AgentConfig
    from openai_runner import OpenAIPlanner


def _dot(a: List[float], b: List[float]) -> float:
    return sum(x * y for x, y in zip(a, b))


def _norm(a: List[float]) -> float:
    return math.sqrt(sum(x * x for x in a)) or 1e-9


def _cosine(a: List[float], b: List[float]) -> float:
    if not a or not b:
        return 0.0
    n = min(len(a), len(b))
    aa = a[:n]
    bb = b[:n]
    return _dot(aa, bb) / (_norm(aa) * _norm(bb))


class EpisodicMemory:
    """SQLite-backed episodic memory with optional OpenAI embeddings."""

    def __init__(self, config: AgentConfig):
        self.config = config
        self.db_path = config.memory_db.resolve()
        self.db_path.parent.mkdir(parents=True, exist_ok=True)
        self._init_db()

    def _init_db(self) -> None:
        with sqlite3.connect(str(self.db_path)) as conn:
            conn.execute(
                """
                CREATE TABLE IF NOT EXISTS episodes (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    ts TEXT DEFAULT (datetime('now')),
                    goal TEXT NOT NULL,
                    summary TEXT NOT NULL,
                    success INTEGER NOT NULL,
                    tags TEXT,
                    embedding TEXT
                )
                """
            )
            conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_episodes_ts ON episodes(ts)"
            )
            conn.commit()

    def _embed(self, text: str) -> List[float]:
        planner = OpenAIPlanner(self.config)
        client = planner._ensure_client()
        resp = client.embeddings.create(
            model=self.config.embedding_model,
            input=text[:8000],
        )
        vector = resp.data[0].embedding  # type: ignore[index]
        return [float(x) for x in vector]

    def add_episode(
        self,
        goal: str,
        summary: str,
        success: bool,
        tags: List[str] | None = None,
    ) -> Dict[str, Any]:
        text = f"goal: {goal}\nsummary: {summary}"
        emb: List[float] = []
        error = None
        try:
            emb = self._embed(text)
        except Exception as exc:
            error = str(exc)
        with sqlite3.connect(str(self.db_path)) as conn:
            conn.execute(
                "INSERT INTO episodes(goal,summary,success,tags,embedding) VALUES(?,?,?,?,?)",
                (
                    goal[:2000],
                    summary[:4000],
                    1 if success else 0,
                    json.dumps(tags or []),
                    json.dumps(emb) if emb else None,
                ),
            )
            conn.commit()
            row_id = conn.execute("SELECT last_insert_rowid()").fetchone()[0]
        return {"ok": True, "id": row_id, "embedded": bool(emb), "embed_error": error}

    def similar(self, query: str, top_k: int = 3) -> List[Dict[str, Any]]:
        # Try semantic search first.
        q_emb: List[float] = []
        try:
            q_emb = self._embed(query)
        except Exception:
            q_emb = []
        with sqlite3.connect(str(self.db_path)) as conn:
            conn.row_factory = sqlite3.Row
            rows = conn.execute(
                "SELECT id, ts, goal, summary, success, tags, embedding FROM episodes ORDER BY id DESC LIMIT 400"
            ).fetchall()
        items: List[Tuple[float, Dict[str, Any]]] = []
        for row in rows:
            emb = []
            if row["embedding"]:
                try:
                    emb = [float(x) for x in json.loads(row["embedding"])]
                except Exception:
                    emb = []
            if q_emb and emb:
                score = _cosine(q_emb, emb)
            else:
                # lexical fallback score
                hay = (str(row["goal"]) + " " + str(row["summary"])).lower()
                terms = [t for t in query.lower().split() if len(t) > 2][:12]
                score = float(sum(1 for t in terms if t in hay))
            item = {
                "id": row["id"],
                "ts": row["ts"],
                "goal": row["goal"],
                "summary": row["summary"],
                "success": bool(row["success"]),
                "tags": json.loads(row["tags"] or "[]"),
                "score": round(score, 5),
            }
            items.append((score, item))
        items.sort(key=lambda x: x[0], reverse=True)
        return [item for _, item in items[: max(1, top_k)]]

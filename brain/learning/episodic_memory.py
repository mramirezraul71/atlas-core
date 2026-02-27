"""Memoria episódica: experiencias concretas con timestamp y resultado (aprendizaje progresivo ATLAS)."""
from __future__ import annotations

import json
import os
import sqlite3
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional


def _db_path() -> Path:
    forced = os.getenv("LEARNING_EPISODIC_DB_PATH") or os.getenv("ATLAS_LEARNING_EPISODIC_DB_PATH")
    if forced:
        return Path(forced).resolve()
    root = os.getenv("ATLAS_REPO_PATH") or os.getenv("ATLAS_PUSH_ROOT") or ""
    if root:
        return Path(root).resolve() / "logs" / "learning_episodic.sqlite"
    return Path(__file__).resolve().parent.parent.parent / "logs" / "learning_episodic.sqlite"


class EpisodicMemory:
    """
    Memoria episódica: experiencias específicas con timestamp.
    Similar a memoria humana de eventos: situación, acción, resultado, éxito/fallo.
    """

    def __init__(self, db_path: Optional[str] = None) -> None:
        self.db_path = Path(db_path) if db_path else _db_path()
        self.db_path.parent.mkdir(parents=True, exist_ok=True)
        self._init_db()

    def _init_db(self) -> None:
        """Esquema SQLite alineado con spec + compatibilidad."""
        conn = sqlite3.connect(self.db_path)
        conn.execute("""
            CREATE TABLE IF NOT EXISTS episodes (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp REAL NOT NULL,
                situation TEXT NOT NULL,
                context TEXT,
                action_taken TEXT,
                result TEXT,
                success INTEGER,
                uncertainty_score REAL DEFAULT 0.0,
                asked_for_help INTEGER DEFAULT 0,
                new_knowledge_count INTEGER DEFAULT 0,
                task_type TEXT DEFAULT 'general',
                tags TEXT,
                importance REAL DEFAULT 0.5,
                times_recalled INTEGER DEFAULT 0,
                created_at TEXT,
                metadata TEXT
            )
        """)
        conn.execute("CREATE INDEX IF NOT EXISTS idx_ep_ts ON episodes(timestamp)")
        conn.execute("CREATE INDEX IF NOT EXISTS idx_ep_task ON episodes(task_type)")
        conn.execute("CREATE INDEX IF NOT EXISTS idx_ep_success ON episodes(success)")
        conn.execute("CREATE INDEX IF NOT EXISTS idx_ep_situation ON episodes(situation)")
        # Migrar tabla antigua: si existe columna description/situation_type, añadir columnas spec
        try:
            info = conn.execute("PRAGMA table_info(episodes)").fetchall()
            names = [row[1] for row in info]
            if "situation" not in names and "description" in names:
                for col, ctype in [
                    ("situation", "TEXT"), ("context", "TEXT"), ("action_taken", "TEXT"),
                    ("result", "TEXT"), ("uncertainty_score", "REAL"), ("asked_for_help", "INTEGER"),
                    ("new_knowledge_count", "INTEGER"), ("task_type", "TEXT"), ("tags", "TEXT"),
                    ("importance", "REAL"), ("times_recalled", "INTEGER"), ("created_at", "TEXT"),
                    ("metadata", "TEXT"),
                ]:
                    if col not in names:
                        conn.execute(f"ALTER TABLE episodes ADD COLUMN {col} {ctype}")
        except Exception:
            pass
        conn.commit()
        conn.close()

    def add_episode(
        self,
        situation: Optional[str] = None,
        context: Optional[Dict[str, Any]] = None,
        action_taken: Optional[str] = None,
        result: Any = None,
        success: Optional[bool] = None,
        uncertainty_score: float = 0.0,
        asked_for_help: bool = False,
        new_knowledge_count: int = 0,
        task_type: str = "general",
        tags: Optional[List[str]] = None,
        importance: float = 0.5,
        # Compatibilidad con loop/consolidator
        situation_type: Optional[str] = None,
        description: Optional[str] = None,
        action: Optional[str] = None,
        outcome: Optional[str] = None,
    ) -> int:
        """
        Registrar episodio. Acepta firma spec (situation, context, action_taken, result, success, ...)
        o firma corta (situation_type, description, action, outcome, success, context).
        Retorna episode_id.
        """
        if description is not None or outcome is not None or situation_type is not None:
            situation = situation or description or ""
            action_taken = action_taken or action or ""
            result = result if result is not None else outcome
            if isinstance(result, dict):
                result = json.dumps(result, ensure_ascii=False)
            else:
                result = str(result) if result is not None else ""
            task_type = task_type if task_type != "general" else (situation_type or "general")
            success = success if success is not None else True
        else:
            situation = situation or ""
            action_taken = action_taken or ""
            if isinstance(result, dict):
                result = json.dumps(result, ensure_ascii=False)
            else:
                result = str(result) if result is not None else ""
            success = bool(success) if success is not None else True

        ctx_str = json.dumps(context or {}, ensure_ascii=False)[:5000]
        now_ts = time.time()
        now_iso = datetime.now().isoformat()
        meta = {}
        if isinstance(result, str) and result:
            try:
                meta = {"result_summary": result[:200], "confidence": 0.0}
            except Exception:
                pass
        metadata_str = json.dumps(meta, ensure_ascii=False)

        conn = sqlite3.connect(self.db_path)
        conn.execute(
            """
            INSERT INTO episodes (
                timestamp, situation, context, action_taken, result, success,
                uncertainty_score, asked_for_help, new_knowledge_count, task_type,
                tags, importance, created_at, metadata
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                now_ts,
                (situation or "")[:2000],
                ctx_str,
                (action_taken or "")[:500],
                (result or "")[:2000] if isinstance(result, str) else str(result)[:2000],
                1 if success else 0,
                uncertainty_score,
                1 if asked_for_help else 0,
                new_knowledge_count,
                (task_type or "general")[:200],
                ",".join(tags) if tags else "",
                importance,
                now_iso,
                metadata_str,
            ),
        )
        episode_id = conn.execute("SELECT last_insert_rowid()").fetchone()[0]
        conn.commit()
        conn.close()
        return episode_id

    def get_recent(self, limit: int = 100) -> List[Dict[str, Any]]:
        """Últimos episodios (compatibilidad con consolidator)."""
        return self.get_recent_episodes(limit=limit)

    def get_recent_episodes(
        self,
        limit: int = 10,
        task_type: Optional[str] = None,
        only_failures: bool = False,
    ) -> List[Dict[str, Any]]:
        """Obtener episodios recientes, opcionalmente por task_type o solo fallos."""
        conn = sqlite3.connect(self.db_path)
        conn.row_factory = sqlite3.Row
        cursor = conn.cursor()
        query = "SELECT * FROM episodes WHERE 1=1"
        params: List[Any] = []
        if task_type:
            query += " AND task_type = ?"
            params.append(task_type)
        if only_failures:
            query += " AND success = 0"
        query += " ORDER BY timestamp DESC LIMIT ?"
        params.append(limit)
        cursor.execute(query, params)
        rows = cursor.fetchall()
        conn.close()
        return [_row_to_dict(r) for r in rows]

    def get_by_situation_type(
        self, situation_type: str, limit: int = 50
    ) -> List[Dict[str, Any]]:
        """Episodios por tipo de situación (compatibilidad consolidator)."""
        return self.get_recent_episodes(limit=limit, task_type=situation_type)

    def get_similar_episodes(
        self,
        situation: str,
        task_type: Optional[str] = None,
        limit: int = 5,
    ) -> List[Dict[str, Any]]:
        """Buscar episodios similares por palabras clave. TODO: integrar búsqueda semántica."""
        if not situation.strip():
            return self.get_recent_episodes(limit=limit, task_type=task_type)
        conn = sqlite3.connect(self.db_path)
        conn.row_factory = sqlite3.Row
        cursor = conn.cursor()
        pattern = f"%{situation}%"
        query = """
            SELECT *, CASE WHEN situation LIKE ? THEN 1.0 ELSE 0.5 END AS similarity
            FROM episodes WHERE situation LIKE ?
        """
        params: List[Any] = [pattern, pattern]
        if task_type:
            query += " AND task_type = ?"
            params.append(task_type)
        query += " ORDER BY similarity DESC, times_recalled DESC LIMIT ?"
        params.append(limit)
        cursor.execute(query, params)
        rows = cursor.fetchall()
        for row in rows:
            cursor.execute(
                "UPDATE episodes SET times_recalled = times_recalled + 1 WHERE id = ?",
                (row["id"],),
            )
        conn.commit()
        conn.close()
        return [_row_to_dict(r) for r in rows]

    def get_learning_curve(self, task_type: str) -> Dict[str, Any]:
        """Curva de aprendizaje para un tipo de tarea."""
        conn = sqlite3.connect(self.db_path)
        conn.row_factory = sqlite3.Row
        cursor = conn.cursor()
        cursor.execute(
            "SELECT * FROM episodes WHERE task_type = ? ORDER BY timestamp ASC",
            (task_type,),
        )
        episodes = [_row_to_dict(r) for r in cursor.fetchall()]
        conn.close()
        if not episodes:
            return {
                "total_attempts": 0,
                "success_rate": 0.0,
                "improvement_trend": "no_data",
                "episodes": [],
            }
        total = len(episodes)
        successes = sum(1 for e in episodes if e.get("success"))
        success_rate = successes / total
        mid = total // 2
        first_half = sum(1 for e in episodes[:mid] if e.get("success")) / mid if mid else 0
        second_half = (
            sum(1 for e in episodes[mid:] if e.get("success")) / (total - mid)
            if total > mid else 0
        )
        if second_half > first_half + 0.1:
            trend = "improving"
        elif second_half < first_half - 0.1:
            trend = "declining"
        else:
            trend = "stable"
        return {
            "total_attempts": total,
            "success_rate": success_rate,
            "first_half_success_rate": first_half,
            "second_half_success_rate": second_half,
            "improvement_trend": trend,
            "recent_episodes": episodes[-5:],
        }

    def get_success_failure_counts(
        self,
        situation_type: Optional[str] = None,
        action: Optional[str] = None,
    ) -> Dict[str, int]:
        """Conteos éxito/fallo (consolidator)."""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        query = "SELECT success, COUNT(*) FROM episodes WHERE 1=1"
        params: List[Any] = []
        if situation_type:
            query += " AND task_type = ?"
            params.append(situation_type)
        if action:
            query += " AND action_taken LIKE ?"
            params.append(f"%{action}%")
        query += " GROUP BY success"
        cursor.execute(query, params)
        rows = cursor.fetchall()
        conn.close()
        counts = {"success": 0, "failure": 0}
        for success, cnt in rows:
            if success:
                counts["success"] = cnt
            else:
                counts["failure"] = cnt
        return counts

    def get_statistics(self) -> Dict[str, Any]:
        """Estadísticas de memoria episódica (spec + growth-metrics)."""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        cursor.execute("SELECT COUNT(*) FROM episodes")
        total = cursor.fetchone()[0]
        cursor.execute("SELECT COUNT(*) FROM episodes WHERE success = 1")
        success_count = cursor.fetchone()[0]
        cursor.execute("SELECT AVG(success) FROM episodes")
        success_rate = cursor.fetchone()[0] or 0
        cursor.execute("SELECT SUM(asked_for_help) FROM episodes")
        help_count = cursor.fetchone()[0] or 0
        cursor.execute("SELECT SUM(new_knowledge_count) FROM episodes")
        total_knowledge = cursor.fetchone()[0] or 0
        cursor.execute(
            """
            SELECT task_type, COUNT(*) AS cnt FROM episodes
            GROUP BY task_type ORDER BY cnt DESC LIMIT 5
            """
        )
        top_tasks = cursor.fetchall()
        conn.close()
        size_mb = self.db_path.stat().st_size / (1024 * 1024) if self.db_path.exists() else 0
        return {
            "total_episodes": total,
            "success_count": success_count,
            "failure_count": total - success_count,
            "storage_mb": round(size_mb, 2),
            "overall_success_rate": success_rate,
            "times_asked_for_help": help_count,
            "help_rate": help_count / total if total > 0 else 0,
            "total_knowledge_learned": total_knowledge,
            "top_task_types": [{"task": t[0], "count": t[1]} for t in top_tasks],
        }

    def update_episode(
        self,
        episode_id: int,
        *,
        action_taken: Optional[str] = None,
        result: Any = None,
        success: Optional[bool] = None,
        uncertainty_score: Optional[float] = None,
        asked_for_help: Optional[bool] = None,
        new_knowledge_count: Optional[int] = None,
        tags: Optional[List[str]] = None,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> bool:
        """
        Actualiza un episodio existente con el resultado final.
        Retorna True si actualizó 1 fila.
        """
        fields = []
        params: List[Any] = []
        if action_taken is not None:
            fields.append("action_taken = ?")
            params.append(str(action_taken)[:500])
        if result is not None:
            if isinstance(result, dict):
                result_str = json.dumps(result, ensure_ascii=False)
            else:
                result_str = str(result)
            fields.append("result = ?")
            params.append(result_str[:2000])
        if success is not None:
            fields.append("success = ?")
            params.append(1 if bool(success) else 0)
        if uncertainty_score is not None:
            fields.append("uncertainty_score = ?")
            params.append(float(uncertainty_score))
        if asked_for_help is not None:
            fields.append("asked_for_help = ?")
            params.append(1 if bool(asked_for_help) else 0)
        if new_knowledge_count is not None:
            fields.append("new_knowledge_count = ?")
            params.append(int(new_knowledge_count))
        if tags is not None:
            fields.append("tags = ?")
            params.append(",".join(tags)[:1000] if tags else "")
        if metadata is not None:
            fields.append("metadata = ?")
            params.append(json.dumps(metadata, ensure_ascii=False)[:5000])

        if not fields:
            return False

        params.append(int(episode_id))
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        cursor.execute(f"UPDATE episodes SET {', '.join(fields)} WHERE id = ?", params)
        conn.commit()
        updated = cursor.rowcount == 1
        conn.close()
        return updated


def _row_to_dict(r: sqlite3.Row) -> Dict[str, Any]:
    d = dict(r)
    for key in list(d.keys()):
        if key == "context" and d.get(key):
            try:
                d["context"] = json.loads(d["context"])
            except Exception:
                pass
        elif key == "result" and d.get(key):
            try:
                val = d["result"]
                d["result"] = json.loads(val) if isinstance(val, str) and val.strip().startswith("{") else val
            except Exception:
                pass
        elif key == "metadata" and d.get(key):
            try:
                d["metadata"] = json.loads(d["metadata"])
            except Exception:
                d["metadata"] = None
    d["success"] = bool(d.get("success"))
    d["asked_for_help"] = bool(d.get("asked_for_help"))
    # Alias para compatibilidad con consolidador/loop (situation_type, action, description, outcome)
    d["situation_type"] = d.get("task_type") or d.get("situation_type") or "general"
    d["action"] = d.get("action_taken") or d.get("action") or ""
    d["description"] = d.get("situation") or d.get("description") or ""
    d["outcome"] = d.get("result") or d.get("outcome") or ""
    return d

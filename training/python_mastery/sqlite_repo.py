from __future__ import annotations

import sqlite3
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional


@dataclass(frozen=True)
class Item:
    key: str
    value: str


class SQLiteRepo:
    """PY010: repositorio SQLite offline-first con transacciones."""

    def __init__(self, db_path: Path):
        self.db_path = Path(db_path)
        self.db_path.parent.mkdir(parents=True, exist_ok=True)
        self._init_db()

    def _connect(self) -> sqlite3.Connection:
        conn = sqlite3.connect(str(self.db_path))
        conn.execute("PRAGMA journal_mode=WAL;")
        conn.execute("PRAGMA synchronous=NORMAL;")
        conn.execute("PRAGMA foreign_keys=ON;")
        return conn

    def _init_db(self) -> None:
        with self._connect() as conn:
            conn.execute(
                """
                CREATE TABLE IF NOT EXISTS items (
                    key TEXT PRIMARY KEY,
                    value TEXT NOT NULL,
                    created_ts TEXT DEFAULT CURRENT_TIMESTAMP
                )
                """
            )
            conn.execute("CREATE INDEX IF NOT EXISTS idx_items_created ON items(created_ts);")

    def upsert_item(self, key: str, value: str) -> None:
        if not key:
            raise ValueError("key requerida")
        with self._connect() as conn:
            conn.execute(
                "INSERT INTO items(key, value) VALUES(?, ?) ON CONFLICT(key) DO UPDATE SET value=excluded.value",
                (key, value),
            )

    def get_item(self, key: str) -> Optional[Item]:
        with self._connect() as conn:
            row = conn.execute("SELECT key, value FROM items WHERE key = ?", (key,)).fetchone()
            return Item(key=row[0], value=row[1]) if row else None

    def list_items(self, limit: int = 50) -> List[Item]:
        with self._connect() as conn:
            rows = conn.execute(
                "SELECT key, value FROM items ORDER BY created_ts DESC LIMIT ?",
                (int(limit),),
            ).fetchall()
            return [Item(key=r[0], value=r[1]) for r in rows]


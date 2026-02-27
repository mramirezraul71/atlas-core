from __future__ import annotations

import os
from pathlib import Path

import pytest


def _enabled() -> bool:
    return os.getenv("RUN_PYTHON_MASTERY", "").strip().lower() in ("1", "true", "yes", "on")


pytestmark = pytest.mark.skipif(not _enabled(), reason="Python Mastery tests deshabilitados (set RUN_PYTHON_MASTERY=1).")


def test_sqlite_repo_crud(tmp_path: Path):
    from training.python_mastery.sqlite_repo import SQLiteRepo

    db = tmp_path / "db.sqlite"
    repo = SQLiteRepo(db)
    repo.upsert_item("a", "1")
    repo.upsert_item("b", "2")
    repo.upsert_item("a", "3")

    assert repo.get_item("a").value == "3"
    items = repo.list_items(limit=10)
    keys = {i.key for i in items}
    assert keys.issuperset({"a", "b"})


from __future__ import annotations

import json
import os
from pathlib import Path

import pytest


def _enabled() -> bool:
    return os.getenv("RUN_PYTHON_MASTERY", "").strip().lower() in ("1", "true", "yes", "on")


pytestmark = pytest.mark.skipif(not _enabled(), reason="Python Mastery tests deshabilitados (set RUN_PYTHON_MASTERY=1).")


def test_atomic_write_json_writes_valid_json(tmp_path: Path):
    from training.python_mastery.io_utils import atomic_write_json

    p = tmp_path / "out.json"
    atomic_write_json(p, {"a": 1, "b": "ATLAS"})

    data = json.loads(p.read_text(encoding="utf-8"))
    assert data["a"] == 1
    assert data["b"] == "ATLAS"


from __future__ import annotations

import json
import os
from pathlib import Path

import pytest


def _enabled() -> bool:
    return os.getenv("RUN_PYTHON_MASTERY", "").strip().lower() in ("1", "true", "yes", "on")


pytestmark = pytest.mark.skipif(not _enabled(), reason="Python Mastery tests deshabilitados (set RUN_PYTHON_MASTERY=1).")


def test_cli_inventory_json_output(tmp_path: Path, capsys):
    # Arrange
    (tmp_path / "a.txt").write_text("hi", encoding="utf-8")

    from training.python_mastery.cli import main

    # Act
    code = main(["inventory", "--root", str(tmp_path), "--json"])
    out = capsys.readouterr().out.strip()

    # Assert
    assert code == 0
    payload = json.loads(out)
    assert "total_files" in payload
    assert "total_size_bytes" in payload
    assert "top_extensions" in payload


from __future__ import annotations

import os
from pathlib import Path

import pytest


def _enabled() -> bool:
    return os.getenv("RUN_PYTHON_MASTERY", "").strip().lower() in ("1", "true", "yes", "on")


pytestmark = pytest.mark.skipif(not _enabled(), reason="Python Mastery tests deshabilitados (set RUN_PYTHON_MASTERY=1).")


def test_scan_inventory_counts_and_exts(tmp_path: Path):
    # Arrange
    (tmp_path / "a.txt").write_text("hi", encoding="utf-8")
    (tmp_path / "b.TXT").write_text("hi", encoding="utf-8")
    (tmp_path / "c").write_text("hi", encoding="utf-8")  # no ext
    d = tmp_path / "sub"
    d.mkdir()
    (d / "x.md").write_text("hello", encoding="utf-8")

    from training.python_mastery.inventory import scan_inventory

    # Act
    res = scan_inventory(tmp_path, min_size_kb=0)

    # Assert
    assert res.total_files == 4
    assert res.total_size_bytes > 0
    exts = dict(res.top_extensions)
    assert exts[".txt"] == 2
    assert exts[".md"] == 1
    assert exts["<noext>"] == 1


from __future__ import annotations

import os

import pytest


def _enabled() -> bool:
    return os.getenv("RUN_PYTHON_MASTERY", "").strip().lower() in ("1", "true", "yes", "on")


pytestmark = pytest.mark.skipif(not _enabled(), reason="Python Mastery tests deshabilitados (set RUN_PYTHON_MASTERY=1).")


def test_parse_args_and_build_payload():
    from training.python_mastery.app_refactor import build_inventory_payload, parse_args

    cfg = parse_args(["--root", "X", "--min-size-kb", "5", "--dry-run"])
    assert str(cfg.root) == "X"
    assert cfg.min_size_kb == 5
    assert cfg.dry_run is True

    payload = build_inventory_payload(2, 10, [(".txt", 2)])
    assert payload["total_files"] == 2
    assert payload["total_size_bytes"] == 10
    assert payload["top_extensions"] == [(".txt", 2)]


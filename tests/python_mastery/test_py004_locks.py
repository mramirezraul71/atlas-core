from __future__ import annotations

import os
import threading
import time
from pathlib import Path

import pytest


def _enabled() -> bool:
    return os.getenv("RUN_PYTHON_MASTERY", "").strip().lower() in ("1", "true", "yes", "on")


pytestmark = pytest.mark.skipif(not _enabled(), reason="Python Mastery tests deshabilitados (set RUN_PYTHON_MASTERY=1).")


def test_file_lock_exclusive_timeout(tmp_path: Path):
    from training.python_mastery.locks import file_lock

    target = tmp_path / "resource.txt"
    target.write_text("x", encoding="utf-8")

    started = threading.Event()
    done = threading.Event()
    results: dict = {}

    def worker():
        started.set()
        try:
            with file_lock(target, timeout_s=0.2):
                results["acquired"] = True
        except Exception as e:
            results["error"] = type(e).__name__
        finally:
            done.set()

    with file_lock(target, timeout_s=1.0):
        t = threading.Thread(target=worker, daemon=True)
        t.start()
        assert started.wait(1.0)
        assert done.wait(2.0)
        assert results.get("acquired") is not True
        assert results.get("error") in ("TimeoutError",)


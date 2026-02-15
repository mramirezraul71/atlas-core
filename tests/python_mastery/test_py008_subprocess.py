from __future__ import annotations

import os
import subprocess

import pytest


def _enabled() -> bool:
    return os.getenv("RUN_PYTHON_MASTERY", "").strip().lower() in ("1", "true", "yes", "on")


pytestmark = pytest.mark.skipif(not _enabled(), reason="Python Mastery tests deshabilitados (set RUN_PYTHON_MASTERY=1).")


def test_run_cmd_success():
    from training.python_mastery.subprocess_utils import run_cmd

    res = run_cmd(["python", "-c", "print('ok')"], timeout_s=5, check=True)
    assert res.returncode == 0
    assert "ok" in res.stdout


def test_run_cmd_nonzero_raises():
    from training.python_mastery.subprocess_utils import run_cmd

    with pytest.raises(subprocess.CalledProcessError):
        run_cmd(["python", "-c", "import sys; sys.exit(2)"], timeout_s=5, check=True)


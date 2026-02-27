from __future__ import annotations

import os
from pathlib import Path

import pytest


def _enabled() -> bool:
    return os.getenv("RUN_PYTHON_MASTERY", "").strip().lower() in ("1", "true", "yes", "on")


pytestmark = pytest.mark.skipif(not _enabled(), reason="Python Mastery tests deshabilitados (set RUN_PYTHON_MASTERY=1).")


def test_packaging_sample_has_pyproject_with_scripts():
    import tomllib

    base = Path("training/python_mastery/packaging_sample")
    pyproject = base / "pyproject.toml"
    data = tomllib.loads(pyproject.read_text(encoding="utf-8"))

    assert data["project"]["name"] == "atlas-py-tool"
    assert data["project"]["version"]
    assert data["project"]["requires-python"]
    scripts = data["project"]["scripts"]
    assert scripts["atlas-py-tool"] == "atlas_py_tool.cli:main"


def test_packaging_sample_cli_main_smoke(capsys):
    from training.python_mastery.packaging_sample.atlas_py_tool.cli import main

    code = main(["--ping"])
    out = capsys.readouterr().out.strip()
    assert code == 0
    assert out == "pong"


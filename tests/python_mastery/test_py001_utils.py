from __future__ import annotations

import os

import pytest


def _enabled() -> bool:
    return os.getenv("RUN_PYTHON_MASTERY", "").strip().lower() in ("1", "true", "yes", "on")


pytestmark = pytest.mark.skipif(not _enabled(), reason="Python Mastery tests deshabilitados (set RUN_PYTHON_MASTERY=1).")


def test_normalize_text_rules():
    from training.python_mastery.utils import normalize_text

    assert normalize_text("  Hola   Mundo ") == "hola mundo"
    assert normalize_text("\nATLAS\tPYTHON  ") == "atlas python"


def test_chunk_list():
    from training.python_mastery.utils import chunk_list

    assert chunk_list([1, 2, 3, 4, 5], 2) == [[1, 2], [3, 4], [5]]


def test_safe_get_nested():
    from training.python_mastery.utils import safe_get

    d = {"a": {"b": {"c": 7}}}
    assert safe_get(d, "a.b.c") == 7
    assert safe_get(d, "a.b.x", default=9) == 9


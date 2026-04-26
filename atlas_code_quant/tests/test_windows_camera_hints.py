"""Pruebas ligeras del helper PnP Windows (sin hardware real)."""
from __future__ import annotations

import sys

import pytest

from atlas_code_quant.vision import windows_camera_hints


def test_fetch_pnp_empty_on_non_windows():
    if sys.platform == "win32":
        pytest.skip("solo validacion no-Windows en CI")
    assert windows_camera_hints.fetch_pnp_camera_hints(timeout_sec=0.5) == []

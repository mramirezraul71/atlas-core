"""Windows PnP hints para cámara (Atlas Code-Quant).

En Linux/CI: solo se valida el contrato ``sys.platform != win32`` → lista vacía.
En Windows: humo real de ``fetch_pnp_camera_hints`` (no debe lanzar).
"""
from __future__ import annotations

import sys

import pytest

# Módulo real: la auditoría citó adapter; la implementación vive en ``atlas_code_quant.vision``.
from atlas_code_quant.vision import windows_camera_hints


def test_fetch_pnp_empty_on_non_windows() -> None:
    """Fuera de Windows el módulo no invoca PowerShell y devuelve []."""
    if sys.platform == "win32":
        pytest.skip("Cubierto por test_win32")
    out = windows_camera_hints.fetch_pnp_camera_hints(timeout_sec=0.5)
    assert out == []
    assert isinstance(out, list)


@pytest.mark.skipif(
    sys.platform != "win32",
    reason="Solo Windows (Get-PnpDevice vía PowerShell)",
)
def test_fetch_pnp_camera_hints_win32_smoke() -> None:
    """En Windows, la enumeración PnP devuelve una lista de dicts o [] si falla silencioso."""
    out = windows_camera_hints.fetch_pnp_camera_hints(timeout_sec=2.0)
    assert isinstance(out, list)
    for row in out[:5]:
        assert isinstance(row, dict)
        assert "name" in row
        assert "class" in row

from __future__ import annotations

import importlib
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def test_atlas_core_manifest_exists_and_is_valid_json() -> None:
    manifest = ROOT / "atlas_core" / "MANIFEST.json"
    assert manifest.is_file(), f"missing {manifest}"
    data = json.loads(manifest.read_text(encoding="utf-8"))
    assert isinstance(data.get("modules"), list)
    assert len(data["modules"]) >= 1


@pytest.mark.parametrize(
    "module_name",
    [
        "atlas_core",
        "atlas_core.autonomy",
        "atlas_core.brain",
        "atlas_core.adapters",
        "atlas_core.runtime",
    ],
)
def test_atlas_core_manifest_modules_import(module_name: str) -> None:
    importlib.import_module(module_name)


def test_atlas_core_shim_points_to_repo_package() -> None:
    """Tras cargar el shim, el paquete atlas_core resuelve al árbol atlas_core/ en la raíz del repo."""
    import importlib

    import atlas_code_quant.atlas_core  # noqa: F401  — efecto lateral: registra atlas_core canónico

    ac = importlib.import_module("atlas_core")
    canonical_dir = (ROOT / "atlas_core").resolve()
    assert Path(ac.__file__).resolve().parent == canonical_dir

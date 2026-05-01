"""Shim: redirige imports a `atlas_core/` en la raíz del repositorio (núcleo canónico).

Documentación y lista de módulos obligatorios: `atlas_core/CANONICAL.md`, `atlas_core/MANIFEST.json`.
Verificación local: `python atlas_code_quant/scripts/verify_atlas_core_manifest.py`.
"""

from __future__ import annotations

import importlib
import importlib.util
import sys
from pathlib import Path


def _load_canonical() -> object:
    repo_root = Path(__file__).resolve().parents[2]
    canonical_pkg = repo_root / "atlas_core"
    canonical_init = canonical_pkg / "__init__.py"
    if not canonical_init.exists():
        raise ModuleNotFoundError(f"No existe atlas_core canónico en: {canonical_pkg}")

    current = sys.modules.get("atlas_core")
    if current is not None and getattr(current, "__file__", "") == str(canonical_init):
        return current

    spec = importlib.util.spec_from_file_location(
        "atlas_core",
        canonical_init,
        submodule_search_locations=[str(canonical_pkg)],
    )
    if spec is None or spec.loader is None:
        raise ModuleNotFoundError("No se pudo construir el spec de atlas_core canónico")
    module = importlib.util.module_from_spec(spec)
    sys.modules["atlas_core"] = module
    spec.loader.exec_module(module)
    return module


_canonical = _load_canonical()
if __name__ == "atlas_core":
    globals().update(_canonical.__dict__)
else:
    for _pkg in ("autonomy", "brain", "adapters", "runtime"):
        _mod = importlib.import_module(f"atlas_core.{_pkg}")
        sys.modules[f"{__name__}.{_pkg}"] = _mod

    __path__ = list(getattr(_canonical, "__path__", []))  # type: ignore[assignment]
    __all__ = list(getattr(_canonical, "__all__", ["autonomy", "brain"]))  # type: ignore[assignment]

from __future__ import annotations

from pathlib import Path


def test_execute_order_governed_returns_plan(tmp_path: Path):
    import os
    from modules.atlas_architect.agent import AtlasArchitect

    # Aislar approvals DB dentro del tmp del test (evita ensuciar logs/atlas_approvals.sqlite del repo).
    os.environ["ATLAS_APPROVALS_DB_PATH"] = str(tmp_path / "atlas_approvals.sqlite")

    arch = AtlasArchitect(repo_root=tmp_path)
    out = arch.execute_order("Diseña una app de inventario para la panadería Rauli", mode="governed", prefer_free=True)
    assert out["mode"] == "governed"
    assert out["plan"]["changes"]
    # debe contener al menos un archivo backend
    paths = [c["path"] for c in out["plan"]["changes"]]
    assert any("backend" in p.replace("\\", "/") for p in paths)


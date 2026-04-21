"""Procedencia efectiva ``atlas_core`` (reproducibilidad mínima, sin migración de repo)."""
from __future__ import annotations

import importlib
import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest

from atlas_code_quant.atlas_core_provenance import (
    attach_atlas_core_provenance_to_plan,
    get_last_atlas_core_provenance,
    repo_root_from_atlas_code_quant,
    resolve_atlas_core_provenance,
)


@pytest.fixture
def restore_atlas_core_module():
    orig = sys.modules.get("atlas_core")
    yield
    if orig is None:
        sys.modules.pop("atlas_core", None)
    else:
        sys.modules["atlas_core"] = orig


def test_resolve_ok_repo_local() -> None:
    importlib.import_module("atlas_code_quant.atlas_core")
    out = resolve_atlas_core_provenance()
    assert out["exists"] is True
    assert out["importable"] is True
    assert out["source_kind"] == "repo_local"
    assert out["canonical_status"] in {"ok", "ambiguous"}
    assert out["resolved_path"]
    p = Path(out["resolved_path"])
    assert p == (repo_root_from_atlas_code_quant() / "atlas_core").resolve()
    assert out["timestamp"]
    snap = get_last_atlas_core_provenance()
    assert snap == out


def test_resolve_repo_root_error(monkeypatch: pytest.MonkeyPatch) -> None:
    def _boom() -> Path:
        raise RuntimeError("no_repo")

    monkeypatch.setattr(
        "atlas_code_quant.atlas_core_provenance.repo_root_from_atlas_code_quant",
        _boom,
    )
    out = resolve_atlas_core_provenance()
    assert out["canonical_status"] == "error"
    assert out["exists"] is False
    assert out["importable"] is False
    assert out["resolved_path"] is None


def test_resolve_missing_canonical_under_fake_root(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    """Sin árbol canónico bajo el repo_root declarado → missing (sin módulo precargado)."""
    fake_root = tmp_path / "repo"
    fake_root.mkdir()
    monkeypatch.setattr(
        "atlas_code_quant.atlas_core_provenance.repo_root_from_atlas_code_quant",
        lambda: fake_root.resolve(),
    )
    sys.modules.pop("atlas_core", None)
    for k in list(sys.modules):
        if k.startswith("atlas_core."):
            del sys.modules[k]
    out = resolve_atlas_core_provenance(ensure_shim_loaded=False)
    try:
        assert out["exists"] is False
        assert out["canonical_status"] in ("missing", "ambiguous")
        if out["importable"]:
            assert out["canonical_status"] == "ambiguous"
    finally:
        importlib.import_module("atlas_code_quant.atlas_core")


def test_resolve_ambiguous_extra_trees_on_disk(monkeypatch: pytest.MonkeyPatch) -> None:
    importlib.import_module("atlas_code_quant.atlas_core")

    monkeypatch.setattr(
        "atlas_code_quant.atlas_core_provenance._extra_atlas_core_package_dirs",
        lambda _repo: [Path("/tmp/phantom_atlas_core_stub")],
    )
    out = resolve_atlas_core_provenance()
    assert out["canonical_status"] == "ambiguous"
    assert "extra_atlas_core_dirs_on_disk" in " ".join(out.get("notes") or [])


def test_resolve_mismatch_fake_atlas_core_module(
    restore_atlas_core_module, tmp_path: Path
) -> None:
    bogus = tmp_path / "foreign_atlas_core"
    bogus.mkdir()
    (bogus / "__init__.py").write_text("# shadow\n")
    fake = types.ModuleType("atlas_core")
    fake.__file__ = str(bogus / "__init__.py")
    sys.modules["atlas_core"] = fake
    out = resolve_atlas_core_provenance(ensure_shim_loaded=False)
    assert out["canonical_status"] == "ambiguous"
    assert out["importable"] is True
    assert Path(out["resolved_path"]) == bogus.resolve()


def test_attach_to_plan_and_snapshot() -> None:
    plan: dict = {"symbol": "X", "automation_mode": "paper_only"}
    attach_atlas_core_provenance_to_plan(plan)
    assert "atlas_core_provenance" in plan
    assert plan["atlas_core_provenance"]["timestamp"]
    assert get_last_atlas_core_provenance() == plan["atlas_core_provenance"]


def test_paper_session_plan_includes_provenance(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.chdir(tmp_path)
    from atlas_code_quant.options.options_intent_router import OptionsIntentRouter
    from atlas_code_quant.options.paper_entry_planner import PaperEntryPlanner
    from atlas_code_quant.options.paper_session_orchestrator import PaperSessionOrchestrator
    from atlas_code_quant.options.session_briefing import SessionBriefingEngine

    iv = MagicMock()
    iv.get_iv_rank.return_value = {
        "symbol": "SPX",
        "iv_current": 0.18,
        "iv_rank": 40.0,
        "iv_hv_ratio": 1.05,
        "quality": "ok",
        "spot": 5200.0,
        "method": "test",
        "expiration": "2026-06-19",
        "dte": 30,
    }
    engine = SessionBriefingEngine(iv, pick_strategy_fn=lambda *a, **k: "iron_condor")
    orch = PaperSessionOrchestrator(engine, OptionsIntentRouter(), PaperEntryPlanner())
    out = orch.build_session_plan(
        symbol="SPX",
        direction="neutral",
        regime="ranging",
        gamma_regime="long_gamma",
        dte_mode="8to21",
    )
    prov = out.get("atlas_core_provenance")
    assert isinstance(prov, dict)
    assert prov.get("resolved_path")
    assert prov.get("canonical_status") in ("ok", "ambiguous", "partial", "missing", "error", "unknown")
